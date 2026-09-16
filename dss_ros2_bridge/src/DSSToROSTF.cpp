/*
    DSS To ROS TF Bridge
*/

#include <rclcpp/rclcpp.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <nats/nats.h>

#include "dss.pb.h"
#include "defaultGateway.h"

#include "nlohmann/json.hpp"

using json = nlohmann::json;

#define MAX_SUBS (64)


// ==================== NATS 클라이언트 ====================

struct NatsClient
{
    natsConnection*      conn = nullptr;
    natsSubscription*    subs[MAX_SUBS]{};
    int                  count = 0;
};


// ==================== ROS Node ====================

class DSSToROSTFNode : public rclcpp::Node
{
public:

    using TopicHandler  = std::function<void(const std::string&, const char*, int)>;
    struct TopicCtx     { TopicHandler* fn; };

    NatsClient nats_;

    std::vector<std::unique_ptr<TopicHandler>> topicHandlers_;
    std::vector<std::unique_ptr<TopicCtx>> rawCtx_;

    rclcpp::TimerBase::SharedPtr timer_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

public:

    DSSToROSTFNode() : Node("DSSToROSTFNode")
    {
        std::string kNatsUrl = "nats://" + getDefaultGateway() + ":4222";

        RCLCPP_INFO(get_logger(), "Connecting NATS %s", kNatsUrl.c_str());

        natsStatus s = natsConnection_ConnectTo(&nats_.conn, kNatsUrl.c_str());

        if (s != NATS_OK)
        {
            RCLCPP_ERROR(get_logger(), "NATS connect failed: %s", natsStatus_GetText(s));
            return;
        }

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        subscribeTopicRaw("dss.tf",
            [this](const std::string&, const char* bytes, int len)
            {
                dss::DSSTF tf_msg;

                if (!tf_msg.ParseFromArray(bytes, len))
                {
                    RCLCPP_ERROR(get_logger(), "Failed to parse DSSTF protobuf");
                    return;
                }

                PublishTF(tf_msg);
            }
        );

        timer_ = this->create_wall_timer(std::chrono::seconds(3),std::bind(&DSSToROSTFNode::onTick, this));

        RCLCPP_INFO(get_logger(), "[NATS] dss.tf → [ROS2] /tf");
    }


    ~DSSToROSTFNode() override
    {
        for (int i = 0; i < nats_.count; ++i)
            natsSubscription_Destroy(nats_.subs[i]);

        natsConnection_Destroy(nats_.conn);
        nats_Close();
    }


private:

// ==================== TF 변환 ====================

    void PublishTF(const dss::DSSTF& dss_tf)
    {
        double stamp_sec = dss_tf.header().stamp();

        geometry_msgs::msg::TransformStamped tf_msg;

        rclcpp::Time ros_stamp(
            static_cast<int64_t>(stamp_sec * 1e9),
            RCL_ROS_TIME
        );

        tf_msg.header.stamp = ros_stamp;

        tf_msg.header.frame_id = dss_tf.parent_frame();
        tf_msg.child_frame_id  = dss_tf.child_frame();

        tf_msg.transform.translation.x = dss_tf.x();
        tf_msg.transform.translation.y = dss_tf.y();
        tf_msg.transform.translation.z = dss_tf.z();

        tf_msg.transform.rotation.x = dss_tf.qx();
        tf_msg.transform.rotation.y = dss_tf.qy();
        tf_msg.transform.rotation.z = dss_tf.qz();
        tf_msg.transform.rotation.w = dss_tf.qw();

        tf_broadcaster_->sendTransform(tf_msg);
    }


// ==================== NATS Callback ====================

    static void sOnTopicRaw(
        natsConnection*,
        natsSubscription*,
        natsMsg* msg,
        void* closure)
    {
        auto* ctx = static_cast<TopicCtx*>(closure);

        std::string subject = natsMsg_GetSubject(msg);
        const char* d = natsMsg_GetData(msg);
        int len = natsMsg_GetDataLength(msg);

        try
        {
            if (d && len > 0)
                (*ctx->fn)(subject, d, len);
        }
        catch (...)
        {
            std::cerr << "TF callback error\n";
        }

        natsMsg_Destroy(msg);
    }


// ==================== Subscribe Helper ====================

    bool subscribeTopicRaw(
        const std::string& subject,
        TopicHandler handler,
        const char* queue = nullptr)
    {
        if (nats_.count >= MAX_SUBS)
            return false;

        topicHandlers_.emplace_back(
            std::make_unique<TopicHandler>(std::move(handler)));

        auto* fnPtr = topicHandlers_.back().get();

        rawCtx_.emplace_back(
            std::make_unique<TopicCtx>(TopicCtx{fnPtr}));

        auto* ctx = rawCtx_.back().get();

        natsSubscription* sub = nullptr;

        natsStatus s = (queue && *queue)
            ? natsConnection_QueueSubscribe(&sub, nats_.conn, subject.c_str(), queue, sOnTopicRaw, ctx)
            : natsConnection_Subscribe(&sub,      nats_.conn, subject.c_str(),       sOnTopicRaw, ctx);

        if (s != NATS_OK)
        {
            RCLCPP_ERROR(get_logger(),
                "subscribeTopicRaw failed: %s",
                natsStatus_GetText(s));

            rawCtx_.pop_back();
            topicHandlers_.pop_back();
            return false;
        }

        nats_.subs[nats_.count++] = sub;

        return true;
    }


// ==================== Heartbeat ====================

    void publishHeartBeat()
    {
        if (!nats_.conn)
            return;

        json message;

        message["timeStamp"] = getCurrentTimeISO8601();
        message["status"] = "alive";

        natsConnection_PublishString(
            nats_.conn,
            "dss.dssToROSTF.heartBeat",
            message.dump().c_str()
        );
    }


    std::string getCurrentTimeISO8601()
    {
        using namespace std::chrono;

        auto now = system_clock::now();
        auto t = system_clock::to_time_t(now);
        auto ms = duration_cast<milliseconds>(now.time_since_epoch()) % 1000;

        std::ostringstream oss;

        oss << std::put_time(std::gmtime(&t), "%Y-%m-%dT%H:%M:%S");
        oss << "." << std::setw(3) << std::setfill('0') << ms.count() << "Z";

        return oss.str();
    }


    void onTick()
    {
        publishHeartBeat();
    }
};


// ==================== main ====================

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<DSSToROSTFNode>());

    rclcpp::shutdown();

    return 0;
}