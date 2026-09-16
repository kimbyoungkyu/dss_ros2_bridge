#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <nats/nats.h>

#include "dss.pb.h"
#include "defaultGateway.h"

#include "nlohmann/json.hpp"

using json = nlohmann::json;

#define MAX_SUBS (64)

// ==================== NATS 클라이언트 ====================

struct NatsClient {
    natsConnection*      conn = nullptr;
    natsSubscription*    subs[MAX_SUBS]{};
    int                  count = 0;
};

class DSSToROSLaserScanNode : public rclcpp::Node
{
public:

    using TopicHandler  = std::function<void(const std::string&, const char*, int)>;
    struct TopicCtx     { TopicHandler* fn; };

private:
    NatsClient nats_;
    std::vector<std::unique_ptr<TopicHandler>> topicHandlers_;
    std::vector<std::unique_ptr<TopicCtx>> rawCtx_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;

public:

    DSSToROSLaserScanNode() : Node("DSSToROSLaserScanNode") {
        std::string url = "nats://" + getDefaultGateway() + ":4222";
        RCLCPP_INFO(get_logger(),"Connecting NATS %s",url.c_str());

        natsStatus s = natsConnection_ConnectTo(&nats_.conn,url.c_str());
        if (s != NATS_OK) {
            RCLCPP_ERROR(get_logger(),"NATS connect failed: %s",natsStatus_GetText(s));
            return;
        }

        pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan",10); 
        subscribeTopicRaw("dss.sensor.lidar2d",[this](const std::string&, const char* bytes, int len) {
                dss::DssLaserScan2D scan_msg;
                if (!scan_msg.ParseFromArray(bytes,len)){
                    RCLCPP_ERROR(get_logger(),"LaserScan protobuf parse failed");
                    return;
                }
                auto use_sim_time_ = this->get_parameter("use_sim_time").as_bool();    
                if (use_sim_time_){
                    pub_->publish(createLaserScan(scan_msg));
                }
            }
        );
        timer_ = this->create_wall_timer(std::chrono::seconds(3),std::bind(&DSSToROSLaserScanNode::onTick,this));
        RCLCPP_INFO(get_logger(),"[NATS] dss.sensor.lidar2d → [ROS2] /scan");
    }

    ~DSSToROSLaserScanNode()
    {
        for (int i = 0; i < nats_.count; ++i) {
            natsSubscription_Destroy(nats_.subs[i]);
        }
        natsConnection_Destroy(nats_.conn);
        nats_Close();
    }

private:

// ============================================================
// LaserScan 생성
// ============================================================

    sensor_msgs::msg::LaserScan createLaserScan( const dss::DssLaserScan2D& scan_msg) {
        sensor_msgs::msg::LaserScan msg;

        double stamp_sec = scan_msg.header().stamp();

        rclcpp::Time ros_stamp(
            static_cast<int64_t>(stamp_sec * 1e9),
            RCL_ROS_TIME
        );

        

        msg.header.stamp = ros_stamp;
        //msg.header.frame_id = "lidar_link";
        //msg.header.frame_id = "turtlebot4_laserscan";
        msg.header.frame_id = "base_link";

        msg.angle_min = scan_msg.angle_min();
        msg.angle_max = scan_msg.angle_max();
        msg.angle_increment = scan_msg.angle_increment();

        msg.time_increment = scan_msg.time_increment();
        msg.scan_time = scan_msg.scan_time();

        msg.range_min = scan_msg.range_min();
        msg.range_max = scan_msg.range_max();

        msg.ranges.resize(scan_msg.ranges_size());

        for (int i = 0; i < scan_msg.ranges_size(); i++)
            msg.ranges[i] = scan_msg.ranges(i);

        msg.intensities.resize(scan_msg.intensities_size());
        for (int i = 0; i < scan_msg.intensities_size(); i++) {
            msg.intensities[i] = scan_msg.intensities(i);
        }

        RCLCPP_INFO(rclcpp::get_logger("laserScan"),"[NATS]dss.sensor.lidar2d → [ROS2]/scan = %ld.%09u",msg.header.stamp.sec,msg.header.stamp.nanosec);

        //RCLCPP_INFO(get_logger(),"/scan stamp_sec = %f",stamp_sec);
        return msg;
    }


// ============================================================
// NATS callback
// ============================================================

    static void sOnTopicRaw(natsConnection*,natsSubscription*,natsMsg* msg,void* closure)
    {
        auto* ctx = static_cast<TopicCtx*>(closure);

        std::string subject = natsMsg_GetSubject(msg);
        const char* d = natsMsg_GetData(msg);
        int len = natsMsg_GetDataLength(msg);

        try
        {
            if (d && len > 0)
                (*ctx->fn)(subject,d,len);
        }
        catch(...)
        {
        }

        natsMsg_Destroy(msg);
    }


// ============================================================
// subscribe helper
// ============================================================

    bool subscribeTopicRaw(
        const std::string& subject,
        TopicHandler handler)
    {
        if (nats_.count >= MAX_SUBS)
            return false;

        topicHandlers_.push_back(
            std::make_unique<TopicHandler>(handler));

        auto* fnPtr = topicHandlers_.back().get();

        rawCtx_.push_back(
            std::make_unique<TopicCtx>(TopicCtx{fnPtr}));

        auto* ctx = rawCtx_.back().get();

        natsSubscription* sub = nullptr;

        natsStatus s =
            natsConnection_Subscribe(
                &sub,
                nats_.conn,
                subject.c_str(),
                sOnTopicRaw,
                ctx
            );

        if (s != NATS_OK)
        {
            RCLCPP_ERROR(get_logger(),
                "subscribe failed: %s",
                natsStatus_GetText(s));

            rawCtx_.pop_back();
            topicHandlers_.pop_back();

            return false;
        }

        nats_.subs[nats_.count++] = sub;

        return true;
    }


// ============================================================
// Heartbeat
// ============================================================

    void publishHeartBeat()
    {
        json message;

        message["timeStamp"] = getCurrentTimeISO8601();
        message["status"] = "alive";

        natsConnection_PublishString(
            nats_.conn,
            "dss.dssToROSLaserScan.heartBeat",
            message.dump().c_str()
        );
    }

    std::string getCurrentTimeISO8601()
    {
        using namespace std::chrono;

        auto now = system_clock::now();
        auto t = system_clock::to_time_t(now);
        auto ms = duration_cast<milliseconds>(
            now.time_since_epoch()) % 1000;

        std::ostringstream oss;

        oss << std::put_time(std::gmtime(&t),
            "%Y-%m-%dT%H:%M:%S");

        oss << "."
            << std::setw(3)
            << std::setfill('0')
            << ms.count()
            << "Z";

        return oss.str();
    }

    void onTick()
    {
        publishHeartBeat();
    }

};

// ============================================================
// main
// ============================================================

int main(int argc,char** argv)
{
    rclcpp::init(argc,argv);

    rclcpp::spin(
        std::make_shared<DSSToROSLaserScanNode>());

    rclcpp::shutdown();

    return 0;
}