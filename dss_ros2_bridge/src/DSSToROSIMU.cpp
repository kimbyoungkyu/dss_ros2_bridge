#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <opencv2/opencv.hpp>   // JPEG 디코딩용
#include <nats/nats.h>
#include <functional>
#include <vector>
#include <memory>
#include <string>
#include <mutex>
#include <thread>
#include <chrono>
#include <sstream>
#include <iomanip>
#include <iostream>
#include <random>
#include <stdio.h>
#include <stdlib.h>
#include <arpa/inet.h>
#include <fstream>
#include "dss.pb.h"
#include "nlohmann/json.hpp"
using json = nlohmann::json;

#define MAX_SUBS (64)   // 동시에 최대 64개 구독 보유

// ==================== NATS 클라이언트 보관 ====================
struct NatsClient {
    natsConnection*      conn = nullptr;
    natsSubscription*    subs[MAX_SUBS]{};
    int                  count = 0;
};



std::string getDefaultGateway()
{
    std::ifstream file("/proc/net/route");
    if (!file.is_open()) {
        return "";
    }

    std::string line;
    while (std::getline(file, line)) {
        std::istringstream iss(line);

        std::string iface, destination, gatewayHex;
        unsigned flags, refcnt, use, metric, mask, mtu, win, irtt;

        if (!(iss >> iface >> destination >> gatewayHex >> flags >> refcnt >> use
              >> metric >> mask >> mtu >> win >> irtt)) {
            continue;
        }

        // Destination 00000000 = default route
        if (destination == "00000000") {
            unsigned long gatewayUL = std::stoul(gatewayHex, nullptr, 16);

            struct in_addr addr;
            addr.s_addr = static_cast<uint32_t>(gatewayUL);

            return inet_ntoa(addr);
        }
    }

    return "";
}


class DSSToROSIMUNode : public rclcpp::Node
{
public:
    using TopicHandler  = std::function<void(const std::string& subject, const char* data, int len)>;
    struct TopicCtx     { TopicHandler*     fn; };

    // NATS
    NatsClient nats_;

    // 수명 보장 컨테이너
    
    std::vector<std::unique_ptr<TopicHandler>>          topicHandlers_;
    std::vector<std::unique_ptr<TopicCtx>>              rawCtx_;
    rclcpp::TimerBase::SharedPtr                        timer_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;

    sensor_msgs::msg::Imu createDynamicDummyImu(double t) {
        sensor_msgs::msg::Imu imu;
        imu.header.stamp = rclcpp::Clock().now();
        imu.header.frame_id = "imu_link";

        // Orientation (Euler → Quaternion 변환 없음, 단순 더미)
        imu.orientation.x = 0.0;
        imu.orientation.y = 0.0;
        imu.orientation.z = sin(t * 0.5) * 0.1;
        imu.orientation.w = 1.0;

        // 각속도
        imu.angular_velocity.x = 0.1 * sin(t);
        imu.angular_velocity.y = 0.1 * cos(t);
        imu.angular_velocity.z = 0.05;

        // 선가속도
        imu.linear_acceleration.x = 0.5 * sin(t * 0.3);
        imu.linear_acceleration.y = 0.5 * cos(t * 0.3);
        imu.linear_acceleration.z = 9.81;
        return imu;
    }

    sensor_msgs::msg::Imu createImu(const dss::DSSIMU& imu_msg) {
        sensor_msgs::msg::Imu imu;
        imu.header.stamp = this->now();
        imu.header.frame_id = "map";

        // -----------------------
        // Orientation
        // -----------------------
        if (imu_msg.has_orientation())
        {
            imu.orientation.x = imu_msg.orientation().x();
            imu.orientation.y = imu_msg.orientation().y();
            imu.orientation.z = imu_msg.orientation().z();
            imu.orientation.w = imu_msg.orientation().w();
        }
        else
        {
            // DSSIMU에 orientation이 없는 경우 → 기본값 사용
            imu.orientation.x = 0.0;
            imu.orientation.y = 0.0;
            imu.orientation.z = 0.0;
            imu.orientation.w = 1.0;
        }

        // Covariance unknown = -1
        imu.orientation_covariance[0] = -1.0;

        // -----------------------
        // Angular velocity
        // -----------------------
        imu.angular_velocity.x = imu_msg.angular_velocity().x();
        imu.angular_velocity.y = imu_msg.angular_velocity().y();
        imu.angular_velocity.z = imu_msg.angular_velocity().z();

        imu.angular_velocity_covariance[0] = -1.0;

        // -----------------------
        // Linear acceleration
        // -----------------------
        imu.linear_acceleration.x = imu_msg.linear_acceleration().x();
        imu.linear_acceleration.y = imu_msg.linear_acceleration().y();
        imu.linear_acceleration.z = imu_msg.linear_acceleration().z();

        imu.linear_acceleration_covariance[0] = -1.0;

        return imu;
    }
public:
    DSSToROSIMUNode() : Node("DSSToROSIMUNode") {
        //RCLCPP_INFO(get_logger(),get_default_gateway().c_str());
        //RCLCPP_INFO(get_logger(),get_default_gateway().c_str());
        RCLCPP_INFO(get_logger(),getDefaultGateway().c_str());        

        //this->declare_parameter<std::string>("nats_server", "nats://127.0.0.1:4222");
        //std::string kNatsUrl = this->get_parameter("nats_server").as_string();
         std::string kNatsUrl = "nats://" + getDefaultGateway()+ ":4222";
        RCLCPP_INFO(get_logger(), kNatsUrl.c_str());
        natsStatus s = natsConnection_ConnectTo(&nats_.conn, kNatsUrl.c_str());
        if (s != NATS_OK) {
            std::cerr << "NATS connect failed: " << natsStatus_GetText(s) << std::endl;
            return;
        }     
        
        timer_   = this->create_wall_timer(std::chrono::seconds(3), std::bind(&DSSToROSIMUNode::onTick, this));
        subscribeTopicRaw("dss.sensor.imu",
            [this](const std::string& subject, const char* bytes, int len)
            {
                dss::DSSIMU imu_msg;
                if (!imu_msg.ParseFromArray(bytes, len)) {
                    std::cerr << "Failed to parse DSSIMU protobuf message\n";
                    return; 
                }
                pub_->publish(createImu(imu_msg));
            }
        );
        pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/dss/sensor/imu", 10);

        RCLCPP_INFO(get_logger(), "[NATS]dss.sensor.imu → [ROS2]/dss/sensor/imu");
    }

    ~DSSToROSIMUNode() override {
        for (int i = 0; i < nats_.count; ++i) {
            natsSubscription_Destroy(nats_.subs[i]);
        }
        natsConnection_Destroy(nats_.conn);
        nats_Close();
    }

    static void sOnTopicRaw(natsConnection*, natsSubscription*, natsMsg* msg, void* closure) {
        auto* ctx = static_cast<TopicCtx*>(closure);
        std::string subject = natsMsg_GetSubject(msg);
        const char* d = natsMsg_GetData(msg);
        int len = natsMsg_GetDataLength(msg);
        try { if (d && len > 0) (*ctx->fn)(subject, d, len); }
        catch (const std::exception& e) { std::cerr << "sOnTopicRaw error: " << e.what() << std::endl; }
        catch (...) { std::cerr << "sOnTopicRaw unknown error\n"; }
        natsMsg_Destroy(msg);
    }

    void publishHeartBeat() {
        if (!nats_.conn) return;
        json message;
        message["timeStamp"] = getCurrentTimeISO8601();
        message["status"]    = "alive";

        natsStatus s = natsConnection_PublishString(nats_.conn, "dss.dssToROSImu.heartBeat", message.dump().c_str());
        if (s != NATS_OK) {
            std::cerr << "Heartbeat publish error: " << natsStatus_GetText(s) << std::endl;
        }
    }

    std::string getCurrentTimeISO8601() {
        using namespace std::chrono;
        auto now = system_clock::now();
        auto t = system_clock::to_time_t(now);
        auto ms = duration_cast<milliseconds>(now.time_since_epoch()) % 1000;

        std::ostringstream oss;
        oss << std::put_time(std::gmtime(&t), "%Y-%m-%dT%H:%M:%S");
        oss << "." << std::setw(3) << std::setfill('0') << ms.count() << "Z";
        return oss.str();
    }



    void onTick() {
        publishHeartBeat();
    }

private:
    bool subscribeTopicRaw(const std::string& subject, TopicHandler handler, const char* queue = nullptr) {
        if (nats_.count >= MAX_SUBS) return false;

        topicHandlers_.emplace_back(std::make_unique<TopicHandler>(std::move(handler)));
        auto* fnPtr = topicHandlers_.back().get();

        rawCtx_.emplace_back(std::make_unique<TopicCtx>(TopicCtx{fnPtr}));
        auto* ctx = rawCtx_.back().get();

        natsSubscription* sub = nullptr;
        natsStatus s = (queue && *queue)
            ? natsConnection_QueueSubscribe(&sub, nats_.conn, subject.c_str(), queue, sOnTopicRaw, ctx)
            : natsConnection_Subscribe(&sub,      nats_.conn, subject.c_str(),       sOnTopicRaw, ctx);

        if (s != NATS_OK) {
            std::cerr << "subscribeTopicRaw failed: " << subject << " : " << natsStatus_GetText(s) << std::endl;
            rawCtx_.pop_back(); topicHandlers_.pop_back();
            return false;
        }
        nats_.subs[nats_.count++] = sub;
        return true;
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DSSToROSIMUNode>());
    rclcpp::shutdown();
}
