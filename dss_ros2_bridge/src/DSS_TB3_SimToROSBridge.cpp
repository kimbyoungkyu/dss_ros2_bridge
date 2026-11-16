#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <rosgraph_msgs/msg/clock.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nats/nats.h>
#include <nlohmann/json.hpp>
//#include <opencv2/opencv.hpp>
#include <chrono>
#include <algorithm>
#include <cstdlib>
#include "DSS_TB3_BurgerFrames.h"
#include <array>
#include <cmath>
#include <cstdint>
#include <ctime>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>
#include "defaultGateway.h"
#include "dss.pb.h"

namespace {
constexpr std::int64_t kNanosecondsPerSecond = 1'000'000'000LL;
constexpr auto kHeartbeatInterval = std::chrono::seconds{3};
constexpr char kTfSubject[] = "dss.tf";
constexpr char kClockSubject[] = "dss.simTime.clock";
constexpr char kImageSubject[] = "dss.sensor.camera.rgb";
constexpr char kImuSubject[] = "dss.sensor.imu";
constexpr char kLaserScanSubject[] = "dss.sensor.lidar2d";
constexpr char kOdomSubject[] = "dss.sensor.odom";
constexpr char kWheelEncoderSubject[] = "dss.sensor.wheelEncoder";
constexpr char kVelocityCommandSubject[] = "dss.turtlebot3_burger.velcmd";
constexpr char kHeartbeatSubject[] = "dss.DSS_TB3_SimToROSBridgeNode.heartBeat";
}  // namespace

class DSS_TB3_SimToROSBridgeNode final : public rclcpp::Node {
public:
    DSS_TB3_SimToROSBridgeNode() : Node("DSS_TB3_SimToROSBridgeNode") {
        RCLCPP_INFO(get_logger(), __DATE__);
        RCLCPP_INFO(get_logger(), __TIME__);
        RCLCPP_INFO(get_logger(), "Domain Id=%zu", get_node_base_interface()->get_context()->get_domain_id());

        publish_static_tf_ = declare_parameter<bool>("publish_static_tf", true);
        publish_odom_tf_ = declare_parameter<bool>("publish_odom_tf", true);
        const std::string nats_url = declare_parameter<std::string>(
            "nats_url", "nats://" + getDefaultGateway() + ":4222");
        RCLCPP_INFO(get_logger(), "Connecting to NATS: %s", nats_url.c_str());
        const natsStatus status = natsConnection_ConnectTo(&nats_connection_, nats_url.c_str());
        if (status != NATS_OK) {
            RCLCPP_ERROR(get_logger(), "NATS connection failed: %s", natsStatus_GetText(status));
            return;
        }
        registTf();
        if (publish_static_tf_) publishStaticTransforms();
        registClock();
        //registImage();
        registImu();
        registLaserScan();
        registHeartbeat();
        registOdom();
        registWheelEncoder();
        registCmdVel();
        if (useSimTime()){
            RCLCPP_INFO(get_logger(), "DSS TurtleBot3 Burger Sim2ROS bridge is running in sim_time mode.");
        }else{
            RCLCPP_WARN(get_logger(), "use_sim_time is false: sensor, clock and dynamic TF publication is disabled.");
        }
    }

    ~DSS_TB3_SimToROSBridgeNode() override
    {
        for (auto* subscription : subscriptions_) {
            if (subscription != nullptr) {
                natsSubscription_Destroy(subscription);
            }
        }
        if (nats_connection_ != nullptr) {
            natsConnection_Destroy(nats_connection_);
        }
        nats_Close();
    }

private:
    using TopicHandler = std::function<void(const std::string&, const char*, int)>;
    struct TopicContext {
        TopicHandler* handler = nullptr;
    };

    static void onNatsMessage(natsConnection*, natsSubscription*, natsMsg* message, void* closure)
    {
        std::unique_ptr<natsMsg, decltype(&natsMsg_Destroy)> message_guard(message, &natsMsg_Destroy);
        auto* context = static_cast<TopicContext*>(closure);
        if (message == nullptr || context == nullptr || context->handler == nullptr) {
            return;
        }

        const char* data = natsMsg_GetData(message);
        const int length = natsMsg_GetDataLength(message);
        if (data == nullptr || length <= 0) {
            return;
        }

        try {
            const char* raw_subject = natsMsg_GetSubject(message);
            (*context->handler)(raw_subject != nullptr ? raw_subject : "", data, length);
        } catch (const std::exception& error) {
            std::cerr << "NATS callback error: " << error.what() << '\n';
        } catch (...) {
            std::cerr << "Unknown NATS callback error\n";
        }
    }

    bool subscribe(const std::string& subject, TopicHandler handler,const char* queue = nullptr)
    {
        if (nats_connection_ == nullptr) {
            RCLCPP_ERROR(get_logger(), "Cannot subscribe to '%s': NATS is disconnected",subject.c_str());
            return false;
        }

        auto owned_handler = std::make_unique<TopicHandler>(std::move(handler));
        auto context = std::make_unique<TopicContext>();
        context->handler = owned_handler.get();

        natsSubscription* subscription = nullptr;
        const natsStatus status = queue != nullptr && *queue != '\0'
            ? natsConnection_QueueSubscribe(
                  &subscription, nats_connection_, subject.c_str(), queue,
                  &DSS_TB3_SimToROSBridgeNode::onNatsMessage, context.get())
            : natsConnection_Subscribe(
                  &subscription, nats_connection_, subject.c_str(),
                  &DSS_TB3_SimToROSBridgeNode::onNatsMessage, context.get());

        if (status != NATS_OK) {
            RCLCPP_ERROR(get_logger(), "NATS subscription failed (%s): %s",subject.c_str(), natsStatus_GetText(status));
            return false;
        }

        topic_handlers_.push_back(std::move(owned_handler));
        topic_contexts_.push_back(std::move(context));
        subscriptions_.push_back(subscription);
        return true;
    }

    bool useSimTime() const
    {
        return get_parameter("use_sim_time").as_bool();
    }

    void registTf()
    {
        // /odom and its TF share the same source, stamp and planar pose.
        // Do not also subscribe to dss.tf: it would create a second TF writer.
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    static std::vector<geometry_msgs::msg::TransformStamped> burgerStaticTransforms()
    {
        const auto make = [](const char* parent, const char* child,
                             double x, double y, double z) {
            geometry_msgs::msg::TransformStamped t;
            t.header.stamp = toRosTime(0.0);
            t.header.frame_id = parent; t.child_frame_id = child;
            t.transform.translation.x=x; t.transform.translation.y=y; t.transform.translation.z=z;
            t.transform.rotation.w=1.0;
            return t;
        };
        auto base = make("base_footprint", "base_link", 0, 0, 0.01);
        auto caster = make("base_link", "caster_back_link", -0.081, 0, -0.004);
        caster.transform.rotation.x = -0.706825181105366;
        caster.transform.rotation.w = 0.7073882691671998;
        auto imu = make("base_link", "imu_link", -0.032, 0, 0.068);
        auto laser = make("base_link", "base_scan", -0.032, 0, 0.172);
        return {base, caster, imu, laser};
    }

    void publishStaticTransforms()
    {
        static_tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
        static_tf_broadcaster_->sendTransform(burgerStaticTransforms());
    }

    void registClock()
    {
        clock_publisher_ = create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);
        subscribe(kClockSubject, [this](const std::string&, const char* data, int length) {
            dss::DssClock source;
            if (!source.ParseFromArray(data, length)) {
                RCLCPP_ERROR(get_logger(), "DssClock protobuf parse failed");
                return;
            }
            if (useSimTime()) {
                clock_publisher_->publish(createClock(source));
            }
        });
    }
/*
    void registImage()
    {
        image_publisher_ = create_publisher<sensor_msgs::msg::Image>("/camera/color/image_raw", 10);
        subscribe(kImageSubject, [this](const std::string&, const char* data, int length) {
            dss::DSSImage source;
            if (!source.ParseFromArray(data, length)) {
                RCLCPP_ERROR(get_logger(), "DSSImage protobuf parse failed");
                return;
            }
            if (useSimTime()) {
                image_publisher_->publish(createImage(source));
            }
        });
    }
*/
    void registImu()
    {
        imu_publisher_ = create_publisher<sensor_msgs::msg::Imu>("/imu", 10);
        subscribe(kImuSubject, [this](const std::string&, const char* data, int length) {
            dss::DSSIMU source;
            if (!source.ParseFromArray(data, length)) {
                RCLCPP_ERROR(get_logger(), "DSSIMU protobuf parse failed");
                return;
            }
            if (useSimTime()) {
                // Keep native simulator geometry/data untouched. Re-express its
                // IMU sample at the requested ROS imu_link point in this bridge.
                const tb3_burger::Vector omega{source.angular_velocity().x(),
                    source.angular_velocity().y(), source.angular_velocity().z()};
                tb3_burger::Vector alpha{};
                const bool valid = imu_acceleration_.update(source.header().stamp(), omega, alpha)
                    && source.has_angular_velocity() && source.has_linear_acceleration();
                imu_publisher_->publish(createROSImu(source, alpha, valid));
            }
        });
    }

    void registLaserScan()
    {
        laser_scan_publisher_ = create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);
        subscribe(kLaserScanSubject,[this](const std::string&, const char* data, int length) {
            dss::DssLaserScan2D source;
            if (!source.ParseFromArray(data, length)) {
                RCLCPP_ERROR(get_logger(), "DssLaserScan2D protobuf parse failed");
                return;
            }
            if (useSimTime()) {
                laser_scan_publisher_->publish(createLaserScan(source));
            }
        });
    }

    void registOdom() {
        odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

        subscribe(kOdomSubject, [this](const std::string&, const char* data, int length) {
            dss::DSSOdom source;
            if (!source.ParseFromArray(data, length)) {
                RCLCPP_ERROR(get_logger(), "DSSOdom protobuf parse failed");
                return;
            }
            if (useSimTime()) {
                if (source.header().frame_id() != "mujoco_world" || source.child_frame_id() != "base_frame") {
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                        "Ignoring odometry: expected mujoco_world -> base_frame");
                    return;
                }
                tb3_burger::Quaternion q{source.pose().orientation().x(), source.pose().orientation().y(),
                    source.pose().orientation().z(), source.pose().orientation().w()};
                if (!std::isfinite(source.header().stamp()) || source.header().stamp() < 0 ||
                    !std::isfinite(source.pose().position().x()) || !std::isfinite(source.pose().position().y()) ||
                    !std::isfinite(source.twist().linear().x()) || !std::isfinite(source.twist().linear().y()) ||
                    !std::isfinite(source.twist().angular().z()) || !tb3_burger::normalize(q)) {
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Ignoring invalid Burger odometry");
                    return;
                }
                auto message = createOdom(source);
                odom_publisher_->publish(message);
                if (publish_odom_tf_) {
                    geometry_msgs::msg::TransformStamped transform;
                    transform.header = message.header;
                    transform.child_frame_id = message.child_frame_id;
                    transform.transform.translation.x = message.pose.pose.position.x;
                    transform.transform.translation.y = message.pose.pose.position.y;
                    transform.transform.translation.z = message.pose.pose.position.z;
                    transform.transform.rotation = message.pose.pose.orientation;
                    tf_broadcaster_->sendTransform(transform);
                }
            }
        });
    }

    void registWheelEncoder()
    {
        joint_state_publisher_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

        subscribe(kWheelEncoderSubject,[this](const std::string&, const char* data, int length) {
            dss::DSSWheelEncoder source;
            if (!source.ParseFromArray(data, length)) {
                RCLCPP_ERROR(get_logger(), "DSSWheelEncoder protobuf parse failed");
                return;
            }
            if (useSimTime()) {
                joint_state_publisher_->publish(createJointState(source));
            }
        });
    }

    void registCmdVel()
    {
        cmd_vel_subscription_ = create_subscription<geometry_msgs::msg::Twist>("/cmd_vel",rclcpp::QoS(10),std::bind(&DSS_TB3_SimToROSBridgeNode::onCmdVel,this,std::placeholders::_1));
        RCLCPP_INFO(get_logger(),"ROS2 /cmd_vel -> NATS %s bridge is ready",kVelocityCommandSubject);
    }

    void onCmdVel(const geometry_msgs::msg::Twist::SharedPtr message)
    {
        if (message == nullptr || nats_connection_ == nullptr) {
            return;
        }

        dss::TurtleBot4Control command;
        command.set_identifier("turtlebot3_burger");

        const auto timestamp_ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        command.set_timestamp(timestamp_ms);

        // This workspace's unchanged schema names the command TurtleBot4Control.
        // Its field numbers/types are wire-compatible with TurtleBot3BurgerControl.
        const bool valid = std::isfinite(message->linear.x) && std::isfinite(message->angular.z);
        const double v = valid ? std::clamp(message->linear.x, -0.22, 0.22) : 0.0;
        const double w = valid ? std::clamp(message->angular.z, -2.75, 2.75) : 0.0;
        command.set_linear_x(v);
        command.set_linear_y(0.0);
        command.set_angular_z(w);
        constexpr double kStoppedEpsilon = 1.0e-6;
        const bool stopped = std::abs(v) < kStoppedEpsilon && std::abs(w) < kStoppedEpsilon;
        command.set_mode(stopped ? dss::TurtleBot4Control::MODE_STOP : dss::TurtleBot4Control::MODE_VELOCITY);
        std::string payload;
        if (!command.SerializeToString(&payload)) {
            RCLCPP_ERROR(get_logger(), "TurtleBot4Control serialization failed");
            return;
        }

        const natsStatus status = natsConnection_Publish(nats_connection_,kVelocityCommandSubject,payload.data(),static_cast<int>(payload.size()));
        if (status != NATS_OK) {
            RCLCPP_ERROR(get_logger(),"NATS velocity command publish failed: %s",natsStatus_GetText(status));
            return;
        }

        RCLCPP_INFO_THROTTLE(
            get_logger(),
            *get_clock(),
            500,
            "[ROS2] /cmd_vel -> [NATS] %s "
            "linear=(%.3f, %.3f), angular_z=%.3f, mode=%s",
            kVelocityCommandSubject,
            message->linear.x,
            message->linear.y,
            message->angular.z,
            stopped ? "STOP" : "VELOCITY");
    }

    template <typename RepeatedField>
    static std::array<double, 36> copyCovariance(
        const RepeatedField& source, const std::array<double, 6>& fallback_diagonal)
    {
        std::array<double, 36> result{};
        if (source.size() == 36) {
            for (int i=0;i<36;++i) result[i]=source.Get(i);
        } else {
            for (std::size_t i=0;i<6;++i) result[i*6+i]=fallback_diagonal[i];
        }
        return result;
    }

    static nav_msgs::msg::Odometry createOdom(const dss::DSSOdom& source)
    {
        nav_msgs::msg::Odometry message;
        message.header.stamp = toRosTime(source.header().stamp());
        message.header.frame_id = "odom";
        message.child_frame_id = "base_footprint";
        // Burger MJCF already uses +X forward, +Y left, +Z up.
        // The native base is the ground reference, not base_link (+0.01 m).
        message.pose.pose.position.x = source.pose().position().x();
        message.pose.pose.position.y = source.pose().position().y();
        message.pose.pose.position.z = 0.0;
        tb3_burger::Quaternion q{source.pose().orientation().x(), source.pose().orientation().y(),
            source.pose().orientation().z(), source.pose().orientation().w()};
        tb3_burger::normalize(q); // validated by the subscriber
        const double heading = tb3_burger::yaw(q);
        message.pose.pose.orientation.z = std::sin(heading/2);
        message.pose.pose.orientation.w = std::cos(heading/2);
        message.pose.covariance = copyCovariance(source.pose_covariance(),
            {1e-4,1e-4,1e-3,1e-3,1e-3,1e-4});
        // framelinvel/frameangvel in the current MJCF are WORLD-frame vectors.
        // nav_msgs/Odometry requires twist in child_frame_id, here base_footprint.
        const auto linear = tb3_burger::worldToFootprint({source.twist().linear().x(),
            source.twist().linear().y(), source.twist().linear().z()}, heading);
        const auto angular = tb3_burger::worldToFootprint({source.twist().angular().x(),
            source.twist().angular().y(), source.twist().angular().z()}, heading);
        message.twist.twist.linear.x = linear[0];
        message.twist.twist.linear.y = linear[1];
        message.twist.twist.angular.z = angular[2];
        message.twist.covariance = tb3_burger::worldToFootprintCovariance(
            copyCovariance(source.twist_covariance(), {1e-4,1e-4,1e-3,1e-3,1e-3,1e-4}), heading);
        return message;
    }

    sensor_msgs::msg::JointState createJointState(const dss::DSSWheelEncoder& source)
    {
        sensor_msgs::msg::JointState message;
        message.header.stamp = toRosTime(source.header().stamp());

        // JointState names belong to the ROS TurtleBot3 Burger URDF.
        message.name = {"wheel_left_joint", "wheel_right_joint"};
        message.position = {source.left_position(), source.right_position()};
        message.velocity = {source.left_velocity(), source.right_velocity()};

        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 500,
            "[NATS] %s -> [ROS2] /joint_states "
            "left=(%.4f rad, %.4f rad/s), right=(%.4f rad, %.4f rad/s)",
            kWheelEncoderSubject,
            message.position[0], message.velocity[0],
            message.position[1], message.velocity[1]);

        return message;

    }

    void registHeartbeat()
    {
        heartbeat_timer_ = create_wall_timer(kHeartbeatInterval,std::bind(&DSS_TB3_SimToROSBridgeNode::publishHeartbeat, this));
    }

    rosgraph_msgs::msg::Clock createClock(const dss::DssClock& source)
    {
        rosgraph_msgs::msg::Clock message;
        const double elapsed_seconds = source.total_elapsed_time();
        if (!std::isfinite(elapsed_seconds) || elapsed_seconds < 0.0) {
            RCLCPP_WARN(get_logger(), "Invalid DSS simulation time: %.9f",elapsed_seconds);
            return message;
        }

        const auto total_nanoseconds = static_cast<std::int64_t>(std::llround(elapsed_seconds * static_cast<double>(kNanosecondsPerSecond)));
        const auto seconds = total_nanoseconds / kNanosecondsPerSecond;
        if (seconds > std::numeric_limits<std::int32_t>::max()) {
            RCLCPP_ERROR(get_logger(),"DSS simulation time exceeds the ROS time range: %.9f",elapsed_seconds);
            message.clock.sec = std::numeric_limits<std::int32_t>::max();
            message.clock.nanosec = 999'999'999U;
            return message;
        }

        message.clock.sec = static_cast<std::int32_t>(seconds);
        message.clock.nanosec = static_cast<std::uint32_t>(total_nanoseconds % kNanosecondsPerSecond);

        RCLCPP_INFO_THROTTLE(
            get_logger(),
            *get_clock(),
            1000,
            "[NATS] %s %.9f sec -> [ROS2] /clock %d.%09u",        
            kClockSubject,
            elapsed_seconds,
            message.clock.sec,
            message.clock.nanosec);

        return message;
    }

    /*
    sensor_msgs::msg::Image createImage(const dss::DSSImage& source) const
    {
        const std::vector<std::uint8_t> jpeg_data(source.data().begin(), source.data().end());
        const cv::Mat bgr = cv::imdecode(jpeg_data, cv::IMREAD_COLOR);
        if (bgr.empty()) {
            throw std::runtime_error("JPEG decoding failed");
        }

        cv::Mat rgb;
        cv::cvtColor(bgr, rgb, cv::COLOR_BGR2RGB);
        if (!rgb.isContinuous()) {
            rgb = rgb.clone();
        }

        sensor_msgs::msg::Image message;
        message.header.stamp = toRosTime(source.header().stamp());
        message.header.frame_id = "camera";
        message.height = static_cast<std::uint32_t>(rgb.rows);
        message.width = static_cast<std::uint32_t>(rgb.cols);
        message.encoding = "rgb8";
        message.is_bigendian = false;
        message.step = static_cast<std::uint32_t>(rgb.cols * rgb.elemSize());
        message.data.assign(rgb.datastart, rgb.dataend);
        return message;
    }
        */

    
    static sensor_msgs::msg::Imu createROSImu(const dss::DSSIMU& source,
        const tb3_burger::Vector& alpha = {0,0,0}, bool angular_acceleration_valid = false)
    {
        sensor_msgs::msg::Imu message;
        message.header.stamp = toRosTime(source.header().stamp());
        message.header.frame_id = "imu_link";
        // Native Burger axes already match ROS; preserve the IMU site vectors.
        tb3_burger::Quaternion q{source.orientation().x(), source.orientation().y(),
            source.orientation().z(), source.orientation().w()};
        if (tb3_burger::normalize(q)) {
            message.orientation.x=q[0]; message.orientation.y=q[1];
            message.orientation.z=q[2]; message.orientation.w=q[3];
        } else {
            message.orientation.w=1.0;
            message.orientation_covariance[0]=-1.0; // unavailable orientation
        }
        const auto copy3 = [](const auto& input, auto& output) {
            if (input.size()==9) for(int i=0;i<9;++i) output[i]=input.Get(i);
        };
        if (message.orientation_covariance[0] != -1.0)
            copy3(source.orientation_covariance(),message.orientation_covariance);
        copy3(source.angular_velocity_covariance(),message.angular_velocity_covariance);
        copy3(source.linear_acceleration_covariance(),message.linear_acceleration_covariance);
        message.angular_velocity.x=source.angular_velocity().x();
        message.angular_velocity.y=source.angular_velocity().y();
        message.angular_velocity.z=source.angular_velocity().z();
        // Rigid-body point shift: a_target = a_source + alpha x r + omega x (omega x r).
        // Frames have identical axes; uniform gravity in specific force cancels.
        const auto acceleration = tb3_burger::accelerationAtRosImu(
            {source.linear_acceleration().x(),source.linear_acceleration().y(),source.linear_acceleration().z()},
            {source.angular_velocity().x(),source.angular_velocity().y(),source.angular_velocity().z()},alpha);
        message.linear_acceleration.x=acceleration[0];
        message.linear_acceleration.y=acceleration[1];
        message.linear_acceleration.z=acceleration[2];
        // Derivative uncertainty is not available in DSSIMU. Do not claim a
        // precise propagated covariance. First sample/reset/gap has no alpha.
        message.linear_acceleration_covariance.fill(0.0);
        if (!angular_acceleration_valid) message.linear_acceleration_covariance[0]=-1.0;
        return message;
    }

    static sensor_msgs::msg::LaserScan createLaserScan(const dss::DssLaserScan2D& source)
    {
        sensor_msgs::msg::LaserScan message;

        message.header.stamp = toRosTime(source.header().stamp());
        message.header.frame_id = "base_scan";

        // Native laser_site has the same axes as the Burger base_scan frame.
        message.angle_min = source.angle_min();
        message.angle_max = source.angle_max();
        message.angle_increment = source.angle_increment();

        message.time_increment = source.time_increment();

        message.scan_time = source.scan_time();

        message.range_min = source.range_min();

        message.range_max = source.range_max();

        /*
        * 회전 변환은 거리 크기에 영향을 주지 않는다.
        * 스캔 순서와 angle_increment도 그대로 유지한다.
        */
        message.ranges.reserve(source.ranges_size());

        for (const auto range : source.ranges()) 
        {
            // Preserve REP-117 distinctions. The simulator must map its
            // native ray miss (-1) to +infinity before sending this message.
            if (std::isnan(range)) {
                message.ranges.push_back(std::numeric_limits<float>::quiet_NaN());
            } else if (range < message.range_min) {
                message.ranges.push_back(-std::numeric_limits<float>::infinity());
            } else if (range > message.range_max) {
                message.ranges.push_back(std::numeric_limits<float>::infinity());
            } else {
                message.ranges.push_back(range);
            }
        }

        message.intensities.reserve(source.intensities_size());
        for (const auto intensity : source.intensities()) {
            message.intensities.push_back(intensity);
        }
        return message;
    }

    static rclcpp::Time toRosTime(double seconds)
    {
        if (!std::isfinite(seconds) || seconds < 0.0) {
            return rclcpp::Time{0, 0, RCL_ROS_TIME};
        }
        return rclcpp::Time{static_cast<std::int64_t>(std::llround(seconds * static_cast<double>(kNanosecondsPerSecond))),RCL_ROS_TIME};
    }

    void publishHeartbeat()
    {
        if (nats_connection_ == nullptr) {
            return;
        }
        const nlohmann::json message{{"timeStamp", currentTimeIso8601()}, {"status", "alive"}};
        const std::string payload = message.dump();
        const natsStatus status = natsConnection_PublishString(nats_connection_, kHeartbeatSubject, payload.c_str());
        if (status != NATS_OK) {
            RCLCPP_ERROR(get_logger(), "Heartbeat publish failed: %s",natsStatus_GetText(status));
        }
    }

    static std::string currentTimeIso8601()
    {
        using namespace std::chrono;
        const auto now = system_clock::now();
        const std::time_t time = system_clock::to_time_t(now);
        const auto milliseconds_part = duration_cast<milliseconds>(now.time_since_epoch()) % 1000;
        std::tm utc_time{};
#if defined(_WIN32)
        gmtime_s(&utc_time, &time);
#else
        gmtime_r(&time, &utc_time);
#endif
        std::ostringstream stream;
        stream << std::put_time(&utc_time, "%Y-%m-%dT%H:%M:%S") << '.' << std::setw(3) << std::setfill('0') << milliseconds_part.count()<< 'Z';
        return stream.str();
    }

    tb3_burger::AngularAccelerationEstimator imu_acceleration_;
    bool publish_static_tf_ = true;
    bool publish_odom_tf_ = true;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
    natsConnection* nats_connection_ = nullptr;
    std::vector<natsSubscription*> subscriptions_;
    std::vector<std::unique_ptr<TopicHandler>> topic_handlers_;
    std::vector<std::unique_ptr<TopicContext>> topic_contexts_;
    rclcpp::TimerBase::SharedPtr heartbeat_timer_;
    rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DSS_TB3_SimToROSBridgeNode>());
    rclcpp::shutdown();
    return 0;
}
