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
#include <opencv2/opencv.hpp>
#include <chrono>
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

namespace
{
constexpr std::int64_t kNanosecondsPerSecond = 1'000'000'000LL;
constexpr auto kHeartbeatInterval = std::chrono::seconds{3};
constexpr char kTfSubject[] = "dss.tf";
constexpr char kClockSubject[] = "dss.simTime.clock";
constexpr char kImageSubject[] = "dss.sensor.camera.rgb";
constexpr char kImuSubject[] = "dss.sensor.imu";
constexpr char kLaserScanSubject[] = "dss.sensor.lidar2d";
constexpr char kOdomSubject[] = "dss.sensor.odom";
constexpr char kWheelEncoderSubject[] = "dss.sensor.wheelEncoder";
constexpr char kVelocityCommandSubject[] = "dss.turtlebot4.velcmd";
constexpr char kHeartbeatSubject[] = "dss.DSS_TB4_SimToROSBridgeNode.heartBeat";
}  // namespace

class DSS_TB4_SimToROSBridgeNode final : public rclcpp::Node {
public:
    DSS_TB4_SimToROSBridgeNode() : Node("DSS_TB4_SimToROSBridgeNode") {
        const char *domain_id = std::getenv("ROS_DOMAIN_ID");
        RCLCPP_INFO(get_logger(), __DATE__);
        RCLCPP_INFO(get_logger(), __TIME__);
        RCLCPP_INFO(get_logger(), "Domain Id=%s",domain_id);

        const std::string nats_url = "nats://" + getDefaultGateway() + ":4222";
        RCLCPP_INFO(get_logger(), "Connecting to NATS: %s", nats_url.c_str());
        const natsStatus status = natsConnection_ConnectTo(&nats_connection_, nats_url.c_str());
        if (status != NATS_OK) {
            RCLCPP_ERROR(get_logger(), "NATS connection failed: %s", natsStatus_GetText(status));
            return;
        }
        registTf();
        publishStaticTransforms();
        registClock();
        //registImage();
        registImu();
        registLaserScan();
        registHeartbeat();
        registOdom();
        registWheelEncoder();
        registCmdVel();
        if (useSimTime()){
            RCLCPP_INFO(get_logger(), "DSS TB4 Sim2ROS bridge is running in sim_time mode.");
        }else{
            RCLCPP_WARN(get_logger(), "use_sim_time is false: sensor, clock and dynamic TF publication is disabled.");
        }
    }

    ~DSS_TB4_SimToROSBridgeNode() override
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
                  &DSS_TB4_SimToROSBridgeNode::onNatsMessage, context.get())
            : natsConnection_Subscribe(
                  &subscription, nats_connection_, subject.c_str(),
                  &DSS_TB4_SimToROSBridgeNode::onNatsMessage, context.get());

        if (status != NATS_OK) {
            RCLCPP_ERROR(get_logger(), "NATS subscription failed (%s): %s",
                         subject.c_str(), natsStatus_GetText(status));
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
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        subscribe(kTfSubject, [this](const std::string&, const char* data, int length) {
            dss::DSSTF source;
            if (!source.ParseFromArray(data, length)) {
                RCLCPP_ERROR(get_logger(), "DSSTF protobuf parse failed");
                return;
            }
            if (!useSimTime()) {
                return;
            }
            // Accept only the native frame pair emitted by tTFSensor.
            if (source.parent_frame() != "mujoco_world" || source.child_frame() != "base_frame") {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,"Ignoring DSS TF: expected mujoco_world -> base_frame");
                return;
            }
            const double stamp = source.header().stamp();
            const double qx = -source.qy();
            const double qy = source.qx();
            const double qz = source.qz();
            const double qw = source.qw();
            const double norm = std::sqrt(qx*qx + qy*qy + qz*qz + qw*qw);
            if (!std::isfinite(stamp) || stamp < 0.0 ||
                !std::isfinite(source.x()) || !std::isfinite(source.y()) ||
                !std::isfinite(source.z()) || !std::isfinite(norm) ||
                norm <= std::numeric_limits<double>::epsilon()) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                    "Ignoring invalid DSS TF pose or timestamp");
                return;
            }

            geometry_msgs::msg::TransformStamped transform;
            transform.header.stamp = toRosTime(stamp);
            transform.header.frame_id = "odom";
            transform.child_frame_id = "base_link";
            // Same basis change as createOdom(): C * R * inverse(C).
            transform.transform.translation.x = -source.y();
            transform.transform.translation.y = source.x();
            transform.transform.translation.z = source.z();
            transform.transform.rotation.x = qx / norm;
            transform.transform.rotation.y = qy / norm;
            transform.transform.rotation.z = qz / norm;
            transform.transform.rotation.w = qw / norm;
            tf_broadcaster_->sendTransform(transform);
        });
    }

    void publishStaticTransforms()
    {
        static_tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
        // turtlebot4.xml offsets after (-y, x, z) basis conversion.
        // Both sensor frames have the converted base axes: no extra yaw.
        geometry_msgs::msg::TransformStamped imu;
        imu.header.stamp = toRosTime(0.0);
        imu.header.frame_id = "base_link";
        imu.child_frame_id = "imu_link";
        imu.transform.translation.x = 0.035;
        imu.transform.translation.y = 0.051;
        imu.transform.translation.z = 0.0293;
        imu.transform.rotation.w = 1.0;

        geometry_msgs::msg::TransformStamped laser;
        laser.header.stamp = toRosTime(0.0);
        laser.header.frame_id = "base_link";
        laser.child_frame_id = "laser_link";
        laser.transform.translation.z = 0.08;
        laser.transform.rotation.w = 1.0;

        // Transient-local static broadcaster retains both for late subscribers.
        static_tf_broadcaster_->sendTransform(
            std::vector<geometry_msgs::msg::TransformStamped>{imu, laser});
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
                imu_publisher_->publish(createROSImu(source));
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
                odom_publisher_->publish(createOdom(source));
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
        cmd_vel_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel",
            rclcpp::QoS(10),
            std::bind(
                &DSS_TB4_SimToROSBridgeNode::onCmdVel,
                this,
                std::placeholders::_1));

        RCLCPP_INFO(
            get_logger(),
            "ROS2 /cmd_vel -> NATS %s bridge is ready",
            kVelocityCommandSubject);
    }

    void onCmdVel(const geometry_msgs::msg::Twist::SharedPtr message)
    {
        if (message == nullptr || nats_connection_ == nullptr) {
            return;
        }

        dss::TurtleBot4Control command;
        command.set_identifier("turtlebot4");

        const auto timestamp_ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        command.set_timestamp(timestamp_ms);

        // TurtleBot4Control uses the ROS base-frame velocity convention.
        command.set_linear_x(message->linear.x);
        command.set_linear_y(message->linear.y);
        command.set_angular_z(message->angular.z);

        constexpr double kStoppedEpsilon = 1.0e-6;
        const bool stopped = std::abs(message->linear.x) < kStoppedEpsilon && std::abs(message->linear.y) < kStoppedEpsilon && std::abs(message->angular.z) < kStoppedEpsilon;
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
    static std::array<double, 36> transformCovariance(
        const RepeatedField& source,
        const std::array<double, 6>& fallback_diagonal)
    {
        std::array<double, 36> result{};

        if (source.size() != 36) {
            for (std::size_t i = 0; i < fallback_diagonal.size(); ++i) {
                result[i * 6 + i] = fallback_diagonal[i];
            }
            return result;
        }

        // ROS = C * MuJoCo, where x_ros=-y_mj, y_ros=x_mj, z_ros=z_mj.
        constexpr std::array<int, 6> source_index{1, 0, 2, 4, 3, 5};
        constexpr std::array<double, 6> source_sign{-1.0, 1.0, 1.0,
                                                    -1.0, 1.0, 1.0};

        for (std::size_t row = 0; row < 6; ++row) {
            for (std::size_t column = 0; column < 6; ++column) {
                const auto source_row = static_cast<std::size_t>(source_index[row]);
                const auto source_column =
                    static_cast<std::size_t>(source_index[column]);
                result[row * 6 + column] =
                    source_sign[row] * source_sign[column] *
                    source.Get(static_cast<int>(source_row * 6 + source_column));
            }
        }
        return result;
    }

    nav_msgs::msg::Odometry createOdom(const dss::DSSOdom& source)
    {
        nav_msgs::msg::Odometry message;
        message.header.stamp = toRosTime(source.header().stamp());
        message.header.frame_id = "odom";
        message.child_frame_id = "base_link";

        if (source.has_pose()) {
            const auto& pose = source.pose();

            if (pose.has_position()) {
                message.pose.pose.position.x = -pose.position().y();
                message.pose.pose.position.y = pose.position().x();
                message.pose.pose.position.z = pose.position().z();
            }

            if (pose.has_orientation()) {
                double x = -pose.orientation().y();
                double y = pose.orientation().x();
                double z = pose.orientation().z();
                double w = pose.orientation().w();
                const double norm = std::sqrt(x * x + y * y + z * z + w * w);

                if (norm > std::numeric_limits<double>::epsilon()) {
                    message.pose.pose.orientation.x = x / norm;
                    message.pose.pose.orientation.y = y / norm;
                    message.pose.pose.orientation.z = z / norm;
                    message.pose.pose.orientation.w = w / norm;
                } else {
                    message.pose.pose.orientation.w = 1.0;
                }
            } else {
                message.pose.pose.orientation.w = 1.0;
            }
        } else {
            message.pose.pose.orientation.w = 1.0;
        }

        message.pose.covariance = transformCovariance(
            source.pose_covariance(),
            {1.0e-4, 1.0e-4, 1.0e-3, 1.0e-3, 1.0e-3, 1.0e-4});

        if (source.has_twist()) {
            const auto& twist = source.twist();

            if (twist.has_linear()) {
                message.twist.twist.linear.x = -twist.linear().y();
                message.twist.twist.linear.y = twist.linear().x();
                message.twist.twist.linear.z = twist.linear().z();
            }

            if (twist.has_angular()) {
                message.twist.twist.angular.x = -twist.angular().y();
                message.twist.twist.angular.y = twist.angular().x();
                message.twist.twist.angular.z = twist.angular().z();
            }
        }

        message.twist.covariance = transformCovariance(
            source.twist_covariance(),
            {1.0e-4, 1.0e-4, 1.0e-3, 1.0e-3, 1.0e-3, 1.0e-4});

        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 500,
            "[NATS] %s -> [ROS2] /odom "
            "position=(%.4f, %.4f, %.4f), linear=(%.4f, %.4f, %.4f)",
            kOdomSubject,
            message.pose.pose.position.x,
            message.pose.pose.position.y,
            message.pose.pose.position.z,
            message.twist.twist.linear.x,
            message.twist.twist.linear.y,
            message.twist.twist.linear.z);

        return message;
    }

    sensor_msgs::msg::JointState createJointState(
        const dss::DSSWheelEncoder& source)
    {
        sensor_msgs::msg::JointState message;
        message.header.stamp = toRosTime(source.header().stamp());

        // These names match the MuJoCo joints in turtlebot4.xml.
        message.name = {"left", "right"};
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
        heartbeat_timer_ = create_wall_timer(kHeartbeatInterval,std::bind(&DSS_TB4_SimToROSBridgeNode::publishHeartbeat, this));
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

    
    sensor_msgs::msg::Imu createROSImu(const dss::DSSIMU& source)
    {
        sensor_msgs::msg::Imu message;
        message.header.stamp = toRosTime(source.header().stamp());
        message.header.frame_id = "imu_link";

        /*
        * MuJoCo -> ROS 2 REP-103
        *
        * ROS X = -MuJoCo Y
        * ROS Y =  MuJoCo X
        * ROS Z =  MuJoCo Z
        */

        if (source.has_orientation()) {
            const double mx = source.orientation().x();
            const double my = source.orientation().y();
            const double mz = source.orientation().z();
            const double mw = source.orientation().w();

            /*
            * R_ros = C * R_mujoco * C^-1
            *
            * C는 Z축 +90도 회전이다.
            * Quaternion 성분 변환 결과:
            */
            double rx = -my;
            double ry =  mx;
            double rz =  mz;
            double rw =  mw;

            const double norm = std::sqrt(
                rx * rx +
                ry * ry +
                rz * rz +
                rw * rw);

            if (norm > std::numeric_limits<double>::epsilon()) {
                message.orientation.x = rx / norm;
                message.orientation.y = ry / norm;
                message.orientation.z = rz / norm;
                message.orientation.w = rw / norm;
            } else {
                message.orientation.x = 0.0;
                message.orientation.y = 0.0;
                message.orientation.z = 0.0;
                message.orientation.w = 1.0;
            }
        } else {
            message.orientation.x = 0.0;
            message.orientation.y = 0.0;
            message.orientation.z = 0.0;
            message.orientation.w = 1.0;
        }

        message.orientation_covariance = {
            1.0e-5, 0.0,    0.0,
            0.0,    1.0e-5, 0.0,
            0.0,    0.0,    1.0e-5
        };

        // Angular velocity에 동일한 좌표 변환 적용
        message.angular_velocity.x =
            -source.angular_velocity().y();

        message.angular_velocity.y =
            source.angular_velocity().x();

        message.angular_velocity.z =
            source.angular_velocity().z();

        message.angular_velocity_covariance = {
            1.0e-4, 0.0,    0.0,
            0.0,    1.0e-4, 0.0,
            0.0,    0.0,    1.0e-4        
        };

        // Linear acceleration에 동일한 좌표 변환 적용
        message.linear_acceleration.x = -source.linear_acceleration().y();

        message.linear_acceleration.y = source.linear_acceleration().x();

        message.linear_acceleration.z = source.linear_acceleration().z();

        message.linear_acceleration_covariance = {
            1.0e-2, 0.0,    0.0,
            0.0,    1.0e-2, 0.0,
            0.0,    0.0,    1.0e-2
        };

        RCLCPP_INFO_THROTTLE(
            get_logger(),
            *get_clock(),
            500,
            "[MuJoCo -> ROS2 IMU]\n"
            "  orientation=(%.6f, %.6f, %.6f, %.6f)\n"
            "  angular_velocity=(%.6f, %.6f, %.6f)\n"
            "  linear_acceleration=(%.6f, %.6f, %.6f)",
            message.orientation.x,
            message.orientation.y,
            message.orientation.z,
            message.orientation.w,
            message.angular_velocity.x,
            message.angular_velocity.y,
            message.angular_velocity.z,
            message.linear_acceleration.x,
            message.linear_acceleration.y,
            message.linear_acceleration.z);

        return message;
    }    


    sensor_msgs::msg::LaserScan createLaserScan(const dss::DssLaserScan2D& source) const
    {
        sensor_msgs::msg::LaserScan message;

        message.header.stamp = toRosTime(source.header().stamp());
        message.header.frame_id = "laser_link";

        /*
        * MuJoCo -> ROS 2 REP-103
        *
        * ROS X = -MuJoCo Y
        * ROS Y =  MuJoCo X
        *
        * 따라서 모든 LiDAR 각도에 +90도를 적용한다.
        */
        constexpr float kMujocoToRosYaw = 1.5707963267948966F;  // +π/2

        message.angle_min = source.angle_min() + kMujocoToRosYaw;

        message.angle_max = source.angle_max() + kMujocoToRosYaw;

        // +90도 회전은 회전 방향을 바꾸지 않는다.
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
    rclcpp::spin(std::make_shared<DSS_TB4_SimToROSBridgeNode>());
    rclcpp::shutdown();
    return 0;
}
