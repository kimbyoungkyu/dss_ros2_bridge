#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <unistd.h>
#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <nats/nats.h>
#include <nlohmann/json.hpp>
#include "dss.pb.h"
#include "defaultGateway.h"
#include "DSSNavFileStore.h"
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

using json = nlohmann::json;

namespace {
constexpr char kControlSubject[] = "dss.nav.control";
constexpr char kHeartbeatSubject[] = "dss.DSS_TB4_ROSNavControlNode.heartBeat";
constexpr int kMaxRequestBytes = 32 * 1024 * 1024;
std::int64_t NowMilliseconds() 
{
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
}

std::string NowISO8601() 
{
    const auto now = std::chrono::system_clock::now();
    const auto t = std::chrono::system_clock::to_time_t(now);
    std::tm utc{};
    gmtime_r(&t, &utc);
    const auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count() % 1000;
    std::ostringstream out;
    out << std::put_time(&utc, "%Y-%m-%dT%H:%M:%S") << '.' << std::setw(3) << std::setfill('0') << milliseconds << 'Z';
    return out.str();
}

// Synchronous NATS subscription: no callback thread accesses this node.
struct NatsClient {
    natsConnection* connection = nullptr;
    natsSubscription* subscription = nullptr;
    ~NatsClient() {
        if (subscription) natsSubscription_Destroy(subscription);
        if (connection) natsConnection_Destroy(connection);
    }
    NatsClient() = default;
    NatsClient(const NatsClient&) = delete;
    NatsClient& operator=(const NatsClient&) = delete;
};

void CheckNats(natsStatus status, const char* operation) {
    if (status != NATS_OK) {
        throw std::runtime_error(std::string(operation) + ": " + natsStatus_GetText(status));
    }
}
} // namespace

// A permanently running command adapter. Managed targets, not this adapter,
// will implement Lifecycle. No fork/system/spawn and no process launch here.
class DSS_TB4_ROSNavControlNode final : public rclcpp::Node {
public:
    DSS_TB4_ROSNavControlNode() : Node("DSS_TB4_ROSNavControlNode") {
        const char *domain_id = std::getenv("ROS_DOMAIN_ID");
        RCLCPP_INFO(get_logger(), __DATE__);
        RCLCPP_INFO(get_logger(), __TIME__);
        RCLCPP_INFO(get_logger(), "Domain Id=%s", domain_id ? domain_id : "0 (default)");

        const char* home = std::getenv("HOME");
        const std::string default_root = home ? std::string(home) + "/.dss/navigation" : "/tmp/dss_navigation";
        storage_directory_ = declare_parameter<std::string>("storage_directory", default_root);
        if (!std::filesystem::path(storage_directory_).is_absolute()) {
            throw std::invalid_argument("storage_directory must be an absolute path");
        }
        const auto url = declare_parameter<std::string>("nats_url", "nats://" + getDefaultGateway() + ":4222");
        CheckNats(natsConnection_ConnectTo(&nats_.connection, url.c_str()), "NATS connect");
        CheckNats(natsConnection_SubscribeSync(&nats_.subscription, nats_.connection, kControlSubject), "NATS subscribe");
        CheckNats(natsSubscription_SetPendingLimits(nats_.subscription, 8, 64 * 1024 * 1024), "NATS pending limits");
        CheckNats(natsConnection_FlushTimeout(nats_.connection, 2000), "NATS flush");

        registerVisualization();

        // Wall time keeps control/heartbeat working while simulation is paused.
        control_timer_ = create_wall_timer(std::chrono::milliseconds(50),[this] {
             pollControlRequests(); 
        });

        heartbeat_timer_ = create_wall_timer(std::chrono::seconds(3), [this] { 
            publishHeartbeat(); 
        });

        RCLCPP_INFO(get_logger(), "Service ready: %s; storage=%s; Lifecycle not implemented", kControlSubject, storage_directory_.c_str());
        if (useSimTime()){
            RCLCPP_INFO(get_logger(), "DSS TB4 ROs NaV Controller is running in sim_time mode.");
        }else{
            RCLCPP_WARN(get_logger(), "use_sim_time is false: visualization uses ROS system time.");
        }
    }

    ~DSS_TB4_ROSNavControlNode() override {
        if (visualization_timer_) visualization_timer_->cancel();
        if (control_timer_) control_timer_->cancel();
        if (heartbeat_timer_) heartbeat_timer_->cancel();
    }

private:
    static void copyStamp(const builtin_interfaces::msg::Time& src, dss::DssNavStamp* dst) {
        dst->set_sec(src.sec); dst->set_nanosec(src.nanosec);
    }

    template<class V> static void copyVector(const V& src, dss::DSSVector3* dst) {
        dst->set_x(src.x); dst->set_y(src.y); dst->set_z(src.z);
    }

    static void copyQuaternion(const geometry_msgs::msg::Quaternion& src,dss::DSSQuaternion* dst) {
        dst->set_x(src.x); dst->set_y(src.y); dst->set_z(src.z); dst->set_w(src.w);
    }

    static bool validQuaternion(const geometry_msgs::msg::Quaternion& q) {
        const double n = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
        return std::isfinite(n) && std::abs(n - 1.0) < 0.01;
    }
    static void copyTransform(const geometry_msgs::msg::TransformStamped& src,dss::DssNavTransform* dst) {
        const auto& p = src.transform.translation;
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) || !validQuaternion(src.transform.rotation)) throw std::runtime_error("Invalid TF pose");
        copyStamp(src.header.stamp, dst->mutable_stamp());
        dst->set_parent_frame(src.header.frame_id); dst->set_child_frame(src.child_frame_id);
        copyVector(p, dst->mutable_translation());
        copyQuaternion(src.transform.rotation, dst->mutable_rotation());
    }

    bool publishWire(const char* subject, const std::string& bytes) {
        const auto maximum = natsConnection_GetMaxPayload(nats_.connection);
        if (maximum <= 0 || bytes.size() > static_cast<std::size_t>(maximum) ||
            bytes.size() > static_cast<std::size_t>(std::numeric_limits<int>::max())) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,"NATS payload too large for %s (%zu bytes)", subject, bytes.size());
            return false;
        }
        if (natsConnection_Status(nats_.connection) != NATS_CONN_STATUS_CONNECTED) return false;
        const auto status = natsConnection_Publish(nats_.connection, subject,bytes.data(), static_cast<int>(bytes.size()));
        if (status != NATS_OK) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,"Visualization publish failed: %s", natsStatus_GetText(status));
            return false;
        }
        return true;
    }
    template<class Message> void publishProto(const char* subject, const Message& message) {
        std::string bytes;
        if (message.SerializeToString(&bytes)) publishWire(subject, bytes);
    }

    void registerVisualization() {
        map_frame_ = declare_parameter<std::string>("map_frame", "map");
        base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
        const auto map_topic = declare_parameter<std::string>("map_topic", "/map");
        const auto scan_topic = declare_parameter<std::string>("scan_topic", "/scan");
        const bool durable_map = declare_parameter<bool>("map_transient_local", true);
        if (map_frame_.empty() || base_frame_.empty()) throw std::invalid_argument("Empty TF frame");
        stream_id_ = std::to_string(NowMilliseconds()) + "_" + std::to_string(::getpid());
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
        // TF callbacks use this node/executor. Lookups below are non-blocking.
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, false);
        auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
        if (durable_map) map_qos.transient_local();
        map_subscription_ = create_subscription<nav_msgs::msg::OccupancyGrid>(map_topic, map_qos,[this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {
                try { 
                    cacheMap(*msg); 
                }
                catch (const std::exception& e) {
                     RCLCPP_ERROR(get_logger(), "Map: %s", e.what()); 
                }
        });
        scan_subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(scan_topic,rclcpp::SensorDataQoS(),[this](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
                checkTimeReset();
                if (pending_scans_.size() >= 10) {
                    sendScan(*pending_scans_.front().message, nullptr, "TF queue overflow");
                    pending_scans_.pop_front();
                }
                pending_scans_.push_back({msg, std::chrono::steady_clock::now()});
        });
        visualization_timer_ = create_wall_timer(std::chrono::milliseconds(50), [this] {
            checkTimeReset();
            publishRobotPose();
            processScans();
            // Send at most four map chunks per tick; large maps cannot starve control.
            for (int i = 0; i < 4 && map_send_index_ < map_chunks_.size(); ++i) {
                if (!publishWire("dss.nav.map", map_chunks_[map_send_index_])) break;
                ++map_send_index_;
            }
            const auto now = std::chrono::steady_clock::now();
            if (map_send_index_ == map_chunks_.size() && !next_map_chunks_.empty()) {
                map_chunks_ = std::move(next_map_chunks_); next_map_chunks_.clear(); map_send_index_=0;
                next_map_replay_ = now + std::chrono::seconds(5);
            }
            if (!map_chunks_.empty() && map_send_index_ == map_chunks_.size() &&
                now >= next_map_replay_) {
                map_send_index_ = 0;
                next_map_replay_ = now + std::chrono::seconds(5);
            }
        });
    }

    void checkTimeReset() {
        const auto now = get_clock()->now().nanoseconds();
        if (last_ros_time_ >= 0 && now < last_ros_time_) {
            map_chunks_.clear(); next_map_chunks_.clear(); map_send_index_ = 0; pending_scans_.clear();
            stream_id_ = std::to_string(NowMilliseconds()) + "_" + std::to_string(::getpid()) + "_" + std::to_string(++reset_counter_);
            // tf2 handles ROS clock jumps. Do not manually clear static TF here.
            RCLCPP_WARN(get_logger(), "ROS time moved backwards; visualization cache invalidated");
        }
        last_ros_time_ = now;
    }

    void cacheMap(const nav_msgs::msg::OccupancyGrid& src) {
        checkTimeReset();
        const std::uint64_t count = std::uint64_t(src.info.width) * src.info.height;
        if (count == 0 || count > 64ULL*1024*1024 || count != src.data.size())
            throw std::invalid_argument("Invalid map dimensions/data size (limit: 64 Mi cells)");
        if (src.header.frame_id != map_frame_)
            throw std::invalid_argument("Map frame differs from configured map_frame");
        const auto& p = src.info.origin.position;
        if (!std::isfinite(src.info.resolution) || src.info.resolution <= 0 ||
            !std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
            !validQuaternion(src.info.origin.orientation)) throw std::invalid_argument("Invalid map metadata");
        for (const auto value : src.data) {
            if (value < -1 || value > 100) throw std::invalid_argument("Occupancy outside [-1,100]");
        }
        const auto maximum = natsConnection_GetMaxPayload(nats_.connection);
        if (maximum <= 4096) throw std::runtime_error("NATS max_payload too small");
        const std::size_t chunk_size = std::min<std::size_t>(128*1024,
            static_cast<std::size_t>(maximum)-4096);
        dss::DssNavMapChunk chunk;
        chunk.set_schema_version(1); chunk.set_stream_id(stream_id_); chunk.set_map_id(++map_id_);
        copyStamp(src.header.stamp, chunk.mutable_stamp()); chunk.set_frame_id(src.header.frame_id);
        chunk.set_width(src.info.width); chunk.set_height(src.info.height);
        chunk.set_resolution(src.info.resolution);
        copyVector(src.info.origin.position, chunk.mutable_origin()->mutable_position());
        copyQuaternion(src.info.origin.orientation, chunk.mutable_origin()->mutable_orientation());
        copyStamp(src.info.map_load_time, chunk.mutable_map_load_time());
        chunk.set_encoding("raw/int8/row-major"); chunk.set_uncompressed_size(count);
        chunk.set_chunk_count(static_cast<std::uint32_t>((src.data.size()+chunk_size-1)/chunk_size));
        std::vector<std::string> chunks;
        for (std::uint32_t i=0; i<chunk.chunk_count(); ++i) {
            chunk.set_chunk_index(i);
            const auto offset = static_cast<std::size_t>(i)*chunk_size;
            chunk.set_data(reinterpret_cast<const char*>(src.data.data())+offset, std::min(chunk_size, src.data.size()-offset));
            std::string bytes;
            if (!chunk.SerializeToString(&bytes) || bytes.size() > static_cast<std::size_t>(maximum))
                throw std::runtime_error("Map chunk exceeds payload limit");
            chunks.push_back(std::move(bytes));
        }
        // Finish current transfer before switching to a newer map (avoid starvation).
        if (map_send_index_ < map_chunks_.size()) { next_map_chunks_ = std::move(chunks); }
        else { map_chunks_ = std::move(chunks); map_send_index_ = 0; }
        next_map_replay_ = std::chrono::steady_clock::now()+std::chrono::seconds(5);
    }

    void publishRobotPose() {
        dss::DssNavRobotPose msg;
        msg.set_schema_version(1); msg.set_stream_id(stream_id_);
        const auto now = get_clock()->now();
        copyStamp(static_cast<builtin_interfaces::msg::Time>(now), msg.mutable_query_stamp());
        try {
            if (useSimTime() && now.nanoseconds()==0) throw std::runtime_error("Waiting for /clock");
            // BufferCore lookup never enters the timeout/dedicated-thread path.
            auto tf = static_cast<tf2::BufferCore&>(*tf_buffer_).lookupTransform(
                map_frame_, base_frame_, tf2::TimePointZero);
            const double age = (now-rclcpp::Time(tf.header.stamp, get_clock()->get_clock_type())).seconds();
            if (age > 0.5 || age < -0.1) throw std::runtime_error("Robot TF stale or ahead of ROS time");
            copyTransform(tf, msg.mutable_transform()); msg.set_valid(true);
        } catch (const std::exception& e) {
            msg.clear_transform(); msg.set_valid(false); msg.set_error(e.what());
        }
        publishProto("dss.nav.tf", msg);
    }

    void sendScan(const sensor_msgs::msg::LaserScan& src,const geometry_msgs::msg::TransformStamped* tf, const std::string& error) {
        dss::DssNavScan out;
        out.set_schema_version(1); out.set_stream_id(stream_id_);
        copyStamp(src.header.stamp, out.mutable_stamp()); out.set_frame_id(src.header.frame_id);
        out.set_angle_min(src.angle_min); out.set_angle_max(src.angle_max);
        out.set_angle_increment(src.angle_increment); out.set_time_increment(src.time_increment);
        out.set_scan_time(src.scan_time); out.set_range_min(src.range_min); out.set_range_max(src.range_max);
        for (const auto v : src.ranges) out.add_ranges(v);
        for (const auto v : src.intensities) out.add_intensities(v);
        out.set_deskewed(false);
        out.set_transform_valid(false);
        if (tf) {
            try { copyTransform(*tf, out.mutable_map_from_scan()); out.set_transform_valid(true); }
            catch (const std::exception& e) { out.clear_map_from_scan(); out.set_transform_error(e.what()); }
        } else out.set_transform_error(error);
        publishProto("dss.nav.scan", out);
    }

    void processScans() {
        while (!pending_scans_.empty()) {
            const auto pending = pending_scans_.front();
            const auto& scan = *pending.message;
            try {
                if (scan.header.frame_id.empty() || scan.header.stamp.sec < 0 ||
                    (scan.header.stamp.sec==0 && scan.header.stamp.nanosec==0))
                    throw std::runtime_error("Scan has empty frame or zero/negative time");
                const tf2::TimePoint scan_time{std::chrono::nanoseconds(
                    rclcpp::Time(scan.header.stamp, RCL_ROS_TIME).nanoseconds())};
                auto tf = static_cast<tf2::BufferCore&>(*tf_buffer_).lookupTransform(
                    map_frame_, scan.header.frame_id, scan_time);
                sendScan(scan, &tf, "");
                pending_scans_.pop_front();
            } catch (const std::exception& e) {
                if (std::chrono::steady_clock::now()-pending.received < std::chrono::milliseconds(500)) break;
                sendScan(scan, nullptr, e.what()); pending_scans_.pop_front();
            }
        }
    }

    void pollControlRequests() {
        // Bound each tick's work. All handling runs in the single ROS executor.
        for (int i = 0; i < 4; ++i) {
            natsMsg* raw = nullptr;
            const auto status = natsSubscription_NextMsg(&raw, nats_.subscription, 1);
            if (status == NATS_TIMEOUT) return;
            if (status != NATS_OK) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                    "NATS receive: %s", natsStatus_GetText(status));
                return;
            }
            std::unique_ptr<natsMsg, decltype(&natsMsg_Destroy)> message(raw, &natsMsg_Destroy);
            handleControlRequest(message.get());
        }
    }

    void handleControlRequest(natsMsg* message) {
        const char* reply = natsMsg_GetReply(message);
        if (!reply || !*reply) {
            RCLCPP_WARN(get_logger(), "Use NATS request/reply for %s; publication ignored", kControlSubject);
            return;
        }
        dss::DssNavigationControllerRequest request;
        dss::DssNavigationControllerResponse response;
        response.set_timestamp(NowMilliseconds()); // Unix milliseconds
        response.set_success(false);
        try {
            const int length = natsMsg_GetDataLength(message);
            if (length <= 0 || length > kMaxRequestBytes ||
                !request.ParseFromArray(natsMsg_GetData(message), length)) {
                throw std::invalid_argument("Invalid or oversized navigation Protobuf request");
            }
            response.set_identifier(request.identifier());
            if (request.identifier().empty()) throw std::invalid_argument("identifier is required");
            if (!dss::NavControlCommand_IsValid(request.command()) ||
                !dss::NavMode_IsValid(request.mode()) ||
                !dss::SlamType_IsValid(request.slam_type())) {
                throw std::invalid_argument("Unknown command, mode or slam_type");
            }
            switch (request.command()) {
            case dss::NAV_COMMAND_START:
                handleStart(request, response);
                break;
            case dss::NAV_COMMAND_STOP:
                handleStop(request, response);
                break;
            default:
                throw std::invalid_argument("Unsupported navigation command");
            }
        } catch (const std::exception& error) {
            response.set_success(false);
            response.set_message(json{{"stage", "error"}, {"reason", error.what()}}.dump());
            RCLCPP_ERROR(get_logger(), "Nav control: %s", error.what());
        }
        std::string payload;
        if (!response.SerializeToString(&payload)) {
            RCLCPP_ERROR(get_logger(), "Navigation response serialization failed");
            return;
        }
        const auto status = natsConnection_Publish(nats_.connection, reply,
            payload.data(), static_cast<int>(payload.size()));
        if (status != NATS_OK) {
            RCLCPP_ERROR(get_logger(), "Navigation reply failed: %s", natsStatus_GetText(status));
        }
    }

    void handleStart(const dss::DssNavigationControllerRequest& request, dss::DssNavigationControllerResponse& response) {
        std::vector<dss_nav::File> files;
        auto add = [&files](const std::string& name, const std::string& bytes) {
            if (name.empty() && bytes.empty()) return;
            if (name.empty() || bytes.empty()) {
                throw std::invalid_argument("Each supplied file needs both filename and content");
            }
            files.emplace_back(name, bytes);
        };
        add(request.config_filename(), request.config_content());
        add(request.map_yaml_filename(), request.map_yaml_content());
        add(request.map_pgm_filename(), request.map_pgm_data()); // binary, including NUL
        if (files.empty()) throw std::invalid_argument("START skeleton requires at least one file");

        json filenames = json::array();
        for (const auto& file : files) filenames.push_back(file.first);
        // This reserved filename also participates in duplicate-name validation.
        const json manifest{
            {"identifier", request.identifier()}, {"requestTimestamp", request.timestamp()},
            {"savedTimestamp", NowMilliseconds()}, {"command", static_cast<int>(request.command())},
            {"mode", static_cast<int>(request.mode())},
            {"slamType", static_cast<int>(request.slam_type())},
            {"useSimTime", request.use_sim_time()}, {"files", filenames}
        };
        files.emplace_back("dss_nav_request.json", manifest.dump(2));
        const auto directory = dss_nav::SaveFiles(storage_directory_, files);
        last_saved_directory_ = directory.string();
        RCLCPP_INFO(get_logger(), "Navigation files saved: %s", last_saved_directory_.c_str());

        // TODO: validate mode-specific configuration, then asynchronously
        // configure/activate compatible Lifecycle targets. Cartographer needs
        // a real Lifecycle wrapper first. Do not report START success yet.
        response.set_success(false);
        response.set_message(json{
            {"stage", "files_saved"}, {"filesSaved", true},
            {"lifecycleImplemented", false}, {"lifecycleApplied", false},
            {"directory", last_saved_directory_}, {"files", filenames},
            {"reason", "Files saved; Lifecycle START is not implemented yet"}
        }.dump());
    }

    void handleStop(const dss::DssNavigationControllerRequest&, dss::DssNavigationControllerResponse& response) {
        // TODO: asynchronously deactivate managed Lifecycle targets.
        // STOP does not delete saved configuration or map files.
        response.set_success(false);
        response.set_message(json{
            {"stage", "not_implemented"}, {"lifecycleImplemented", false},
            {"lifecycleApplied", false}, {"reason", "Lifecycle STOP is not implemented yet"}
        }.dump());
    }

    void publishHeartbeat() {
        const json heartbeat{
            {"identifier", "DSS_TB4_ROSNavControlNode"}, {"timeStamp", NowISO8601()},
            {"status", "alive"}, {"service", kControlSubject},
            {"lifecycleImplemented", false}, {"lastSavedDirectory", last_saved_directory_}
        };
        const auto payload = heartbeat.dump();
        const auto status = natsConnection_PublishString(
            nats_.connection, kHeartbeatSubject, payload.c_str());
        if (status != NATS_OK) {
            RCLCPP_WARN(get_logger(), "Heartbeat publish failed: %s", natsStatus_GetText(status));
        }
    }

    bool useSimTime() const
    {
        return get_parameter("use_sim_time").as_bool();
    }


    struct PendingScan {
        sensor_msgs::msg::LaserScan::ConstSharedPtr message;
        std::chrono::steady_clock::time_point received;
    };
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
    rclcpp::TimerBase::SharedPtr visualization_timer_;
    std::deque<PendingScan> pending_scans_;
    std::string map_frame_, base_frame_, stream_id_;
    std::vector<std::string> map_chunks_, next_map_chunks_;
    std::size_t map_send_index_ = 0;
    std::uint64_t map_id_ = 0, reset_counter_ = 0;
    std::int64_t last_ros_time_ = -1;
    std::chrono::steady_clock::time_point next_map_replay_{};
    NatsClient nats_;
    std::string storage_directory_;
    std::string last_saved_directory_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::TimerBase::SharedPtr heartbeat_timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    int result = 0;
    try {
        auto node = std::make_shared<DSS_TB4_ROSNavControlNode>();
        rclcpp::spin(node); // SingleThreadedExecutor; do not change without synchronization.
    } catch (const std::exception& error) {
        std::cerr << "DSS_TB4_ROSNavControlNode: " << error.what() << '\n';
        result = 1;
    }
    if (rclcpp::ok()) rclcpp::shutdown();
    nats_Close(); // All NATS connections/subscriptions have been destroyed.
    return result;
}
