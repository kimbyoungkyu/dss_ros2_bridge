#include <rclcpp/rclcpp.hpp>
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

std::int64_t NowMilliseconds() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
}

std::string NowISO8601() {
    const auto now = std::chrono::system_clock::now();
    const auto t = std::chrono::system_clock::to_time_t(now);
    std::tm utc{};
    gmtime_r(&t, &utc);
    const auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()).count() % 1000;
    std::ostringstream out;
    out << std::put_time(&utc, "%Y-%m-%dT%H:%M:%S") << '.'
        << std::setw(3) << std::setfill('0') << milliseconds << 'Z';
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
        const char* home = std::getenv("HOME");
        const std::string default_root = home
            ? std::string(home) + "/.dss/navigation" : "/tmp/dss_navigation";
        storage_directory_ = declare_parameter<std::string>("storage_directory", default_root);
        if (!std::filesystem::path(storage_directory_).is_absolute()) {
            throw std::invalid_argument("storage_directory must be an absolute path");
        }
        const auto url = declare_parameter<std::string>(
            "nats_url", "nats://" + getDefaultGateway() + ":4222");
        CheckNats(natsConnection_ConnectTo(&nats_.connection, url.c_str()), "NATS connect");
        CheckNats(natsConnection_SubscribeSync(
            &nats_.subscription, nats_.connection, kControlSubject), "NATS subscribe");
        CheckNats(natsSubscription_SetPendingLimits(
            nats_.subscription, 8, 64 * 1024 * 1024), "NATS pending limits");
        CheckNats(natsConnection_FlushTimeout(nats_.connection, 2000), "NATS flush");

        // Wall time keeps control/heartbeat working while simulation is paused.
        control_timer_ = create_wall_timer(std::chrono::milliseconds(50),
            [this] { pollControlRequests(); });
        heartbeat_timer_ = create_wall_timer(std::chrono::seconds(3),
            [this] { publishHeartbeat(); });
        RCLCPP_INFO(get_logger(), "Service ready: %s; storage=%s; Lifecycle not implemented",
            kControlSubject, storage_directory_.c_str());


        if (useSimTime()){
            RCLCPP_INFO(get_logger(), "DSS TB4 ROs NaV Controller is running in sim_time mode.");
        }else{
            RCLCPP_WARN(get_logger(), "use_sim_time is false: sensor, clock and dynamic TF publication is disabled.");
        }

    }

    ~DSS_TB4_ROSNavControlNode() override {
        if (control_timer_) control_timer_->cancel();
        if (heartbeat_timer_) heartbeat_timer_->cancel();
    }

private:
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

    void handleStart(const dss::DssNavigationControllerRequest& request,
                     dss::DssNavigationControllerResponse& response) {
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

    void handleStop(const dss::DssNavigationControllerRequest&,
                    dss::DssNavigationControllerResponse& response) {
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
