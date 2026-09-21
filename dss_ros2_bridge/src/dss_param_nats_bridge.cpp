#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_type.hpp>
#include <rcl_interfaces/msg/parameter_value.hpp>
#include <rcl_interfaces/srv/describe_parameters.hpp>
#include <rcl_interfaces/srv/get_parameters.hpp>
#include <rcl_interfaces/srv/list_parameters.hpp>
#include <rcl_interfaces/srv/set_parameters_atomically.hpp>

#include <nats/nats.h>
#include <nlohmann/json.hpp>

#include <chrono>
#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <future>
#include <limits>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

using json = nlohmann::json;
using namespace std::chrono_literals;

namespace {
constexpr char kPackageName[] = "dss_param_nats_bridge";
constexpr char kTargetNode[] = "/dss_bridge";
constexpr char kListSubject[] = "dss.param.list";
constexpr char kGetSubject[] = "dss.param.get";
constexpr char kSetSubject[] = "dss.param.set";
constexpr std::size_t kMaximumRequestBytes = 1024 * 1024;

void check_nats(natsStatus status, const char *operation)
{
  if (status != NATS_OK) {
    throw std::runtime_error(std::string(operation) + ": " + natsStatus_GetText(status));
  }
}

struct NatsResources
{
  natsConnection *connection = nullptr;
  natsSubscription *subscription = nullptr;

  ~NatsResources()
  {
    if (subscription) natsSubscription_Destroy(subscription);
    if (connection) natsConnection_Destroy(connection);
  }

  NatsResources() = default;
  NatsResources(const NatsResources &) = delete;
  NatsResources &operator=(const NatsResources &) = delete;
};

std::string type_name(std::uint8_t type)
{
  using Type = rcl_interfaces::msg::ParameterType;
  switch (type) {
    case Type::PARAMETER_BOOL: return "bool";
    case Type::PARAMETER_INTEGER: return "integer";
    case Type::PARAMETER_DOUBLE: return "double";
    case Type::PARAMETER_STRING: return "string";
    case Type::PARAMETER_BYTE_ARRAY: return "byte_array";
    case Type::PARAMETER_BOOL_ARRAY: return "bool_array";
    case Type::PARAMETER_INTEGER_ARRAY: return "integer_array";
    case Type::PARAMETER_DOUBLE_ARRAY: return "double_array";
    case Type::PARAMETER_STRING_ARRAY: return "string_array";
    default: return "not_set";
  }
}

json value_to_json(const rcl_interfaces::msg::ParameterValue &value)
{
  using Type = rcl_interfaces::msg::ParameterType;
  switch (value.type) {
    case Type::PARAMETER_BOOL: return value.bool_value;
    case Type::PARAMETER_INTEGER: return value.integer_value;
    case Type::PARAMETER_DOUBLE: return value.double_value;
    case Type::PARAMETER_STRING: return value.string_value;
    case Type::PARAMETER_BYTE_ARRAY: return value.byte_array_value;
    case Type::PARAMETER_BOOL_ARRAY: return value.bool_array_value;
    case Type::PARAMETER_INTEGER_ARRAY: return value.integer_array_value;
    case Type::PARAMETER_DOUBLE_ARRAY: return value.double_array_value;
    case Type::PARAMETER_STRING_ARRAY: return value.string_array_value;
    case Type::PARAMETER_NOT_SET: return nullptr;
    default: throw std::invalid_argument("Unknown ROS parameter type");
  }
}

rcl_interfaces::msg::ParameterValue json_to_value(const json &input)
{
  using Type = rcl_interfaces::msg::ParameterType;
  rcl_interfaces::msg::ParameterValue result;

  if (input.is_boolean()) {
    result.type = Type::PARAMETER_BOOL;
    result.bool_value = input.get<bool>();
  } else if (input.is_number_integer() || input.is_number_unsigned()) {
    if (input.is_number_unsigned() &&
        input.get<std::uint64_t>() > static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max())) {
      throw std::out_of_range("Unsigned integer exceeds ROS int64 range");
    }
    result.type = Type::PARAMETER_INTEGER;
    result.integer_value = input.get<std::int64_t>();
  } else if (input.is_number_float()) {
    result.type = Type::PARAMETER_DOUBLE;
    result.double_value = input.get<double>();
  } else if (input.is_string()) {
    result.type = Type::PARAMETER_STRING;
    result.string_value = input.get<std::string>();
  } else if (input.is_array()) {
    if (input.empty()) {
      throw std::invalid_argument("Empty JSON arrays have no inferable ROS parameter type");
    }
    if (input.front().is_boolean()) {
      result.type = Type::PARAMETER_BOOL_ARRAY;
      result.bool_array_value = input.get<std::vector<bool>>();
    } else if (input.front().is_number_integer() || input.front().is_number_unsigned()) {
      result.type = Type::PARAMETER_INTEGER_ARRAY;
      result.integer_array_value = input.get<std::vector<std::int64_t>>();
    } else if (input.front().is_number()) {
      result.type = Type::PARAMETER_DOUBLE_ARRAY;
      result.double_array_value = input.get<std::vector<double>>();
    } else if (input.front().is_string()) {
      result.type = Type::PARAMETER_STRING_ARRAY;
      result.string_array_value = input.get<std::vector<std::string>>();
    } else {
      throw std::invalid_argument("Unsupported JSON array parameter type");
    }
  } else {
    throw std::invalid_argument("Parameter value must be bool, integer, double, string, or a typed array");
  }
  return result;
}

rcl_interfaces::msg::ParameterValue json_to_typed_value(const json &input, std::uint8_t expected_type)
{
  using Type = rcl_interfaces::msg::ParameterType;
  rcl_interfaces::msg::ParameterValue result;
  result.type = expected_type;
  switch (expected_type) {
    case Type::PARAMETER_BOOL:
      if (!input.is_boolean()) throw std::invalid_argument("Expected bool value");
      result.bool_value = input.get<bool>(); break;
    case Type::PARAMETER_INTEGER:
      if (!input.is_number_integer() && !input.is_number_unsigned())
        throw std::invalid_argument("Expected integer value");
      if (input.is_number_unsigned() &&
          input.get<std::uint64_t>() > static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max()))
        throw std::out_of_range("Unsigned integer exceeds ROS int64 range");
      result.integer_value = input.get<std::int64_t>(); break;
    case Type::PARAMETER_DOUBLE:
      if (!input.is_number()) throw std::invalid_argument("Expected numeric value");
      result.double_value = input.get<double>(); break;
    case Type::PARAMETER_STRING:
      if (!input.is_string()) throw std::invalid_argument("Expected string value");
      result.string_value = input.get<std::string>(); break;
    case Type::PARAMETER_BYTE_ARRAY:
      if (!input.is_array()) throw std::invalid_argument("Expected byte array");
      for (const auto &item : input) {
        const auto value = item.get<int>();
        if (value < 0 || value > 255) throw std::out_of_range("Byte array values must be in [0,255]");
        result.byte_array_value.push_back(static_cast<std::uint8_t>(value));
      }
      break;
    case Type::PARAMETER_BOOL_ARRAY:
      result.bool_array_value = input.get<std::vector<bool>>(); break;
    case Type::PARAMETER_INTEGER_ARRAY:
      result.integer_array_value = input.get<std::vector<std::int64_t>>(); break;
    case Type::PARAMETER_DOUBLE_ARRAY:
      result.double_array_value = input.get<std::vector<double>>(); break;
    case Type::PARAMETER_STRING_ARRAY:
      result.string_array_value = input.get<std::vector<std::string>>(); break;
    case Type::PARAMETER_NOT_SET:
      return json_to_value(input);
    default:
      throw std::invalid_argument("Unknown ROS parameter type");
  }
  return result;
}
}  // namespace

class DssParamNatsBridge final : public rclcpp::Node
{
public:
  DssParamNatsBridge()
  : Node("dss_param_nats_bridge")
  {
    target_node_ = declare_parameter<std::string>("target_node", kTargetNode);
    nats_url_ = declare_parameter<std::string>("nats_url", "nats://127.0.0.1:4222");
    service_timeout_ = std::chrono::milliseconds(
      declare_parameter<int>("service_timeout_ms", 3000));

    poll_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    client_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    check_nats(natsConnection_ConnectTo(&nats_.connection, nats_url_.c_str()), "NATS connect");
    check_nats(natsConnection_SubscribeSync(&nats_.subscription, nats_.connection, "dss.param.*"),
      "NATS subscribe");
    check_nats(natsSubscription_SetPendingLimits(nats_.subscription, 64, 8 * 1024 * 1024),
      "NATS pending limits");
    check_nats(natsConnection_FlushTimeout(nats_.connection, 2000), "NATS flush");

    poll_timer_ = create_wall_timer(10ms, [this] { poll_requests(); }, poll_group_);
    RCLCPP_INFO(get_logger(), "JSON-string NATS parameter bridge ready: default_target=%s, url=%s",
      target_node_.c_str(), nats_url_.c_str());
  }

  ~DssParamNatsBridge() override
  {
    if (poll_timer_) poll_timer_->cancel();
  }

private:
  template<typename FutureT>
  auto await(FutureT &future, const std::string &service_name)
  {
    if (future.wait_for(service_timeout_) != std::future_status::ready) {
      throw std::runtime_error("ROS service timeout: " + service_name);
    }
    return future.get();
  }

  template<typename ClientT>
  void require_service(const std::shared_ptr<ClientT> &client, const std::string &service_name)
  {
    if (!client->wait_for_service(service_timeout_)) {
      throw std::runtime_error("ROS service unavailable: " + service_name);
    }
  }

  struct ParameterClients
  {
    rclcpp::Client<rcl_interfaces::srv::ListParameters>::SharedPtr list;
    rclcpp::Client<rcl_interfaces::srv::DescribeParameters>::SharedPtr describe;
    rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedPtr get;
    rclcpp::Client<rcl_interfaces::srv::SetParametersAtomically>::SharedPtr set;
  };

  static std::string normalize_node_name(std::string name)
  {
    if (name.empty()) throw std::invalid_argument("node_name cannot be empty");
    if (name.front() != '/') name.insert(name.begin(), '/');
    while (name.size() > 1 && name.back() == '/') name.pop_back();
    return name;
  }

  static std::string legacy_package_name(const std::string &node_name)
  {
    const auto position = node_name.find_last_of('/');
    return position == std::string::npos ? node_name : node_name.substr(position + 1);
  }

  ParameterClients make_clients(const std::string &node_name)
  {
    const auto node = normalize_node_name(node_name);
    return ParameterClients{
      create_client<rcl_interfaces::srv::ListParameters>(
        node + "/list_parameters", rmw_qos_profile_services_default, client_group_),
      create_client<rcl_interfaces::srv::DescribeParameters>(
        node + "/describe_parameters", rmw_qos_profile_services_default, client_group_),
      create_client<rcl_interfaces::srv::GetParameters>(
        node + "/get_parameters", rmw_qos_profile_services_default, client_group_),
      create_client<rcl_interfaces::srv::SetParametersAtomically>(
        node + "/set_parameters_atomically", rmw_qos_profile_services_default, client_group_)
    };
  }

  std::vector<std::string> discover_parameter_nodes() 
  {
    constexpr char suffix[] = "/list_parameters";
    constexpr char service_type[] = "rcl_interfaces/srv/ListParameters";
    std::set<std::string> nodes;
    const auto services = get_node_graph_interface()->get_service_names_and_types();
    for (const auto &[service_name, types] : services) {
      if (service_name.size() <= sizeof(suffix) - 1 ||
          service_name.compare(service_name.size() - (sizeof(suffix) - 1), sizeof(suffix) - 1, suffix) != 0 ||
          std::find(types.begin(), types.end(), service_type) == types.end()) {
        continue;
      }
      nodes.insert(service_name.substr(0, service_name.size() - (sizeof(suffix) - 1)));
    }
    return {nodes.begin(), nodes.end()};
  }

  std::vector<std::string> list_names(const std::string &node_name, ParameterClients &clients)
  {
    require_service(clients.list, node_name + "/list_parameters");
    auto request = std::make_shared<rcl_interfaces::srv::ListParameters::Request>();
    request->depth = rcl_interfaces::srv::ListParameters::Request::DEPTH_RECURSIVE;
    auto future = clients.list->async_send_request(request);
    return await(future, node_name + "/list_parameters")->result.names;
  }

  json describe_node(const std::string &requested_node)
  {
    const auto node_name = normalize_node_name(requested_node);
    auto clients = make_clients(node_name);
    const auto names = list_names(node_name, clients);
    auto response = json{
      {"node_name", node_name},
      {"package_name", legacy_package_name(node_name)},
      {"parameters", json::array()}
    };
    if (names.empty()) return response;

    require_service(clients.describe, node_name + "/describe_parameters");
    require_service(clients.get, node_name + "/get_parameters");
    auto describe_request = std::make_shared<rcl_interfaces::srv::DescribeParameters::Request>();
    auto get_request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
    describe_request->names = names;
    get_request->names = names;
    auto describe_future = clients.describe->async_send_request(describe_request);
    auto get_future = clients.get->async_send_request(get_request);
    const auto descriptions = await(describe_future, node_name + "/describe_parameters");
    const auto values = await(get_future, node_name + "/get_parameters");
    if (descriptions->descriptors.size() != names.size() || values->values.size() != names.size()) {
      throw std::runtime_error("Target returned inconsistent parameter result counts");
    }
    for (std::size_t i = 0; i < names.size(); ++i) {
      response["parameters"].push_back({
        {"name", names[i]},
        {"type", type_name(descriptions->descriptors[i].type)},
        {"value", value_to_json(values->values[i])}
      });
    }
    return response;
  }

  json handle_list(const json &request_json)
  {
    if (!request_json.is_object()) {
      throw std::invalid_argument("dss.param.list payload must be an object");
    }
    if (request_json.contains("node_name")) {
      return describe_node(request_json.at("node_name").get<std::string>());
    }

    auto response = json{
      {"package_name", kPackageName},
      {"nodes", json::array()}
    };
    const auto nodes = discover_parameter_nodes();
    for (const auto &node_name : nodes) {
      try {
        response["nodes"].push_back(describe_node(node_name));
      } catch (const std::exception &error) {
        response["nodes"].push_back(json{
          {"node_name", node_name},
          {"package_name", legacy_package_name(node_name)},
          {"parameters", json::array()},
          {"error", error.what()}
        });
      }
    }
    response["node_count"] = response["nodes"].size();
    return response;
  }

  json handle_get(const json &request_json)
  {
    std::string node_name = target_node_;
    std::vector<std::string> names;
    if (request_json.is_object() && request_json.empty()) {
      // Backward-compatible: {} means all parameters on the default target.
    } else if (request_json.is_array()) {
      names = request_json.get<std::vector<std::string>>();
    } else if (request_json.is_object()) {
      node_name = request_json.value("node_name", target_node_);
      if (request_json.contains("names")) {
        names = request_json.at("names").get<std::vector<std::string>>();
      }
    } else {
      throw std::invalid_argument(
        "dss.param.get payload must be {}, an array, or {node_name,names}");
    }

    node_name = normalize_node_name(node_name);
    auto clients = make_clients(node_name);
    if (names.empty()) names = list_names(node_name, clients);
    auto response = json{
      {"node_name", node_name},
      {"package_name", legacy_package_name(node_name)},
      {"values", json::object()}
    };
    if (names.empty()) return response;
    require_service(clients.get, node_name + "/get_parameters");
    auto request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
    request->names = names;
    auto future = clients.get->async_send_request(request);
    const auto result = await(future, node_name + "/get_parameters");
    if (result->values.size() != names.size()) {
      throw std::runtime_error("Target returned inconsistent parameter value count");
    }
    for (std::size_t i = 0; i < names.size(); ++i) {
      response["values"][names[i]] = value_to_json(result->values[i]);
    }
    return response;
  }

  json handle_set(const json &request_json)
  {
    if (!request_json.is_object() || request_json.empty()) {
      throw std::invalid_argument("dss.param.set payload must be a non-empty JSON object");
    }
    std::string node_name = target_node_;
    const json *values = &request_json;
    if (request_json.contains("node_name") || request_json.contains("values")) {
      node_name = request_json.value("node_name", target_node_);
      if (!request_json.contains("values") || !request_json.at("values").is_object()) {
        throw std::invalid_argument("Expanded set payload requires an object field named values");
      }
      values = &request_json.at("values");
    }
    if (values->empty()) throw std::invalid_argument("No parameter values supplied");

    node_name = normalize_node_name(node_name);
    auto clients = make_clients(node_name);
    require_service(clients.set, node_name + "/set_parameters_atomically");
    require_service(clients.describe, node_name + "/describe_parameters");

    std::vector<std::string> names;
    names.reserve(values->size());
    for (const auto &[name, unused] : values->items()) {
      (void)unused;
      if (name.empty()) throw std::invalid_argument("Parameter name cannot be empty");
      names.push_back(name);
    }
    auto describe_request = std::make_shared<rcl_interfaces::srv::DescribeParameters::Request>();
    describe_request->names = names;
    auto describe_future = clients.describe->async_send_request(describe_request);
    const auto descriptions = await(
      describe_future, node_name + "/describe_parameters");
    if (descriptions->descriptors.size() != names.size()) {
      throw std::runtime_error("Target returned inconsistent parameter descriptor count");
    }

    auto request = std::make_shared<rcl_interfaces::srv::SetParametersAtomically::Request>();
    for (std::size_t i = 0; i < names.size(); ++i) {
      const auto &name = names[i];
      rcl_interfaces::msg::Parameter parameter;
      parameter.name = name;
      try {
        parameter.value = json_to_typed_value(values->at(name), descriptions->descriptors[i].type);
      } catch (const std::exception &error) {
        throw std::invalid_argument(name + ": " + error.what());
      }
      request->parameters.push_back(std::move(parameter));
    }
    auto future = clients.set->async_send_request(request);
    const auto result = await(future, node_name + "/set_parameters_atomically")->result;
    return json{
      {"node_name", node_name},
      {"package_name", legacy_package_name(node_name)},
      {"success", result.successful},
      {"message", result.successful ? "Parameters updated successfully" : result.reason}
    };
  }

  void poll_requests()
  {
    for (int i = 0; i < 8; ++i) {
      natsMsg *raw = nullptr;
      const auto status = natsSubscription_NextMsg(&raw, nats_.subscription, 0);
      if (status == NATS_TIMEOUT) return;
      if (status != NATS_OK) {
        RCLCPP_ERROR(get_logger(), "NATS receive failed: %s", natsStatus_GetText(status));
        return;
      }
      std::unique_ptr<natsMsg, decltype(&natsMsg_Destroy)> message(raw, &natsMsg_Destroy);
      handle_request(message.get());
    }
  }

  void handle_request(natsMsg *message)
  {
    const char *reply = natsMsg_GetReply(message);
    if (!reply || !*reply) {
      RCLCPP_WARN(get_logger(), "NATS request/reply is required for %s", natsMsg_GetSubject(message));
      return;
    }

    json response;
    try {
      const int length = natsMsg_GetDataLength(message);
      if (length < 0 || static_cast<std::size_t>(length) > kMaximumRequestBytes) {
        throw std::invalid_argument("Invalid or oversized JSON request");
      }
      const char *data = natsMsg_GetData(message);
      const json request_json = length == 0 ? json::object() : json::parse(data, data + length);
      const std::string subject = natsMsg_GetSubject(message);
      if (subject == kListSubject) response = handle_list(request_json);
      else if (subject == kGetSubject) response = handle_get(request_json);
      else if (subject == kSetSubject) response = handle_set(request_json);
      else throw std::invalid_argument("Unsupported NATS subject: " + subject);
    } catch (const std::exception &error) {
      response = json{{"package_name", kPackageName}, {"success", false}, {"message", error.what()}};
      RCLCPP_ERROR(get_logger(), "Parameter request failed: %s", error.what());
    }

    const std::string payload = response.dump();
    const auto status = natsConnection_Publish(
      nats_.connection, reply, payload.data(), static_cast<int>(payload.size()));
    if (status != NATS_OK) {
      RCLCPP_ERROR(get_logger(), "NATS reply failed: %s", natsStatus_GetText(status));
    }
  }

  std::string target_node_;
  std::string nats_url_;
  std::chrono::milliseconds service_timeout_{3000};
  NatsResources nats_;
  rclcpp::CallbackGroup::SharedPtr poll_group_;
  rclcpp::CallbackGroup::SharedPtr client_group_;
  rclcpp::TimerBase::SharedPtr poll_timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    auto node = std::make_shared<DssParamNatsBridge>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
    executor.add_node(node);
    executor.spin();
  } catch (const std::exception &error) {
    RCLCPP_FATAL(rclcpp::get_logger("dss_param_nats_bridge"), "%s", error.what());
    result = 1;
  }
  if (rclcpp::ok()) rclcpp::shutdown();
  nats_Close();
  return result;
}
