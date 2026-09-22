// Model-independent, per-goal Nav2 ingress. No actuation: publishes typed input
// to the existing Bindu lease/execution path. RMW GID is checked per message.
#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <bindu_interfaces/msg/navigation_velocity.hpp>
#include <std_msgs/msg/string.hpp>

class IdentityBridge : public rclcpp::Node {
public:
  IdentityBridge(const std::string &goal, const std::string &source,
                 const std::string &gid, double start)
      : Node("velocity_identity"), goal_(goal), source_(source), start_(start) {
    if (gid.size() != RMW_GID_STORAGE_SIZE * 2) throw std::runtime_error("INVALID_PUBLISHER_GID");
    for (size_t i = 0; i < expected_.size(); ++i)
      expected_[i] = static_cast<uint8_t>(std::stoul(gid.substr(i*2, 2), nullptr, 16));
    declare_parameter<std::string>("goal_id", goal_);
    binding_ = add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter> &params) {
      rcl_interfaces::msg::SetParametersResult result; result.successful = true;
      for (const auto &param : params) {
        if (param.get_name() != "goal_id") continue;
        if (!goal_.empty() || param.get_type() != rclcpp::ParameterType::PARAMETER_STRING ||
            param.as_string().size() != 32 ||
            !std::all_of(param.as_string().begin(), param.as_string().end(),
              [](char c) { return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f'); })) {
          result.successful = false; result.reason = "GOAL_BINDING_IMMUTABLE"; return result;
        }
      }
      for (const auto &param : params) if (param.get_name() == "goal_id") {
        goal_ = param.as_string(); start_ = now().seconds(); last_ = 0.;
      }
      return result;
    });
    output_ = create_publisher<bindu_interfaces::msg::NavigationVelocity>("identified_velocity", 10);
    fault_ = create_publisher<std_msgs::msg::String>("identity_fault", rclcpp::QoS(1).transient_local());
    input_ = create_subscription<geometry_msgs::msg::TwistStamped>("cmd_vel", 10,
      [this](geometry_msgs::msg::TwistStamped::ConstSharedPtr msg, const rclcpp::MessageInfo &info) {
        if (failed_ || goal_.empty()) return;
        const auto &gid = info.get_rmw_message_info().publisher_gid;
        if (!std::equal(expected_.begin(), expected_.end(), gid.data)) {
          fail("NAV2_PUBLISHER_CHANGED"); return;
        }
        double stamp = rclcpp::Time(msg->header.stamp).seconds();
        if (!std::isfinite(stamp) || stamp < start_ || stamp <= last_) {
          fail("NAV2_COMMAND_STALE"); return;
        }
        last_ = stamp;
        bindu_interfaces::msg::NavigationVelocity out;
        out.goal_id = goal_; out.source_id = source_; out.sequence = ++sequence_;
        out.command = *msg;
        output_->publish(out);
      });
  }
private:
  void fail(const std::string &code) {
    failed_ = true;
    std_msgs::msg::String message; message.data = code; fault_->publish(message);
    RCLCPP_ERROR(get_logger(), "%s", code.c_str());
  }
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr binding_;
  std::string goal_, source_;
  std::array<uint8_t, RMW_GID_STORAGE_SIZE> expected_{};
  double start_, last_{0.}; uint64_t sequence_{0}; bool failed_{false};
  rclcpp::Publisher<bindu_interfaces::msg::NavigationVelocity>::SharedPtr output_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr fault_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr input_;
};
int main(int argc, char **argv) {
  if (argc < 5) return 2;
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<IdentityBridge>(argv[1], argv[2], argv[3], std::stod(argv[4])));
    rclcpp::shutdown(); return 0;
  } catch (const std::exception &e) {
    std::cerr << e.what() << std::endl; rclcpp::shutdown(); return 1;
  }
}
