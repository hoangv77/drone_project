#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <safety_monitor/safety_policy.hpp>

#include <chrono>
#include <memory>
#include <stdexcept>

class SafetyMonitor : public rclcpp::Node
{
public:
  SafetyMonitor() : Node("safety_monitor")
  {
    rcl_interfaces::msg::ParameterDescriptor fixed;
    fixed.read_only = true;
    policy_.battery_min_launch = declare_parameter("battery_min_launch", 0.30, fixed);
    policy_.battery_min_flight = declare_parameter("battery_min_flight", 0.20, fixed);
    policy_.battery_emergency = declare_parameter("battery_emergency", 0.10, fixed);
    if (!std::isfinite(policy_.battery_min_launch) ||
      !std::isfinite(policy_.battery_min_flight) || !std::isfinite(policy_.battery_emergency) ||
      !(0.0 < policy_.battery_emergency && policy_.battery_emergency < policy_.battery_min_flight &&
      policy_.battery_min_flight < policy_.battery_min_launch && policy_.battery_min_launch <= 1.0))
    {
      throw std::invalid_argument("Require 0 < battery_emergency < battery_min_flight < battery_min_launch <= 1");
    }
    policy_.require_gnss = declare_parameter("require_gnss", true, fixed);
    policy_.require_gcs = declare_parameter("require_gcs", true, fixed);
    policy_.require_rc = declare_parameter("require_rc", true, fixed);
    policy_.launch_modes = declare_parameter("launch_modes", policy_.launch_modes, fixed);
    policy_.autonomous_modes = declare_parameter("autonomous_modes", policy_.autonomous_modes, fixed);
    // Configuration may narrow these sets, but cannot authorize operator modes
    // or unfinished visual guidance as autonomous behavior.
    const safety_monitor::Policy defaults;
    for (const auto & mode : policy_.launch_modes) {
      if (!safety_monitor::contains(defaults.launch_modes, mode)) {
        throw std::invalid_argument("launch_modes must be a subset of LOITER, POSITION");
      }
    }
    for (const auto & mode : policy_.autonomous_modes) {
      if (!safety_monitor::contains(defaults.autonomous_modes, mode)) {
        throw std::invalid_argument("autonomous_modes contains an unsupported autonomous mode");
      }
    }
    const int timeout_ms = declare_parameter("state_timeout_ms", 500, fixed);
    if (timeout_ms < 100 || timeout_ms > 2000) {
      throw std::invalid_argument("state_timeout_ms must be in [100, 2000]");
    }
    state_timeout_ = std::chrono::milliseconds(timeout_ms);
    publisher_ = create_publisher<safety_monitor::Status>(
      "/drone/safety_status", rclcpp::QoS(1).reliable().durability_volatile());
    subscription_ = create_subscription<safety_monitor::State>(
      "/drone/state", rclcpp::QoS(1).best_effort().durability_volatile(),
      [this](safety_monitor::State::SharedPtr state) {
        state_ = *state;
        received_ = true;
        last_receive_ = std::chrono::steady_clock::now();
        // Never let a lost status stream turn an armed vehicle into a benign
        // ground failure. Clear this latch only with both fresh confirmations.
        if (state_.armed || (state_.land_detected_fresh && !state_.landed)) {
          flight_active_ = true;
        } else if (state_.status_fresh && state_.land_detected_fresh && state_.landed) {
          flight_active_ = false;
        }
      });
    timer_ = create_wall_timer(std::chrono::milliseconds(100), [this]() {publish();});
    RCLCPP_INFO(get_logger(), "Safety permissions publishing at 10 Hz; RC=%s GCS=%s GNSS=%s required",
      policy_.require_rc ? "yes" : "no", policy_.require_gcs ? "yes" : "no",
      policy_.require_gnss ? "yes" : "no");
  }

private:
  void publish()
  {
    safety_monitor::Status status;
    if (!received_) {
      status.level = safety_monitor::Status::UNKNOWN;
      status.phase = safety_monitor::Status::PHASE_UNKNOWN;
      status.reason = "Waiting for /drone/state";
      status.launch_block_reason = status.reason;
      status.autonomous_flight_block_reason = status.reason;
      status.visual_landing_block_reason = status.reason;
    } else {
      const bool fresh = std::chrono::steady_clock::now() - last_receive_ <= state_timeout_;
      status = safety_monitor::evaluate(state_, policy_, fresh, flight_active_);
    }
    status.timestamp = now();
    if (status.level != last_level_ || status.reason != last_reason_) {
      RCLCPP_INFO(get_logger(), "Safety level %u: %s", status.level, status.reason.c_str());
      last_level_ = status.level;
      last_reason_ = status.reason;
    }
    publisher_->publish(status);
  }
  safety_monitor::Policy policy_;
  safety_monitor::State state_;
  bool received_{false};
  bool flight_active_{false};
  uint8_t last_level_{255};
  std::string last_reason_;
  std::chrono::steady_clock::time_point last_receive_{};
  std::chrono::milliseconds state_timeout_{500};
  rclcpp::Subscription<safety_monitor::State>::SharedPtr subscription_;
  rclcpp::Publisher<safety_monitor::Status>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<SafetyMonitor>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("safety_monitor"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
