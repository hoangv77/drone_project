#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <state_machine_manager/mission_machine.hpp>

#include <functional>
#include <memory>
#include <random>
#include <sstream>
#include <stdexcept>

namespace sm = state_machine_manager;

static std::string json_string(const std::string & value)
{
  std::string result = "\"";
  for (unsigned char c : value) {
    if (c == '"' || c == '\\') {result += '\\'; result += static_cast<char>(c);}
    else if (c < 0x20) {result += ' ';}
    else {result += static_cast<char>(c);}
  }
  return result + "\"";
}

class StateMachineManager : public rclcpp::Node
{
public:
  StateMachineManager() : Node("state_machine_manager")
  {
    sm::Config config;
    rcl_interfaces::msg::ParameterDescriptor fixed;
    fixed.read_only = true;
    const auto duration = [this, &fixed](const char * name, sm::Milliseconds initial) {
        const int value = declare_parameter<int>(name, static_cast<int>(initial.count()), fixed);
        if (value < 50 || value > 86400000) {
          throw std::invalid_argument(std::string(name) + " must be in [50, 86400000] ms");
        }
        return sm::Milliseconds(value);
      };
    config.telemetry_timeout = duration("telemetry_timeout_ms", config.telemetry_timeout);
    config.safety_timeout = duration("safety_timeout_ms", config.safety_timeout);
    config.feedback_timeout = duration("feedback_timeout_ms", config.feedback_timeout);
    config.arming_handoff_timeout = duration("arming_handoff_timeout_ms", config.arming_handoff_timeout);
    config.mission_timeout = duration("mission_timeout_ms", config.mission_timeout);
    config.preflight_timeout = duration("preflight_timeout_ms", config.preflight_timeout);
    config.armed_waiting_timeout = duration("armed_waiting_timeout_ms", config.armed_waiting_timeout);
    config.throw_timeout = duration("throw_timeout_ms", config.throw_timeout);
    config.stabilize_timeout = duration("stabilize_timeout_ms", config.stabilize_timeout);
    config.search_timeout = duration("search_timeout_ms", config.search_timeout);
    config.tracking_timeout = duration("tracking_timeout_ms", config.tracking_timeout);
    config.tag_loss_timeout = duration("tag_loss_timeout_ms", config.tag_loss_timeout);
    config.landing_timeout = duration("landing_timeout_ms", config.landing_timeout);
    config.recovery_timeout = duration("recovery_timeout_ms", config.recovery_timeout);
    config.enable_stage_feedback = declare_parameter("enable_stage_feedback", false, fixed);
    config.allow_flight_warnings = declare_parameter("allow_flight_warnings", false, fixed);
    std::random_device random;
    const uint64_t seed = (static_cast<uint64_t>(random()) << 32) ^ random();
    machine_ = std::make_unique<sm::MissionMachine>(config, sm::Clock::now(), seed);
    publisher_ = create_publisher<sm::Mission>("/drone/mission_state",
      rclcpp::QoS(1).reliable().durability_volatile());
    drone_sub_ = create_subscription<sm::Drone>("/drone/state",
      rclcpp::QoS(1).best_effort().durability_volatile(),
      [this](sm::Drone::SharedPtr data) {machine_->receive_drone(*data, sm::Clock::now());});
    safety_sub_ = create_subscription<sm::Safety>("/drone/safety_status",
      rclcpp::QoS(1).reliable().durability_volatile(),
      [this](sm::Safety::SharedPtr data) {machine_->receive_safety(*data, sm::Clock::now());});
    feedback_sub_ = create_subscription<sm::Feedback>("/drone/mission/stage_feedback",
      rclcpp::QoS(1).reliable().durability_volatile(),
      [this](sm::Feedback::SharedPtr data) {
        const auto result = machine_->receive_feedback(*data, sm::Clock::now());
        if (!result.accepted) {RCLCPP_DEBUG(get_logger(), "Feedback rejected: %s", result.reason.c_str());}
      });
    start_service_ = control_service("/drone/mission/start", &sm::MissionMachine::start);
    abort_service_ = control_service("/drone/mission/abort", &sm::MissionMachine::abort);
    reset_service_ = control_service("/drone/mission/reset", &sm::MissionMachine::reset);
    tick_timer_ = create_wall_timer(sm::Milliseconds(20), [this]() {
        machine_->tick(sm::Clock::now());
        log_transition();
      });
    publish_timer_ = create_wall_timer(sm::Milliseconds(100), [this]() {publish();});
    RCLCPP_INFO(get_logger(), "Mission manager started in STANDBY; passive observation only");
  }

private:
  using Trigger = std_srvs::srv::Trigger;
  using Control = sm::Result (sm::MissionMachine::*)(sm::Time);
  rclcpp::Service<Trigger>::SharedPtr control_service(const char * name, Control control)
  {
    return create_service<Trigger>(name,
      [this, control, name](const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response) {
        const auto result = ((*machine_).*control)(sm::Clock::now());
        response->success = result.accepted;
        response->message = result.reason;
        RCLCPP_INFO(get_logger(), "{\"event\":\"operator_request\",\"service\":%s,\"accepted\":%s,\"reason\":%s}",
          json_string(name).c_str(), result.accepted ? "true" : "false", json_string(result.reason).c_str());
        log_transition();
        publish();
      });
  }
  void log_transition()
  {
    const auto state = machine_->snapshot(sm::Clock::now());
    if (state.transition_sequence == last_logged_) {return;}
    last_logged_ = state.transition_sequence;
    std::ostringstream record;
    record << "{\"event\":\"mission_transition\",\"mission_id\":" << state.mission_id <<
      ",\"sequence\":" << state.transition_sequence << ",\"from\":" <<
      json_string(sm::MissionMachine::state_name(state.previous_state)) << ",\"to\":" <<
      json_string(state.state_name) << ",\"reason\":" << json_string(state.transition_reason) << "}";
    RCLCPP_INFO(get_logger(), "%s", record.str().c_str());
    publish();
  }
  void publish()
  {
    auto state = machine_->snapshot(sm::Clock::now());
    state.timestamp = now();
    publisher_->publish(state);
  }
  std::unique_ptr<sm::MissionMachine> machine_;
  uint64_t last_logged_{0};
  rclcpp::Publisher<sm::Mission>::SharedPtr publisher_;
  rclcpp::Subscription<sm::Drone>::SharedPtr drone_sub_;
  rclcpp::Subscription<sm::Safety>::SharedPtr safety_sub_;
  rclcpp::Subscription<sm::Feedback>::SharedPtr feedback_sub_;
  rclcpp::Service<Trigger>::SharedPtr start_service_, abort_service_, reset_service_;
  rclcpp::TimerBase::SharedPtr tick_timer_, publish_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<StateMachineManager>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("state_machine_manager"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
