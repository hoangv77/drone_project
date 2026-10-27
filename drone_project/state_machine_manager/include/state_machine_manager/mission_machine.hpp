#pragma once

#include <px4_interface_node/msg/drone_state.hpp>
#include <safety_monitor/safety_gate.hpp>
#include <state_machine_manager/msg/mission_state.hpp>
#include <state_machine_manager/msg/stage_feedback.hpp>

#include <chrono>
#include <deque>
#include <string>

namespace state_machine_manager
{
using Mission = msg::MissionState;
using Feedback = msg::StageFeedback;
using Drone = px4_interface_node::msg::DroneState;
using Safety = safety_monitor::msg::SafetyStatus;
using Clock = std::chrono::steady_clock;
using Time = Clock::time_point;
using Milliseconds = std::chrono::milliseconds;

struct Config
{
  Milliseconds telemetry_timeout{500};
  Milliseconds safety_timeout{500};
  Milliseconds feedback_timeout{500};
  Milliseconds arming_handoff_timeout{1000};
  Milliseconds mission_timeout{180000};
  Milliseconds preflight_timeout{30000};
  Milliseconds armed_waiting_timeout{30000};
  Milliseconds throw_timeout{5000};
  Milliseconds stabilize_timeout{10000};
  Milliseconds search_timeout{30000};
  Milliseconds tracking_timeout{60000};
  Milliseconds tag_loss_timeout{1000};
  Milliseconds landing_timeout{60000};
  Milliseconds recovery_timeout{30000};
  bool enable_stage_feedback{false};
  bool allow_flight_warnings{false};
  // Opt-in SITL integration. Default passive behavior is preserved.
  bool native_throw_launch{false};
};

struct Result {bool accepted; std::string reason;};

// A single-threaded deterministic core. All elapsed times come from its caller;
// no ROS executor, PX4 command client, setpoint publisher, or wall clock is used.
class MissionMachine
{
public:
  explicit MissionMachine(const Config & config = {}, Time now = Clock::now(),
    uint64_t first_mission_id = 1);
  void receive_drone(const Drone & drone, Time now);
  void receive_safety(const Safety & safety, Time now);
  Result receive_feedback(const Feedback & feedback, Time now);
  Result start(Time now);
  Result abort(Time now);
  Result reset(Time now);
  void tick(Time now);
  void confirm_launch_arming() {arming_confirmed_ = true;}
  void set_external_command_pending(bool pending) {external_command_pending_ = pending;}
  Mission snapshot(Time now) const;
  static const char * state_name(uint8_t state);

private:
  struct Observation {Feedback feedback; Time received;};
  bool drone_fresh(Time now) const;
  bool safety_fresh(Time now) const;
  bool ground_confirmed(Time now) const;
  bool launch_allowed(Time now) const;
  bool flight_allowed(Time now) const;
  bool visual_allowed(Time now) const;
  bool px4_healthy(Time now) const;
  Milliseconds timeout() const;
  void enter(uint8_t state, const std::string & reason, Time now);
  void stop_normal(const std::string & reason, Time now, uint8_t result = Mission::RESULT_FAILED);
  Result reject(const std::string & reason);
  std::string blocked_reason(Time now) const;

  Config config_;
  safety_monitor::SafetyGate safety_gate_;
  Drone drone_;
  Safety safety_;
  Mission mission_;
  bool have_drone_{false};
  bool have_safety_{false};
  Time drone_rx_{};
  Time safety_rx_{};
  Time entered_;
  Time started_;
  Time last_tag_rx_{};
  Time handoff_started_{};
  bool handoff_pending_{false};
  bool arming_confirmed_{false};
  bool external_command_pending_{false};
  uint64_t next_mission_id_;
  uint64_t last_observation_{0};
  uint8_t pending_result_{Mission::RESULT_FAILED};
  std::deque<Observation> observations_;
};
}  // namespace state_machine_manager
