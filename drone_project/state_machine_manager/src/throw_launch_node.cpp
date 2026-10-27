#include <state_machine_manager/mission_machine.hpp>
#include <state_machine_manager/msg/native_launch_status.hpp>
#include <px4_interface_node/srv/execute_command.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <algorithm>
#include <cmath>
#include <memory>
#include <optional>

namespace sm = state_machine_manager;
using Command = px4_interface_node::srv::ExecuteCommand;
using Native = sm::msg::NativeLaunchStatus;
using Trigger = std_srvs::srv::Trigger;

// Owns the same deterministic mission core as the passive manager. Run only one
// manager. This executable is installed separately and never launched by default.
class ThrowLaunchNode : public rclcpp::Node
{
public:
  ThrowLaunchNode() : Node("throw_launch_manager")
  {
    if (!declare_parameter("sitl_only", true)) {
      throw std::invalid_argument("Physical throw launch is not implemented or approved");
    }
    sm::Config config;
    config.enable_stage_feedback = true;
    config.native_throw_launch = true;
    // Airborne land detection precedes native motor release at the throw apex.
    config.arming_handoff_timeout = sm::Milliseconds(2500);
    config.armed_waiting_timeout = sm::Milliseconds(15000);
    config.stabilize_timeout = sm::Milliseconds(15000);
    machine_ = std::make_unique<sm::MissionMachine>(config);
    state_pub_ = create_publisher<sm::Mission>("/drone/mission_state", 1);
    client_ = create_client<Command>("/drone/execute_command");
    drone_sub_ = create_subscription<sm::Drone>("/drone/state", rclcpp::QoS(1).best_effort(),
      [this](sm::Drone::SharedPtr d) {
        drone_ = *d; drone_rx_ = sm::Clock::now(); machine_->receive_drone(*d, drone_rx_);
        if (awaiting_command_state_ && d->status_fresh && !d->command_pending &&
          d->armed == expected_armed_ && d->flight_mode == expected_mode_)
        {
          awaiting_command_state_ = false;
          machine_->set_external_command_pending(false);
        }
      });
    safety_sub_ = create_subscription<sm::Safety>("/drone/safety_status", 1,
      [this](sm::Safety::SharedPtr s) {machine_->receive_safety(*s, sm::Clock::now());});
    native_sub_ = create_subscription<Native>("/simulation/native_launch_status", 1,
      [this](Native::SharedPtr n) {
        if (n->px4_timestamp <= native_.px4_timestamp) {return;}
        native_ = *n; native_rx_ = sm::Clock::now();
        if (active_ && n->armed && n->lockdown) {saw_lockdown_ = true;}
      });
    attitude_sub_ = create_subscription<px4_msgs::msg::VehicleAttitude>(
      "/fmu/out/vehicle_attitude", rclcpp::SensorDataQoS(),
      [this](px4_msgs::msg::VehicleAttitude::SharedPtr a) {
        if (a->timestamp <= attitude_timestamp_) {return;}
        attitude_timestamp_ = a->timestamp;
        double norm = 0;
        for (auto q : a->q) {norm += q * q;}
        if (!std::isfinite(norm) || std::abs(norm - 1.0) > 0.02) {tilt_ = INFINITY; return;}
        tilt_ = std::acos(std::clamp(1.0 - 2.0 * (a->q[1] * a->q[1] +
          a->q[2] * a->q[2]), -1.0, 1.0));
        attitude_rx_ = sm::Clock::now();
      });
    start_ = create_service<Trigger>("/drone/mission/start",
      [this](Trigger::Request::SharedPtr, Trigger::Response::SharedPtr r) {
        const auto t = sm::Clock::now();
        if (active_ || pending_ || awaiting_command_state_ || !native_fresh(t) || native_.armed || !native_.lockdown ||
          native_.kill || native_.termination || !client_->service_is_ready())
        {
          r->success = false;
          r->message = "Requires fresh SITL throw-enabled lockdown, disarmed state, and command service";
          return;
        }
        const auto result = machine_->start(t);
        r->success = result.accepted; r->message = result.reason;
        if (result.accepted) {
          active_ = true; saw_lockdown_ = false; unlocked_ = false; command_step_ = 0;
          recovery_attempts_ = 0; stable_since_.reset();
          issue(Command::Request::RUN_PREFLIGHT_CHECKS);
        }
      });
    abort_ = create_service<Trigger>("/drone/mission/abort",
      [this](Trigger::Request::SharedPtr, Trigger::Response::SharedPtr r) {
        const auto result = machine_->abort(sm::Clock::now());
        r->success = result.accepted; r->message = result.reason;
        if (result.accepted) {active_ = false; stable_since_.reset();}
      });
    reset_ = create_service<Trigger>("/drone/mission/reset",
      [this](Trigger::Request::SharedPtr, Trigger::Response::SharedPtr r) {
        if (pending_ || awaiting_command_state_) {r->success = false; r->message = "Command/state handoff still pending"; return;}
        const auto result = machine_->reset(sm::Clock::now());
        r->success = result.accepted; r->message = result.reason;
      });
    timer_ = create_wall_timer(sm::Milliseconds(20), [this]() {tick();});
  }

private:
  bool native_fresh(sm::Time t) const
  {return native_.px4_timestamp != 0 && t >= native_rx_ && t - native_rx_ < sm::Milliseconds(1200);}

  void feedback(uint8_t event, const std::string & detail, sm::Time t = sm::Clock::now())
  {
    const auto state = machine_->snapshot(t);
    sm::Feedback f;
    f.mission_id = state.mission_id; f.transition_sequence = state.transition_sequence;
    f.observation_sequence = ++observation_; f.verified = true; f.event = event; f.detail = detail;
    machine_->receive_feedback(f, t);
  }

  void fail(const std::string & reason)
  {
    RCLCPP_ERROR(get_logger(), "%s", reason.c_str());
    feedback(sm::Feedback::STAGE_FAILED, reason);
    active_ = false; stable_since_.reset();
  }

  void issue(uint8_t kind, const std::string & mode = "", bool recovery = false)
  {
    if (pending_ || awaiting_command_state_ || !client_->service_is_ready()) {return;}
    auto request = std::make_shared<Command::Request>();
    request->command = kind; request->mode = mode;
    pending_ = true; command_started_ = sm::Clock::now();
    machine_->set_external_command_pending(true);
    client_->async_send_request(request,
      [this, recovery](rclcpp::Client<Command>::SharedFuture future) {
        pending_ = false;
        const auto response = future.get();
        // Keep the core's local in-flight latch until the state stream catches
        // up to the service result. An abort before the first 10 Hz pending
        // snapshot must not complete on cached disarmed telemetry.
        awaiting_command_state_ = true;
        expected_armed_ = response->armed;
        expected_mode_ = response->current_mode;
        const bool confirmed = response->success && response->accepted && response->status_fresh &&
          response->result == Command::Response::SUCCESS;
        if (recovery) {
          RCLCPP_INFO(get_logger(), "Recovery command confirmed=%s: %s",
            confirmed ? "true" : "false", response->message.c_str());
          return;
        }
        // An abort may arrive while ARM is in flight. Never continue its sequence.
        if (!active_) {return;}
        if (!confirmed) {fail("PX4 command failed: " + response->message); return;}
        ++command_step_;
      });
  }

  void tick()
  {
    const auto t = sm::Clock::now();
    // Evaluate the newest safety/status inputs before any command side effect.
    machine_->tick(t);
    auto state = machine_->snapshot(t);
    if (state.current_state == sm::Mission::RECOVERY || state.current_state == sm::Mission::ABORTED ||
      state.current_state == sm::Mission::FAULT || state.current_state == sm::Mission::STANDBY)
    {active_ = false;}
    if (pending_ && t - command_started_ > sm::Milliseconds(9000) && active_) {
      fail("Command service response timeout; request outcome uncertain, no retry of arming");
    }
    if (active_ && (!native_fresh(t) || native_.kill || native_.termination)) {
      fail("Native launch status lost or PX4 kill/termination active");
    }
    if (active_ && command_step_ == 3 && saw_lockdown_ && native_fresh(t) &&
      native_.armed && native_.lockdown)
    {machine_->confirm_launch_arming();}
    if (active_ && state.current_state == sm::Mission::PREFLIGHT && !pending_ && !awaiting_command_state_) {
      if (command_step_ == 1) {issue(Command::Request::SET_MODE, "LOITER");}
      else if (command_step_ == 2) {issue(Command::Request::ARM);}
    }
    // A lockdown->unlocked transition while armed is PX4's native detection
    // evidence. Speed thresholds are never reimplemented here.
    if (active_ && native_fresh(t) && saw_lockdown_ && native_.armed && !native_.lockdown) {
      unlocked_ = true;
    }
    if (active_ && state.current_state == sm::Mission::ARMED_WAITING && unlocked_ && !drone_.landed) {
      feedback(sm::Feedback::THROW_DETECTED, "PX4 released native actuator lockdown and reports airborne", t);
    } else if (active_ && state.current_state == sm::Mission::THROW_LAUNCH && unlocked_) {
      feedback(sm::Feedback::LAUNCH_COMPLETE, "PX4 native throw phase completed", t);
    } else if (active_ && state.current_state == sm::Mission::STABILIZE) {
      const double speed = std::sqrt(drone_.vx * drone_.vx + drone_.vy * drone_.vy + drone_.vz * drone_.vz);
      const bool stable = t - drone_rx_ < sm::Milliseconds(500) && drone_.local_position_fresh &&
        drone_.position_valid && !drone_.landed && drone_.armed && drone_.flight_mode == "LOITER" &&
        attitude_timestamp_ != 0 && t - attitude_rx_ < sm::Milliseconds(300) &&
        std::isfinite(speed) && speed < 0.7 && std::isfinite(tilt_) && tilt_ < 0.17;
      if (!stable) {stable_since_.reset();}
      else if (!stable_since_) {stable_since_ = t;}
      else if (t - *stable_since_ >= sm::Milliseconds(1000)) {
        feedback(sm::Feedback::STABILIZED, "One second: tilt <0.17 rad and speed <0.7 m/s in LOITER", t);
      }
    }
    machine_->tick(t);
    state = machine_->snapshot(t);
    if (state.current_state == sm::Mission::RECOVERY || state.current_state == sm::Mission::ABORTED ||
      state.current_state == sm::Mission::FAULT || state.current_state == sm::Mission::STANDBY)
    {active_ = false;}
    if (state.current_state == sm::Mission::RECOVERY && !pending_ && !awaiting_command_state_ && recovery_attempts_ < 3 &&
      t - drone_rx_ < sm::Milliseconds(500) && drone_.status_fresh && drone_.land_detected_fresh &&
      drone_.armed && !drone_.command_pending && t - recovery_last_ >= sm::Milliseconds(1000))
    {
      // Only native lockdown confirms a pre-release vehicle. Never disarm an
      // airborne or unlocked vehicle, even if the land detector is lagging.
      recovery_last_ = t; ++recovery_attempts_;
      if (drone_.landed && native_fresh(t) && native_.armed && native_.lockdown) {
        issue(Command::Request::DISARM, "", true);
      } else {issue(Command::Request::SET_MODE, "LAND", true);}
    }
    if (t - published_ >= sm::Milliseconds(100) || state.transition_sequence != last_transition_) {
      if (state.transition_sequence != last_transition_) {
        RCLCPP_INFO(get_logger(), "Mission %s: %s", state.state_name.c_str(), state.transition_reason.c_str());
        last_transition_ = state.transition_sequence;
      }
      state.timestamp = now(); state_pub_->publish(state); published_ = t;
    }
  }

  std::unique_ptr<sm::MissionMachine> machine_;
  sm::Drone drone_;
  Native native_;
  sm::Time drone_rx_{}, native_rx_{}, attitude_rx_{}, command_started_{}, published_{}, recovery_last_{};
  std::optional<sm::Time> stable_since_;
  double tilt_{INFINITY};
  uint64_t attitude_timestamp_{0}, observation_{0}, last_transition_{0};
  bool active_{false}, pending_{false}, saw_lockdown_{false}, unlocked_{false};
  bool awaiting_command_state_{false}, expected_armed_{false};
  std::string expected_mode_;
  int command_step_{0}, recovery_attempts_{0};
  rclcpp::Publisher<sm::Mission>::SharedPtr state_pub_;
  rclcpp::Subscription<sm::Drone>::SharedPtr drone_sub_;
  rclcpp::Subscription<sm::Safety>::SharedPtr safety_sub_;
  rclcpp::Subscription<Native>::SharedPtr native_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr attitude_sub_;
  rclcpp::Client<Command>::SharedPtr client_;
  rclcpp::Service<Trigger>::SharedPtr start_, abort_, reset_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {rclcpp::spin(std::make_shared<ThrowLaunchNode>());}
  catch (const std::exception & e) {RCLCPP_FATAL(rclcpp::get_logger("throw_launch"), "%s", e.what()); return 1;}
  rclcpp::shutdown();
  return 0;
}
