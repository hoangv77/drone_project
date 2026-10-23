#pragma once

#include <px4_interface_node/msg/drone_state.hpp>
#include <safety_monitor/msg/safety_status.hpp>

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

namespace safety_monitor
{
using State = px4_interface_node::msg::DroneState;
using Status = msg::SafetyStatus;

struct Policy
{
  double battery_min_launch{0.30};
  double battery_min_flight{0.20};
  double battery_emergency{0.10};
  bool require_gnss{true};
  bool require_gcs{true};
  bool require_rc{true};
  std::vector<std::string> launch_modes{"LOITER", "POSITION"};
  std::vector<std::string> autonomous_modes{
    "LOITER", "MISSION", "OFFBOARD", "RTL", "TAKEOFF", "LAND"};
};

inline bool contains(const std::vector<std::string> & values, const std::string & value)
{
  return std::find(values.begin(), values.end(), value) != values.end();
}

inline bool critical_battery_warning(uint8_t warning)
{
  return warning == State::BATTERY_WARNING_CRITICAL ||
         warning == State::BATTERY_WARNING_EMERGENCY ||
         warning == State::BATTERY_WARNING_FAILED ||
         warning == State::BATTERY_WARNING_UNHEALTHY;
}

inline bool usable_battery_warning(uint8_t warning)
{
  return warning == State::BATTERY_WARNING_NONE || warning == State::BATTERY_WARNING_LOW;
}

// Pure policy evaluation. stream_fresh and flight_active are supplied by a
// monotonic-clock receiver that remembers an armed/airborne vehicle across loss.
inline Status evaluate(const State & s, const Policy & p, bool stream_fresh, bool flight_active)
{
  Status out;
  out.level = Status::READY;
  out.phase = flight_active ? Status::FLIGHT : Status::GROUNDED;
  if (flight_active && s.flight_mode == "LAND") {out.phase = Status::LANDING;}
  if (flight_active && s.flight_mode == "PRECISION_LAND") {out.phase = Status::VISUAL_LANDING;}
  const auto fault_level = flight_active ? Status::EMERGENCY : Status::BLOCKED;
  std::string common;
  const auto append = [](std::string & text, const std::string & reason) {
      if (!text.empty()) {text += "; ";}
      text += reason;
    };
  const auto issue = [&](uint8_t level, const std::string & reason, bool blocks = true) {
      out.level = std::max(out.level, level);
      append(out.reason, reason);
      if (blocks) {append(common, reason);}
    };
  const bool flags_fresh = stream_fresh && s.failsafe_flags_fresh;
  const bool status_fresh = stream_fresh && s.status_fresh;
  if (!flight_active && !(status_fresh && s.land_detected_fresh && s.landed && !s.armed)) {
    out.phase = Status::PHASE_UNKNOWN;
  }
  out.telemetry_healthy = status_fresh && flags_fresh && s.local_position_fresh &&
    s.battery_fresh && s.land_detected_fresh;
  out.preflight_passed = status_fresh && s.pre_flight_checks_pass;
  out.px4_failsafe_active = s.failsafe;
  out.landed = status_fresh && s.land_detected_fresh && s.landed;
  out.position_healthy = stream_fresh && s.local_position_fresh && s.position_valid &&
    std::isfinite(s.x) && std::isfinite(s.y) && std::isfinite(s.z) &&
    std::isfinite(s.vx) && std::isfinite(s.vy) && std::isfinite(s.vz) &&
    flags_fresh && !s.local_position_invalid && !s.local_velocity_invalid;
  out.gnss_healthy = stream_fresh && s.gps_fresh && s.gps_valid &&
    flags_fresh && !s.global_position_invalid;
  out.operator_link_healthy = status_fresh && flags_fresh &&
    (!p.require_gcs || !s.gcs_connection_lost) &&
    (!p.require_rc || !s.manual_control_signal_lost);
  const bool battery_valid = stream_fresh && s.battery_fresh &&
    std::isfinite(s.battery_remaining) && s.battery_remaining >= 0.0F &&
    s.battery_remaining <= 1.0F;
  const bool battery_critical = critical_battery_warning(s.battery_warning) ||
    critical_battery_warning(s.failsafe_battery_warning) ||
    s.battery_unhealthy || s.battery_low_remaining_time;
  const bool battery_state_ok = usable_battery_warning(s.battery_warning) &&
    usable_battery_warning(s.failsafe_battery_warning);
  out.battery_healthy = battery_valid && flags_fresh && !battery_critical &&
    battery_state_ok && s.battery_remaining >= static_cast<float>(p.battery_min_flight);

  if (!stream_fresh) {
    issue(fault_level, "Drone state stream missing/stale");
  } else {
    if (!s.status_fresh) {issue(fault_level, "PX4 vehicle status missing/stale");}
    if (!s.local_position_fresh) {issue(fault_level, "PX4 local position missing/stale");}
    if (!s.battery_fresh) {issue(fault_level, "PX4 battery telemetry missing/stale");}
    if (!s.land_detected_fresh) {issue(fault_level, "PX4 land detection missing/stale");}
    if (!s.failsafe_flags_fresh) {issue(fault_level, "PX4 failsafe flags missing/stale");}
  }
  if (status_fresh && s.failsafe) {issue(fault_level, "PX4 failsafe active; defer to PX4 recovery");}
  if (flags_fresh && s.critical_failure) {issue(fault_level, "PX4 critical flight-control failure");}
  if (flags_fresh && s.geofence_breached) {issue(fault_level, "PX4 geofence breached");}
  if (flags_fresh && s.mission_failure) {issue(Status::BLOCKED, "PX4 mission failure");}
  if (!out.position_healthy) {issue(fault_level, "Local position or velocity estimate unavailable/invalid");}
  if (!out.operator_link_healthy) {issue(fault_level, "Required operator link lost or unavailable");}
  if (p.require_gnss && !out.gnss_healthy) {issue(Status::BLOCKED, "Required GNSS unavailable/invalid");}
  if (!battery_valid) {
    issue(fault_level, "Battery measurement unavailable/invalid");
  } else if ((flags_fresh && battery_critical) ||
    s.battery_remaining <= static_cast<float>(p.battery_emergency))
  {
    issue(fault_level, "Critical battery condition; defer to PX4 recovery");
  } else if (!out.battery_healthy) {
    issue(Status::BLOCKED, "Battery below flight minimum, charging, or warning state unknown");
  } else if (s.battery_remaining < static_cast<float>(p.battery_min_launch) ||
    s.battery_warning == State::BATTERY_WARNING_LOW ||
    s.failsafe_battery_warning == State::BATTERY_WARNING_LOW)
  {
    issue(Status::WARNING, "Low battery reserve; launch blocked", false);
  }
  if (flags_fresh && s.flight_mode == "OFFBOARD" && s.offboard_control_signal_lost) {
    issue(fault_level, "PX4 offboard control signal lost");
  }
  if (flags_fresh && s.flight_mode == "RTL" && s.home_position_invalid) {
    issue(Status::BLOCKED, "RTL requires a valid home position");
  }

  out.launch_block_reason = common;
  out.autonomous_flight_block_reason = common;
  const auto launch_block = [&](const std::string & reason) {
      append(out.launch_block_reason, reason);
      if (!flight_active) {
        out.level = std::max(out.level, Status::BLOCKED);
        append(out.reason, reason);
      }
    };
  if (!out.preflight_passed) {launch_block("PX4 preflight checks have not passed");}
  if (!s.safety_off || s.calibration_enabled) {launch_block("Safety enabled or calibration active");}
  if (!out.landed || s.armed || flight_active) {launch_block("Launch requires disarmed, confirmed landed state");}
  if (!contains(p.launch_modes, s.flight_mode)) {launch_block("Current mode does not permit launch");}
  if (!out.battery_healthy || s.battery_remaining < static_cast<float>(p.battery_min_launch) ||
    s.battery_warning != State::BATTERY_WARNING_NONE ||
    s.failsafe_battery_warning != State::BATTERY_WARNING_NONE)
  {
    launch_block("Battery does not meet launch reserve requirements");
  }
  if (!status_fresh || !s.armed) {
    append(out.autonomous_flight_block_reason, "Autonomous flight requires confirmed armed state");
  }
  if (!contains(p.autonomous_modes, s.flight_mode)) {
    append(out.autonomous_flight_block_reason, "Current mode does not permit autonomous flight");
    if (flight_active) {
      out.level = std::max(out.level, Status::BLOCKED);
      append(out.reason, "Current mode does not permit autonomous flight (operator control)");
    }
  }
  out.launch_permitted = out.launch_block_reason.empty();
  out.autonomous_flight_permitted = out.autonomous_flight_block_reason.empty();
  out.visual_landing_permitted = false;
  out.visual_landing_block_reason = "AprilTag tracking and landing envelope checks not implemented";
  if (out.phase == Status::VISUAL_LANDING) {
    out.level = std::max(out.level, Status::BLOCKED);
    append(out.reason, out.visual_landing_block_reason);
    out.autonomous_flight_permitted = false;
  }
  if (out.reason.empty()) {out.reason = "All required checks for the current phase passed";}
  return out;
}
}  // namespace safety_monitor
