#include <gtest/gtest.h>
#include <safety_monitor/safety_policy.hpp>
#include <safety_monitor/safety_gate.hpp>

#include <limits>

using namespace safety_monitor;

static State healthy(bool flying = false)
{
  State s;
  s.status_fresh = s.local_position_fresh = s.battery_fresh = true;
  s.land_detected_fresh = s.failsafe_flags_fresh = s.gps_fresh = true;
  s.position_valid = s.gps_valid = s.pre_flight_checks_pass = s.safety_off = true;
  s.battery_remaining = 0.8F;
  s.flight_mode = "LOITER";
  s.armed = flying;
  s.landed = !flying;
  return s;
}

TEST(SafetyPolicy, HealthyGroundPermitsLaunchOnly)
{
  auto r = evaluate(healthy(), Policy{}, true, false);
  EXPECT_EQ(r.level, Status::READY);
  EXPECT_TRUE(r.launch_permitted);
  EXPECT_FALSE(r.autonomous_flight_permitted);
  EXPECT_FALSE(r.visual_landing_permitted);
}

TEST(SafetyPolicy, PreflightChecksBlockLaunchButNotEstablishedFlight)
{
  auto s = healthy();
  s.pre_flight_checks_pass = false;
  auto r = evaluate(s, Policy{}, true, false);
  EXPECT_EQ(r.level, Status::BLOCKED);
  EXPECT_FALSE(r.launch_permitted);
  s.armed = true;
  s.landed = false;
  r = evaluate(s, Policy{}, true, true);
  EXPECT_EQ(r.level, Status::READY);
  EXPECT_TRUE(r.autonomous_flight_permitted);
}

TEST(SafetyPolicy, FailsafeGroundBlockedFlightEmergency)
{
  auto s = healthy();
  s.failsafe = true;
  EXPECT_EQ(evaluate(s, Policy{}, true, false).level, Status::BLOCKED);
  EXPECT_EQ(evaluate(s, Policy{}, true, true).level, Status::EMERGENCY);
}

TEST(SafetyPolicy, StreamLossUsesLastFlightPhase)
{
  EXPECT_EQ(evaluate(healthy(), Policy{}, false, false).level, Status::BLOCKED);
  auto r = evaluate(healthy(true), Policy{}, false, true);
  EXPECT_EQ(r.level, Status::EMERGENCY);
  EXPECT_FALSE(r.telemetry_healthy);
  EXPECT_FALSE(r.autonomous_flight_permitted);
}

TEST(SafetyPolicy, EveryRequiredSourceMustBeFresh)
{
  for (auto field : {&State::status_fresh, &State::local_position_fresh,
    &State::battery_fresh, &State::land_detected_fresh, &State::failsafe_flags_fresh})
  {
    auto s = healthy();
    s.*field = false;
    auto r = evaluate(s, Policy{}, true, false);
    EXPECT_FALSE(r.telemetry_healthy);
    EXPECT_FALSE(r.launch_permitted);
  }
}

TEST(SafetyPolicy, InvalidAndNonfinitePositionBlockAutonomy)
{
  for (auto field : {&State::x, &State::y, &State::z, &State::vx, &State::vy, &State::vz}) {
    auto s = healthy(true);
    s.*field = std::numeric_limits<float>::quiet_NaN();
    auto r = evaluate(s, Policy{}, true, true);
    EXPECT_FALSE(r.position_healthy);
    EXPECT_EQ(r.level, Status::EMERGENCY);
    EXPECT_FALSE(r.autonomous_flight_permitted);
  }
  for (auto field : {&State::local_position_invalid, &State::local_velocity_invalid}) {
    auto s = healthy(true);
    s.*field = true;
    EXPECT_EQ(evaluate(s, Policy{}, true, true).level, Status::EMERGENCY);
  }
}

TEST(SafetyPolicy, CriticalBatteryCodesAndBothWarningSources)
{
  for (auto code : {State::BATTERY_WARNING_CRITICAL, State::BATTERY_WARNING_EMERGENCY,
    State::BATTERY_WARNING_FAILED, State::BATTERY_WARNING_UNHEALTHY})
  {
    for (auto field : {&State::battery_warning, &State::failsafe_battery_warning}) {
      auto s = healthy(true);
      s.*field = code;
      EXPECT_EQ(evaluate(s, Policy{}, true, true).level, Status::EMERGENCY);
      EXPECT_FALSE(evaluate(s, Policy{}, true, true).autonomous_flight_permitted);
    }
  }
}

TEST(SafetyPolicy, ChargingIsBlockedAndNotOrderedAsEmergency)
{
  auto s = healthy();
  s.battery_warning = State::BATTERY_WARNING_CHARGING;
  auto r = evaluate(s, Policy{}, true, false);
  EXPECT_EQ(r.level, Status::BLOCKED);
  EXPECT_FALSE(r.battery_healthy);
  s.battery_warning = 5;  // Reserved / unknown code.
  EXPECT_FALSE(evaluate(s, Policy{}, true, false).launch_permitted);
}

TEST(SafetyPolicy, BatteryReservesDependOnFlightPhase)
{
  auto s = healthy();
  s.battery_remaining = 0.25F;
  EXPECT_EQ(evaluate(s, Policy{}, true, false).level, Status::BLOCKED);
  s.armed = true;
  auto r = evaluate(s, Policy{}, true, true);
  EXPECT_EQ(r.level, Status::WARNING);
  EXPECT_TRUE(r.autonomous_flight_permitted);
  s.battery_remaining = 0.15F;
  EXPECT_EQ(evaluate(s, Policy{}, true, true).level, Status::BLOCKED);
  s.battery_remaining = 0.05F;
  EXPECT_EQ(evaluate(s, Policy{}, true, true).level, Status::EMERGENCY);
}

TEST(SafetyPolicy, BadBatteryMeasurementsFailClosed)
{
  for (float remaining : {-1.0F, 1.1F, std::numeric_limits<float>::quiet_NaN(),
    std::numeric_limits<float>::infinity()})
  {
    auto s = healthy(true);
    s.battery_remaining = remaining;
    auto r = evaluate(s, Policy{}, true, true);
    EXPECT_EQ(r.level, Status::EMERGENCY);
    EXPECT_FALSE(r.battery_healthy);
    EXPECT_FALSE(r.autonomous_flight_permitted);
  }
}

TEST(SafetyPolicy, ExactFloatBatteryThresholdsAreConsistent)
{
  auto s = healthy();
  s.battery_remaining = 0.30F;
  EXPECT_TRUE(evaluate(s, Policy{}, true, false).launch_permitted);
  s.armed = true;
  s.battery_remaining = 0.20F;
  EXPECT_TRUE(evaluate(s, Policy{}, true, true).autonomous_flight_permitted);
  s.battery_remaining = 0.10F;
  EXPECT_EQ(evaluate(s, Policy{}, true, true).level, Status::EMERGENCY);
}

TEST(SafetyPolicy, Px4BatteryHealthAndTimeWarningsRespected)
{
  for (auto field : {&State::battery_unhealthy, &State::battery_low_remaining_time}) {
    auto s = healthy(true);
    s.*field = true;
    EXPECT_EQ(evaluate(s, Policy{}, true, true).level, Status::EMERGENCY);
  }
}

TEST(SafetyPolicy, RequiredOperatorLinksFailClosed)
{
  for (auto field : {&State::manual_control_signal_lost, &State::gcs_connection_lost}) {
    auto s = healthy(true);
    s.*field = true;
    auto r = evaluate(s, Policy{}, true, true);
    EXPECT_FALSE(r.operator_link_healthy);
    EXPECT_EQ(r.level, Status::EMERGENCY);
  }
}

TEST(SafetyPolicy, OptionalLinksAreExplicitPolicyChoices)
{
  auto s = healthy();
  s.manual_control_signal_lost = s.gcs_connection_lost = true;
  Policy p;
  p.require_rc = p.require_gcs = false;
  EXPECT_TRUE(evaluate(s, p, true, false).launch_permitted);
}

TEST(SafetyPolicy, GnssLossBlocksOnlyWhenRequiredAndDoesNotDemandEmergencyLanding)
{
  auto s = healthy(true);
  s.gps_valid = false;
  EXPECT_EQ(evaluate(s, Policy{}, true, true).level, Status::BLOCKED);
  Policy p;
  p.require_gnss = false;
  EXPECT_TRUE(evaluate(s, p, true, true).autonomous_flight_permitted);
  s.gps_valid = true;
  s.global_position_invalid = true;
  EXPECT_FALSE(evaluate(s, Policy{}, true, true).gnss_healthy);
}

TEST(SafetyPolicy, ManualTakeoverBlocksAutonomyWithoutDeclaringEmergency)
{
  auto s = healthy(true);
  s.flight_mode = "POSITION";
  auto r = evaluate(s, Policy{}, true, true);
  EXPECT_EQ(r.level, Status::BLOCKED);
  EXPECT_FALSE(r.autonomous_flight_permitted);
  EXPECT_FALSE(r.launch_permitted);
}

TEST(SafetyPolicy, UnknownModeCannotAuthorizeAutonomy)
{
  auto s = healthy(true);
  s.flight_mode = "UNKNOWN";
  EXPECT_FALSE(evaluate(s, Policy{}, true, true).autonomous_flight_permitted);
}

TEST(SafetyPolicy, OffboardLossOnlyAppliesInOffboard)
{
  auto s = healthy(true);
  s.offboard_control_signal_lost = true;
  EXPECT_TRUE(evaluate(s, Policy{}, true, true).autonomous_flight_permitted);
  s.flight_mode = "OFFBOARD";
  EXPECT_EQ(evaluate(s, Policy{}, true, true).level, Status::EMERGENCY);
}

TEST(SafetyPolicy, RtlNeedsHomeAndLandingDoesNotNeedPreflight)
{
  auto s = healthy(true);
  s.home_position_invalid = true;
  s.flight_mode = "RTL";
  EXPECT_FALSE(evaluate(s, Policy{}, true, true).autonomous_flight_permitted);
  s.flight_mode = "LAND";
  s.pre_flight_checks_pass = false;
  auto r = evaluate(s, Policy{}, true, true);
  EXPECT_EQ(r.phase, Status::LANDING);
  EXPECT_TRUE(r.autonomous_flight_permitted);
}

TEST(SafetyPolicy, VisualLandingIsBlockedUntilTrackerAndEnvelopeExist)
{
  auto s = healthy(true);
  s.flight_mode = "PRECISION_LAND";
  auto r = evaluate(s, Policy{}, true, true);
  EXPECT_EQ(r.phase, Status::VISUAL_LANDING);
  EXPECT_EQ(r.level, Status::BLOCKED);
  EXPECT_FALSE(r.visual_landing_permitted);
  EXPECT_FALSE(r.autonomous_flight_permitted);
}

TEST(SafetyPolicy, SafetySwitchCalibrationAndAirborneStateBlockLaunch)
{
  auto s = healthy();
  s.safety_off = false;
  EXPECT_FALSE(evaluate(s, Policy{}, true, false).launch_permitted);
  s.safety_off = true;
  s.calibration_enabled = true;
  EXPECT_FALSE(evaluate(s, Policy{}, true, false).launch_permitted);
  s.calibration_enabled = false;
  s.landed = false;
  EXPECT_FALSE(evaluate(s, Policy{}, true, false).launch_permitted);
}

TEST(SafetyGate, MissingStaleAndBlockedMessagesDenyTransitions)
{
  SafetyGate gate;
  auto now = SafetyGate::Clock::now();
  const auto op = SafetyGate::Operation::Launch;
  EXPECT_FALSE(gate.permits(op));
  auto s = evaluate(healthy(), Policy{}, true, false);
  gate.receive(s, now);
  EXPECT_TRUE(gate.permits(op, false, now));
  EXPECT_FALSE(gate.permits(op, false, now + std::chrono::milliseconds(501)));
  for (auto level : {Status::UNKNOWN, Status::BLOCKED, Status::EMERGENCY, uint8_t(255)}) {
    s.level = level;
    gate.receive(s, now);
    EXPECT_FALSE(gate.permits(op, true, now));
  }
}

TEST(SafetyGate, WarningRequiresExplicitPhasePermissionAndConsent)
{
  SafetyGate gate;
  auto s = healthy(true);
  s.battery_remaining = 0.25F;
  auto r = evaluate(s, Policy{}, true, true);
  gate.receive(r);
  EXPECT_FALSE(gate.permits(SafetyGate::Operation::AutonomousFlight));
  EXPECT_TRUE(gate.permits(SafetyGate::Operation::AutonomousFlight, true));
  EXPECT_FALSE(gate.permits(SafetyGate::Operation::Launch, true));
  EXPECT_FALSE(gate.permits(SafetyGate::Operation::VisualLanding, true));
}
