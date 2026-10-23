#include <gtest/gtest.h>
#include <state_machine_manager/mission_machine.hpp>

#include <limits>

namespace sm = state_machine_manager;
using M = sm::Mission;
using F = sm::Feedback;
using S = sm::Safety;

static sm::Config future_config()
{
  sm::Config config;
  config.enable_stage_feedback = true;
  return config;
}

class MissionTest : public ::testing::Test
{
protected:
  sm::Config config = future_config();
  sm::Time now{sm::Milliseconds(10000)};
  sm::MissionMachine machine{config, now, 42};
  sm::Drone drone;
  S safety;
  uint64_t observation = 0;

  void SetUp() override {ground();}
  void sync() {machine.receive_drone(drone, now); machine.receive_safety(safety, now);}
  M state() {return machine.snapshot(now);}
  void ground()
  {
    drone = sm::Drone{};
    drone.status_fresh = drone.land_detected_fresh = drone.failsafe_flags_fresh = true;
    drone.local_position_fresh = drone.position_valid = drone.battery_fresh = true;
    drone.ready_to_arm = drone.pre_flight_checks_pass = drone.safety_off = drone.landed = true;
    drone.flight_mode = "LOITER";
    drone.battery_remaining = 0.8F;
    safety = S{};
    safety.level = S::READY;
    safety.phase = S::GROUNDED;
    safety.telemetry_healthy = safety.preflight_passed = safety.battery_healthy = true;
    safety.position_healthy = safety.operator_link_healthy = safety.gnss_healthy = true;
    safety.landed = safety.launch_permitted = true;
    sync();
  }
  F feedback(uint8_t event)
  {
    F f;
    f.mission_id = state().mission_id;
    f.transition_sequence = state().transition_sequence;
    f.observation_sequence = ++observation;
    f.event = event;
    f.verified = true;
    return f;
  }
  void event(uint8_t value)
  {
    ASSERT_TRUE(machine.receive_feedback(feedback(value), now).accepted);
    machine.tick(now);
  }
  void arm()
  {
    ASSERT_TRUE(machine.start(now).accepted);
    drone.armed = true;
    drone.ready_to_arm = false;
    safety.phase = S::FLIGHT;
    safety.launch_permitted = false;
    safety.autonomous_flight_permitted = true;
    sync();
    machine.tick(now);
    ASSERT_EQ(state().current_state, M::ARMED_WAITING);
  }
  void launch()
  {
    arm();
    drone.landed = safety.landed = false;
    sync();
    event(F::THROW_DETECTED);
    ASSERT_EQ(state().current_state, M::THROW_LAUNCH);
  }
  void search()
  {
    launch();
    event(F::LAUNCH_COMPLETE);
    ASSERT_EQ(state().current_state, M::STABILIZE);
    event(F::STABILIZED);
    ASSERT_EQ(state().current_state, M::SEARCH_TAG);
  }
  void track()
  {
    search();
    event(F::TAG_ACQUIRED);
    ASSERT_EQ(state().current_state, M::TRACK_TAG);
  }
  void land_stage()
  {
    track();
    drone.flight_mode = "PRECISION_LAND";
    safety.phase = S::VISUAL_LANDING;
    safety.autonomous_flight_permitted = false;
    safety.visual_landing_permitted = true;  // Future monitor contract, not today's monitor.
    sync();
    event(F::LANDING_READY);
    ASSERT_EQ(state().current_state, M::PRECISION_LAND);
  }
};

TEST_F(MissionTest, StartAndPublishedTransitionMetadata)
{
  EXPECT_EQ(state().current_state, M::STANDBY);
  EXPECT_FALSE(state().mission_active);
  ASSERT_TRUE(machine.start(now).accepted);
  EXPECT_EQ(state().current_state, M::PREFLIGHT);
  EXPECT_EQ(state().previous_state, M::STANDBY);
  EXPECT_EQ(state().mission_id, 42U);
  EXPECT_EQ(state().transition_sequence, 1U);
  EXPECT_TRUE(state().mission_active);
  EXPECT_EQ(state().entry_action, "AWAIT_EXTERNAL_ARMING_VERIFICATION");
  EXPECT_FALSE(machine.start(now).accepted);
  EXPECT_EQ(state().transition_sequence, 1U);
}

TEST(MissionWithoutInput, StartRequiresBothTopics)
{
  sm::MissionMachine machine;
  EXPECT_FALSE(machine.start(sm::Clock::now()).accepted);
  EXPECT_EQ(machine.snapshot(sm::Clock::now()).current_state, M::STANDBY);
}

TEST_F(MissionTest, SafetyCannotBeBypassedByHealthyDrone)
{
  for (auto level : {S::UNKNOWN, S::WARNING, S::BLOCKED, S::EMERGENCY, uint8_t(255)}) {
    safety.level = level;
    sync();
    EXPECT_FALSE(machine.start(now).accepted);
  }
  safety.level = S::READY;
  safety.launch_permitted = false;
  sync();
  EXPECT_FALSE(machine.start(now).accepted);
}

TEST_F(MissionTest, RawPx4ConditionsCannotBeBypassedBySafetyTrue)
{
  for (auto member : {&sm::Drone::ready_to_arm, &sm::Drone::pre_flight_checks_pass,
    &sm::Drone::safety_off, &sm::Drone::position_valid, &sm::Drone::status_fresh,
    &sm::Drone::land_detected_fresh, &sm::Drone::failsafe_flags_fresh})
  {
    ground();
    drone.*member = false;
    sync();
    EXPECT_FALSE(machine.start(now).accepted);
  }
  for (auto member : {&sm::Drone::armed, &sm::Drone::failsafe, &sm::Drone::critical_failure,
    &sm::Drone::calibration_enabled, &sm::Drone::command_pending})
  {
    ground();
    drone.*member = true;
    sync();
    EXPECT_FALSE(machine.start(now).accepted);
  }
  ground();
  drone.vx = std::numeric_limits<float>::quiet_NaN();
  sync();
  EXPECT_FALSE(machine.start(now).accepted);
}

TEST_F(MissionTest, MissingSafetyHeartbeatRemainsUnsafeWithCachedHealthyFlags)
{
  ASSERT_TRUE(machine.start(now).accepted);
  now += sm::Milliseconds(501);
  machine.receive_drone(drone, now);
  machine.tick(now);
  EXPECT_EQ(state().current_state, M::ABORTED);
  EXPECT_FALSE(state().safety_fresh);
  EXPECT_FALSE(machine.reset(now).accepted);
}

TEST_F(MissionTest, MissingDroneHeartbeatCannotBeHiddenByFreshSafety)
{
  arm();
  now += sm::Milliseconds(501);
  machine.receive_safety(safety, now);
  machine.tick(now);
  EXPECT_EQ(state().current_state, M::RECOVERY);
  EXPECT_FALSE(state().telemetry_fresh);
}

TEST_F(MissionTest, ExternalArmingNeedsMatchingSafetyPhase)
{
  ASSERT_TRUE(machine.start(now).accepted);
  drone.armed = true;
  drone.ready_to_arm = false;
  sync();
  machine.tick(now);
  EXPECT_EQ(state().current_state, M::PREFLIGHT);
  now += sm::Milliseconds(200);
  safety.phase = S::FLIGHT;
  safety.launch_permitted = false;
  safety.autonomous_flight_permitted = true;
  sync();
  machine.tick(now);
  EXPECT_EQ(state().current_state, M::ARMED_WAITING);
}

TEST_F(MissionTest, ArmingSafetyHandoffIsBounded)
{
  ASSERT_TRUE(machine.start(now).accepted);
  drone.armed = true;
  sync();
  machine.tick(now);
  now += config.arming_handoff_timeout;
  sync();
  machine.tick(now);
  EXPECT_EQ(state().current_state, M::RECOVERY);
}

TEST_F(MissionTest, VerifiedFutureStagePath)
{
  land_stage();
  EXPECT_EQ(state().previous_state, M::TRACK_TAG);
  EXPECT_TRUE(state().mission_active);
  ground();
  machine.tick(now);
  EXPECT_EQ(state().current_state, M::STANDBY);
  EXPECT_EQ(state().mission_result, M::RESULT_SUCCEEDED);
  EXPECT_FALSE(state().mission_active);
}

TEST_F(MissionTest, ArmingAndAirborneAloneNeverProveThrow)
{
  arm();
  drone.landed = safety.landed = false;
  sync();
  machine.tick(now);
  EXPECT_EQ(state().current_state, M::ARMED_WAITING);
  now += config.arming_handoff_timeout;
  sync();
  machine.tick(now);
  EXPECT_EQ(state().current_state, M::RECOVERY);
}

TEST_F(MissionTest, ThrowFeedbackCannotAdvanceWhileLanded)
{
  arm();
  event(F::THROW_DETECTED);
  EXPECT_EQ(state().current_state, M::ARMED_WAITING);
  now += config.feedback_timeout;
  drone.landed = false;
  sync();
  machine.tick(now);
  EXPECT_EQ(state().current_state, M::ARMED_WAITING);
}

TEST(MissionDefaults, FutureFeedbackDisabled)
{
  sm::MissionMachine machine;
  F event;
  event.verified = true;
  EXPECT_FALSE(machine.receive_feedback(event, sm::Clock::now()).accepted);
  EXPECT_FALSE(machine.snapshot(sm::Clock::now()).stage_feedback_enabled);
}

TEST_F(MissionTest, WrongMissionStateUnverifiedAndReplayRejected)
{
  arm();
  auto valid = feedback(F::THROW_DETECTED);
  auto wrong = valid;
  wrong.mission_id++;
  EXPECT_FALSE(machine.receive_feedback(wrong, now).accepted);
  wrong = valid;
  wrong.transition_sequence--;
  EXPECT_FALSE(machine.receive_feedback(wrong, now).accepted);
  wrong = valid;
  wrong.verified = false;
  EXPECT_FALSE(machine.receive_feedback(wrong, now).accepted);
  ASSERT_TRUE(machine.receive_feedback(valid, now).accepted);
  EXPECT_FALSE(machine.receive_feedback(valid, now).accepted);
  drone.landed = safety.landed = false;
  sync();
  machine.tick(now);
  EXPECT_EQ(state().current_state, M::THROW_LAUNCH);
  EXPECT_FALSE(machine.receive_feedback(valid, now).accepted);
}

TEST_F(MissionTest, InvalidEventDoesNotSkipStates)
{
  launch();
  const auto sequence = state().transition_sequence;
  event(F::TAG_ACQUIRED);
  EXPECT_EQ(state().current_state, M::THROW_LAUNCH);
  EXPECT_EQ(state().transition_sequence, sequence);
  EXPECT_GT(state().rejected_requests, 0U);
}

TEST_F(MissionTest, StageFailureWinsOverPositiveObservation)
{
  launch();
  ASSERT_TRUE(machine.receive_feedback(feedback(F::LAUNCH_COMPLETE), now).accepted);
  ASSERT_TRUE(machine.receive_feedback(feedback(F::STAGE_FAILED), now).accepted);
  machine.tick(now);
  EXPECT_EQ(state().current_state, M::RECOVERY);
}

TEST_F(MissionTest, GroundStageFailureAborts)
{
  ASSERT_TRUE(machine.start(now).accepted);
  event(F::STAGE_FAILED);
  EXPECT_EQ(state().current_state, M::ABORTED);
  EXPECT_EQ(state().mission_result, M::RESULT_FAILED);
}

TEST_F(MissionTest, StageFailureCannotExpireIntoPositiveProgress)
{
  launch();
  ASSERT_TRUE(machine.receive_feedback(feedback(F::STAGE_FAILED), now).accepted);
  now += config.feedback_timeout + sm::Milliseconds(1);
  sync();
  event(F::LAUNCH_COMPLETE);
  EXPECT_EQ(state().current_state, M::RECOVERY);
}

TEST_F(MissionTest, FailsafeTakesPriorityOverPositiveStageFeedback)
{
  launch();
  drone.failsafe = true;
  sync();
  event(F::LAUNCH_COMPLETE);
  EXPECT_EQ(state().current_state, M::RECOVERY);
}

TEST_F(MissionTest, OperatorTakeoverAndNativeRecoveryModeStopNormalMission)
{
  launch();
  drone.flight_mode = "POSITION";
  sync();
  machine.tick(now);
  EXPECT_EQ(state().current_state, M::RECOVERY);
}

TEST_F(MissionTest, TagLossReturnsToSearchUnderSafeConditions)
{
  track();
  event(F::TAG_LOST);
  EXPECT_EQ(state().current_state, M::SEARCH_TAG);
}

TEST_F(MissionTest, TagHeartbeatExpiryReturnsToSearch)
{
  track();
  now += config.tag_loss_timeout;
  sync();
  machine.tick(now);
  EXPECT_EQ(state().current_state, M::SEARCH_TAG);
}

TEST_F(MissionTest, TagLossCannotBeHiddenByNewHeartbeatAfterExecutorDelay)
{
  track();
  ASSERT_TRUE(machine.receive_feedback(feedback(F::TAG_LOST), now).accepted);
  now += config.feedback_timeout + sm::Milliseconds(1);
  sync();
  event(F::TAG_TRACKED);
  EXPECT_EQ(state().current_state, M::SEARCH_TAG);
}

TEST_F(MissionTest, QueuedTimelyTagHeartbeatPreventsFalseExpiry)
{
  track();
  now += sm::Milliseconds(900);
  sync();
  ASSERT_TRUE(machine.receive_feedback(feedback(F::TAG_TRACKED), now).accepted);
  now += sm::Milliseconds(100);
  sync();
  machine.tick(now);
  EXPECT_EQ(state().current_state, M::TRACK_TAG);
}

TEST_F(MissionTest, CurrentMonitorCannotAuthorizeVisualLanding)
{
  track();
  event(F::LANDING_READY);
  EXPECT_EQ(state().current_state, M::TRACK_TAG);
  EXPECT_FALSE(safety.visual_landing_permitted);
  EXPECT_NE(state().last_rejection.find("Precision landing blocked"), std::string::npos);
}

TEST_F(MissionTest, VisualPermissionAloneCannotReplaceActivePx4Mode)
{
  track();
  safety.visual_landing_permitted = true;
  sync();
  event(F::LANDING_READY);
  EXPECT_EQ(state().current_state, M::TRACK_TAG);
}

TEST_F(MissionTest, PreflightTimeoutAbortsWithoutFlightActions)
{
  ASSERT_TRUE(machine.start(now).accepted);
  now += config.preflight_timeout;
  sync();
  machine.tick(now);
  EXPECT_EQ(state().current_state, M::ABORTED);
}

TEST_F(MissionTest, FlightStageTimeoutAndRecoveryTimeout)
{
  launch();
  now += config.throw_timeout;
  sync();
  machine.tick(now);
  ASSERT_EQ(state().current_state, M::RECOVERY);
  now += config.recovery_timeout;
  sync();
  machine.tick(now);
  EXPECT_EQ(state().current_state, M::FAULT);
  EXPECT_FALSE(state().mission_active);
  EXPECT_FALSE(machine.reset(now).accepted);
  ground();
  ASSERT_TRUE(machine.reset(now).accepted);
  EXPECT_EQ(state().current_state, M::STANDBY);
}

TEST_F(MissionTest, EachTimedFlightStageStopsOnDeadline)
{
  // Construct independent machines so every stage's own deadline is exercised.
  for (int stage = 0; stage < 6; ++stage) {
    sm::Time t{sm::Milliseconds(10000)};
    sm::Config c = future_config();
    c.mission_timeout = sm::Milliseconds(1000000);
    sm::MissionMachine core(c, t);
    ground();
    core.receive_drone(drone, t); core.receive_safety(safety, t);
    ASSERT_TRUE(core.start(t).accepted);
    drone.armed = true;
    safety.phase = S::FLIGHT; safety.autonomous_flight_permitted = true;
    core.receive_drone(drone, t); core.receive_safety(safety, t); core.tick(t);
    auto send = [&](uint8_t event) {
        auto status = core.snapshot(t);
        F f; f.mission_id = status.mission_id; f.transition_sequence = status.transition_sequence;
        f.observation_sequence = 1; f.verified = true; f.event = event;
        ASSERT_TRUE(core.receive_feedback(f, t).accepted); core.tick(t);
      };
    sm::Milliseconds wait = c.armed_waiting_timeout;
    if (stage >= 1) {
      drone.landed = false;
      core.receive_drone(drone, t); send(F::THROW_DETECTED); wait = c.throw_timeout;
    }
    if (stage >= 2) {send(F::LAUNCH_COMPLETE); wait = c.stabilize_timeout;}
    if (stage >= 3) {send(F::STABILIZED); wait = c.search_timeout;}
    if (stage >= 4) {send(F::TAG_ACQUIRED); wait = c.tracking_timeout;}
    if (stage >= 5) {
      drone.flight_mode = "PRECISION_LAND";
      safety.phase = S::VISUAL_LANDING; safety.visual_landing_permitted = true;
      core.receive_drone(drone, t); core.receive_safety(safety, t);
      send(F::LANDING_READY); wait = c.landing_timeout;
    }
    t += wait;
    core.receive_drone(drone, t); core.receive_safety(safety, t); core.tick(t);
    EXPECT_EQ(core.snapshot(t).current_state, M::RECOVERY) << stage;
  }
}

TEST_F(MissionTest, DeadlineWinsOverLateSuccess)
{
  launch();
  now += config.throw_timeout;
  sync();
  event(F::LAUNCH_COMPLETE);
  EXPECT_EQ(state().current_state, M::RECOVERY);
}

TEST_F(MissionTest, OverallMissionDeadlineBoundsRepeatedTagReacquisition)
{
  track();
  for (int iteration = 0; iteration < 449; ++iteration) {
    now += sm::Milliseconds(400);
    sync();
    event(F::TAG_LOST);
    ASSERT_EQ(state().current_state, M::SEARCH_TAG);
    event(F::TAG_ACQUIRED);
    ASSERT_EQ(state().current_state, M::TRACK_TAG);
  }
  now += sm::Milliseconds(400);
  sync();
  machine.tick(now);
  EXPECT_EQ(state().current_state, M::RECOVERY);
  EXPECT_EQ(state().transition_reason, "Overall mission timeout");
}

TEST_F(MissionTest, AbortAndResetRequireVerifiedGroundAndSafety)
{
  arm();
  ASSERT_TRUE(machine.abort(now).accepted);
  EXPECT_EQ(state().current_state, M::RECOVERY);
  EXPECT_TRUE(state().abort_requested);
  EXPECT_FALSE(machine.reset(now).accepted);
  ground();
  safety.level = S::BLOCKED;
  safety.launch_permitted = false;
  sync();
  machine.tick(now);
  ASSERT_EQ(state().current_state, M::ABORTED);
  EXPECT_EQ(state().mission_result, M::RESULT_ABORTED);
  EXPECT_FALSE(machine.reset(now).accepted);
  ground();
  ASSERT_TRUE(machine.reset(now).accepted);
  ASSERT_TRUE(machine.start(now).accepted);
  EXPECT_EQ(state().mission_id, 43U);
}

TEST_F(MissionTest, PendingArmCommandIsNotSafeGroundForAbortCompletion)
{
  ASSERT_TRUE(machine.start(now).accepted);
  drone.command_pending = true;
  sync();
  ASSERT_TRUE(machine.abort(now).accepted);
  EXPECT_EQ(state().current_state, M::RECOVERY);
}

TEST_F(MissionTest, InvalidOperatorRequestsDoNotChangeState)
{
  EXPECT_FALSE(machine.abort(now).accepted);
  EXPECT_FALSE(machine.reset(now).accepted);
  ASSERT_TRUE(machine.start(now).accepted);
  EXPECT_FALSE(machine.reset(now).accepted);
  ASSERT_TRUE(machine.abort(now).accepted);
  EXPECT_EQ(state().current_state, M::ABORTED);
  EXPECT_FALSE(machine.start(now).accepted);
}

TEST(MissionConfiguration, NonpositiveTimeoutRejected)
{
  sm::Config config;
  config.search_timeout = sm::Milliseconds(0);
  EXPECT_THROW(sm::MissionMachine{config}, std::invalid_argument);
}
