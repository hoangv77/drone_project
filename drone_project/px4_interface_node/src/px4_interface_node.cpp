// ROS 2 C++ API: nodes, publishers, subscriptions, timers, and logging.
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>

// Our clean internal state message generated from msg/DroneState.msg.
#include <px4_interface_node/msg/drone_state.hpp>
#include <px4_interface_node/srv/set_flight_mode.hpp>
#include <px4_interface_node/srv/execute_command.hpp>

// PX4 telemetry and command messages.
#include <px4_msgs/msg/battery_status.hpp>
#include <px4_msgs/msg/failsafe_flags.hpp>
#if __has_include(<px4_msgs/msg/sensor_gps.hpp>)
#include <px4_msgs/msg/sensor_gps.hpp>
#else
#include <px4_msgs/msg/sensor_gnss.hpp>
#include <px4_msgs/msg/vehicle_gnss.hpp>
#endif
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_command_ack.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

class Px4InterfaceNode : public rclcpp::Node
{
public:
    Px4InterfaceNode()
        : Node("px4_interface_node")
    {
        // These settings define the command endpoint and cannot change during
        // a request. Source IDs must be reserved for this node on the vehicle.
        rcl_interfaces::msg::ParameterDescriptor startup_only;
        startup_only.read_only = true;
        const auto id_parameter = [this, &startup_only](const char * name, int value) {
            const auto id = declare_parameter<int>(name, value, startup_only);
            if (id < 1 || id > 255) {
                throw std::invalid_argument(std::string(name) + " must be in [1, 255]");
            }
            return static_cast<uint8_t>(id);
        };
        target_system_ = id_parameter("target_system", 1);
        target_component_ = id_parameter("target_component", 1);
        source_system_ = id_parameter("source_system", 1);
        source_component_ = id_parameter("source_component", 191);
        const auto timeout_parameter = [this, &startup_only](const char * name, int value) {
            const auto timeout = declare_parameter<int>(name, value, startup_only);
            if (timeout < 100 || timeout > 60000) {
                throw std::invalid_argument(std::string(name) + " must be in [100, 60000] ms");
            }
            return std::chrono::milliseconds(timeout);
        };
        ack_timeout_ = timeout_parameter("ack_timeout_ms", 2000);
        const auto legacy_timeout = timeout_parameter("mode_timeout_ms", 5000);
        transition_timeout_ = timeout_parameter(
            "transition_timeout_ms", static_cast<int>(legacy_timeout.count()));

        land_subscription_ = create_subscription<px4_msgs::msg::VehicleLandDetected>(
            "/fmu/out/vehicle_land_detected", rclcpp::SensorDataQoS(),
            [this](px4_msgs::msg::VehicleLandDetected::SharedPtr msg) {
                landed_ = msg->landed;
                last_land_rx_ = std::chrono::steady_clock::now();
                has_land_ = true;
            });

        // PX4 telemetry uses best-effort sensor-data QoS. A subscriber with
        // incompatible reliable-only QoS might never receive these messages.
        const auto px4_qos = rclcpp::SensorDataQoS();

        status_subscription_ =
            create_subscription<px4_msgs::msg::VehicleStatus>(
                "/fmu/out/vehicle_status_v4",
                px4_qos,
                std::bind(
                    &Px4InterfaceNode::status_callback,
                    this,
                    std::placeholders::_1));

        local_position_subscription_ =
            create_subscription<px4_msgs::msg::VehicleLocalPosition>(
                "/fmu/out/vehicle_local_position_v1",
                px4_qos,
                std::bind(
                    &Px4InterfaceNode::local_position_callback,
                    this,
                    std::placeholders::_1));

        odometry_subscription_ =
            create_subscription<px4_msgs::msg::VehicleOdometry>(
                "/fmu/out/vehicle_odometry",
                px4_qos,
                std::bind(
                    &Px4InterfaceNode::odometry_callback,
                    this,
                    std::placeholders::_1));

        gps_subscription_ =
            create_subscription<GpsMessage>(
                gps_topic(),
                px4_qos,
                std::bind(
                    &Px4InterfaceNode::gps_callback,
                    this,
                    std::placeholders::_1));

        battery_subscription_ =
            create_subscription<px4_msgs::msg::BatteryStatus>(
                "/fmu/out/battery_status_v1",
                px4_qos,
                std::bind(
                    &Px4InterfaceNode::battery_callback,
                    this,
                    std::placeholders::_1));

        failsafe_subscription_ = create_subscription<px4_msgs::msg::FailsafeFlags>(
            "/fmu/out/failsafe_flags", px4_qos,
            [this](px4_msgs::msg::FailsafeFlags::SharedPtr msg) {
                failsafe_flags_ = *msg;
                has_failsafe_flags_ = true;
                last_failsafe_flags_rx_ = std::chrono::steady_clock::now();
            });

        // This is the clean topic consumed by nodes we control. A depth of 10
        // means DDS may queue up to ten outgoing samples for a slow subscriber.
        state_publisher_ =
            create_publisher<px4_interface_node::msg::DroneState>(
                "/drone/state",
                rclcpp::QoS(10).reliable());

        command_publisher_ = create_publisher<px4_msgs::msg::VehicleCommand>(
            "/fmu/in/vehicle_command", rclcpp::QoS(10).reliable());
        command_ack_subscription_ = create_subscription<px4_msgs::msg::VehicleCommandAck>(
            "/fmu/out/vehicle_command_ack_v1", px4_qos,
            std::bind(&Px4InterfaceNode::command_ack_callback, this, std::placeholders::_1));

        // Defer the response, leaving the executor free to receive ACKs and
        // telemetry. All callbacks use the default mutually exclusive group.
        mode_service_ = create_service<SetFlightMode>(
            "/drone/set_flight_mode",
            [this](std::shared_ptr<rmw_request_id_t> header,
                   SetFlightMode::Request::SharedPtr request) {
                request_command(ExecuteCommand::Request::SET_MODE, request->mode,
                    [this, header](const CommandResponse & result) {
                        SetFlightMode::Response response;
                        response.success = result.success;
                        response.accepted = result.accepted;
                        response.result = result.result;
                        response.ack_result = result.ack_result;
                        response.current_mode = result.current_mode;
                        response.status_fresh = result.status_fresh;
                        response.message = result.message;
                        mode_service_->send_response(*header, response);
                    });
            });
        command_service_ = create_service<ExecuteCommand>(
            "/drone/execute_command",
            [this](std::shared_ptr<rmw_request_id_t> header,
                   ExecuteCommand::Request::SharedPtr request) {
                request_command(request->command, request->mode,
                    [this, header](const CommandResponse & result) {
                        auto response = result;
                        command_service_->send_response(*header, response);
                    });
            });
        command_timer_ = create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&Px4InterfaceNode::check_command, this));

        // Publish one assembled snapshot every 100 ms (10 Hz), independent of
        // the different arrival rates of the five PX4 topics.
        publish_timer_ = create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&Px4InterfaceNode::publish_state, this));

        RCLCPP_INFO(
            get_logger(),
            "PX4 interface started; publishing /drone/state at 10 Hz; "
            "command API: /drone/execute_command; mode API: /drone/set_flight_mode.");
    }

private:
    using SteadyTimePoint = std::chrono::steady_clock::time_point;
    using SetFlightMode = px4_interface_node::srv::SetFlightMode;
    using ExecuteCommand = px4_interface_node::srv::ExecuteCommand;
    using CommandRequest = ExecuteCommand::Request;
    using CommandResponse = ExecuteCommand::Response;
    using Completion = std::function<void(const CommandResponse &)>;
    using CommandAck = px4_msgs::msg::VehicleCommandAck;
#if __has_include(<px4_msgs/msg/sensor_gps.hpp>)
    using GpsMessage = px4_msgs::msg::SensorGps;
#else
    using GpsMessage = px4_msgs::msg::VehicleGnss;
#endif

    static const char * gps_topic()
    {
#if __has_include(<px4_msgs/msg/sensor_gps.hpp>)
        return "/fmu/out/vehicle_gps_position";
#else
        return "/fmu/out/vehicle_gnss";
#endif
    }

    struct FlightMode
    {
        const char * name;
        uint8_t nav_state;
        float main_mode;
        float sub_mode;
    };

    static const std::array<FlightMode, 11> & flight_modes()
    {
        using VehicleStatus = px4_msgs::msg::VehicleStatus;
        // DO_SET_MODE param2/param3 are PX4 custom main/sub modes, NOT nav_state.
        // Values are defined in PX4's src/modules/commander/px4_custom_mode.h.
        static const std::array<FlightMode, 11> modes{{
            {"MANUAL", VehicleStatus::NAVIGATION_STATE_MANUAL, 1.0F, 0.0F},
            {"ALTITUDE", VehicleStatus::NAVIGATION_STATE_ALTCTL, 2.0F, 0.0F},
            {"POSITION", VehicleStatus::NAVIGATION_STATE_POSCTL, 3.0F, 0.0F},
            {"MISSION", VehicleStatus::NAVIGATION_STATE_AUTO_MISSION, 4.0F, 4.0F},
            {"LOITER", VehicleStatus::NAVIGATION_STATE_AUTO_LOITER, 4.0F, 3.0F},
            {"RTL", VehicleStatus::NAVIGATION_STATE_AUTO_RTL, 4.0F, 5.0F},
            {"OFFBOARD", VehicleStatus::NAVIGATION_STATE_OFFBOARD, 6.0F, 0.0F},
            {"STABILIZED", VehicleStatus::NAVIGATION_STATE_STAB, 7.0F, 0.0F},
            {"TAKEOFF", VehicleStatus::NAVIGATION_STATE_AUTO_TAKEOFF, 4.0F, 2.0F},
            {"LAND", VehicleStatus::NAVIGATION_STATE_AUTO_LAND, 4.0F, 6.0F},
            {"PRECISION_LAND", VehicleStatus::NAVIGATION_STATE_AUTO_PRECLAND, 4.0F, 9.0F},
        }};
        return modes;
    }

    std::string flight_mode_name(uint8_t nav_state) const
    {
        for (const auto & mode : flight_modes()) {
            if (mode.nav_state == nav_state) {
                return mode.name;
            }
        }
        return "UNKNOWN";
    }

    struct PendingCommand
    {
        Completion complete;
        CommandResponse response;
        const FlightMode * mode{nullptr};
        uint32_t px4_command{0};
        SteadyTimePoint deadline;
        uint64_t status_timestamp_at_request{0};
        uint64_t ack_timestamp{0};
    };

    struct GuardResult
    {
        uint8_t result;
        std::string message;
    };

    GuardResult command_guard(
        uint8_t kind, const FlightMode * mode, const SteadyTimePoint & steady_now) const
    {
        if (!is_fresh(has_status_, last_status_rx_, status_timeout_, steady_now)) {
            return {CommandResponse::STATUS_UNAVAILABLE, "No fresh VehicleStatus from target PX4"};
        }
        if (kind == CommandRequest::SET_MODE && failsafe_ &&
            mode->nav_state != px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_RTL &&
            mode->nav_state != px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_LAND)
        {
            return {CommandResponse::FAILSAFE_ACTIVE, "Failsafe active; only RTL or LAND mode requests allowed"};
        }
        if (kind == CommandRequest::ARM) {
            if (failsafe_) {
                return {CommandResponse::FAILSAFE_ACTIVE, "PX4 failsafe is active"};
            }
            if (gcs_connection_lost_) {
                return {CommandResponse::GCS_UNAVAILABLE, "PX4 has no ground-control-station connection"};
            }
            if (!pre_flight_checks_pass_ || calibration_enabled_ || !safety_off_) {
                return {CommandResponse::PREFLIGHT_FAILED,
                    "Arming requires passed PX4 preflight checks, safety off, and no calibration"};
            }
        }
        if (kind == CommandRequest::ARM || kind == CommandRequest::DISARM) {
            if (!is_fresh(has_land_, last_land_rx_, status_timeout_, steady_now)) {
                return {CommandResponse::LANDED_UNAVAILABLE, "No fresh land-detection telemetry"};
            }
            if (!landed_) {
                return {kind == CommandRequest::DISARM ? CommandResponse::UNSAFE_DISARM :
                    CommandResponse::PREFLIGHT_FAILED, "Vehicle must be landed; force commands are not supported"};
            }
        }
        return {CommandResponse::SUCCESS, ""};
    }

    void respond(CommandResponse response, uint8_t result, const std::string & message,
        const Completion & complete)
    {
        response.success = result == CommandResponse::SUCCESS;
        response.result = result;
        response.current_mode = flight_mode_;
        response.armed = armed_;
        response.status_fresh = is_fresh(
            has_status_, last_status_rx_, status_timeout_, std::chrono::steady_clock::now());
        response.message = message;
        complete(response);
    }

    void finish_command(uint8_t result, const std::string & message)
    {
        auto completed = std::move(pending_command_);
        RCLCPP_INFO(get_logger(), "Request %llu command %u: %s",
            static_cast<unsigned long long>(completed->response.request_id),
            completed->response.command, message.c_str());
        respond(completed->response, result, message, completed->complete);
    }

    void request_command(uint8_t kind, const std::string & mode_name, Completion complete)
    {
        check_command();
        CommandResponse response;
        response.request_id = ++request_sequence_;
        response.command = kind;
        response.ack_result = CommandResponse::NO_ACK;
        const FlightMode * mode = nullptr;
        if (kind == CommandRequest::SET_MODE) {
            for (const auto & candidate : flight_modes()) {
                if (mode_name == candidate.name) {
                    mode = &candidate;
                    break;
                }
            }
        }
        if (kind > CommandRequest::RUN_PREFLIGHT_CHECKS ||
            (kind == CommandRequest::SET_MODE && !mode) ||
            (kind != CommandRequest::SET_MODE && !mode_name.empty()))
        {
            respond(response, CommandResponse::INVALID_REQUEST,
                "Unsupported command/mode, or mode supplied for a non-mode command", complete);
            return;
        }
        if (pending_command_) {
            respond(response, CommandResponse::BUSY, "Another PX4 command is pending", complete);
            return;
        }
        const auto steady_now = std::chrono::steady_clock::now();
        const auto guard = command_guard(kind, mode, steady_now);
        if (guard.result != CommandResponse::SUCCESS) {
            respond(response, guard.result, guard.message, complete);
            return;
        }

        px4_msgs::msg::VehicleCommand command;
        if (kind == CommandRequest::SET_MODE) {
            command.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE;
            command.param1 = 1.0F;  // MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
            command.param2 = mode->main_mode;
            command.param3 = mode->sub_mode;
        } else if (kind == CommandRequest::RUN_PREFLIGHT_CHECKS) {
            command.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_RUN_PREARM_CHECKS;
        } else {
            command.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
            command.param1 = kind == CommandRequest::ARM ? 1.0F : 0.0F;
            command.param2 = 0.0F;  // Never bypass PX4 checks or force a disarm.
        }
        pending_command_ = std::make_unique<PendingCommand>();
        pending_command_->complete = std::move(complete);
        pending_command_->response = response;
        pending_command_->mode = mode;
        pending_command_->px4_command = command.command;
        pending_command_->deadline = steady_now + ack_timeout_;
        pending_command_->status_timestamp_at_request = last_status_timestamp_;
        command.timestamp = static_cast<uint64_t>(now().nanoseconds() / 1000);
        command.target_system = target_system_;
        command.target_component = target_component_;
        command.source_system = source_system_;
        command.source_component = source_component_;
        command.confirmation = 0;
        command.from_external = true;
        command_publisher_->publish(command);
    }

    void command_ack_callback(CommandAck::SharedPtr msg)
    {
        check_command();
        if (!pending_command_ || pending_command_->response.accepted ||
            msg->command != pending_command_->px4_command ||
            msg->target_system != source_system_ ||
            msg->target_component != source_component_ || msg->from_external ||
            msg->timestamp <= pending_command_->status_timestamp_at_request ||
            msg->timestamp <= pending_command_->ack_timestamp)
        {
            return;
        }
        pending_command_->response.ack_result = msg->result;
        pending_command_->response.ack_result_param1 = msg->result_param1;
        pending_command_->response.ack_result_param2 = msg->result_param2;
        pending_command_->ack_timestamp = msg->timestamp;
        if (msg->result == CommandAck::VEHICLE_CMD_RESULT_IN_PROGRESS) {
            return;  // Progress never extends the terminal-ACK deadline.
        }
        if (msg->result != CommandAck::VEHICLE_CMD_RESULT_ACCEPTED) {
            finish_command(CommandResponse::REJECTED,
                "PX4 rejected command (ACK result " + std::to_string(msg->result) + ")");
            return;
        }
        pending_command_->response.accepted = true;
        pending_command_->deadline = std::chrono::steady_clock::now() + transition_timeout_;
        check_command();
    }

    void check_command()
    {
        if (!pending_command_) {
            return;
        }
        const auto steady_now = std::chrono::steady_clock::now();
        if (steady_now >= pending_command_->deadline) {
            finish_command(pending_command_->response.accepted ?
                CommandResponse::TRANSITION_TIMEOUT : CommandResponse::ACK_TIMEOUT,
                "Timed out waiting for terminal acknowledgment or confirmed state transition");
            return;
        }
        if (!is_fresh(has_status_, last_status_rx_, status_timeout_, steady_now)) {
            finish_command(CommandResponse::STATUS_UNAVAILABLE, "VehicleStatus became stale during command");
            return;
        }
        // Different DDS topics can arrive in either order. Only PX4 timestamps
        // are compared to one another; cached pre-command state cannot confirm.
        if (!pending_command_->response.accepted ||
            last_status_timestamp_ < pending_command_->ack_timestamp)
        {
            return;
        }
        const auto kind = pending_command_->response.command;
        if (kind == CommandRequest::RUN_PREFLIGHT_CHECKS) {
            finish_command(pre_flight_checks_pass_ ? CommandResponse::SUCCESS :
                CommandResponse::PREFLIGHT_FAILED, pre_flight_checks_pass_ ?
                "PX4 accepted preflight check and reports checks passed" :
                "PX4 accepted preflight check but reports failed checks");
        } else if ((kind == CommandRequest::SET_MODE && nav_state_ == pending_command_->mode->nav_state) ||
            (kind == CommandRequest::ARM && armed_) || (kind == CommandRequest::DISARM && !armed_))
        {
            finish_command(CommandResponse::SUCCESS, "PX4 accepted and fresh telemetry confirmed requested state");
        }
    }

    // Freshness uses a monotonic clock rather than PX4 boot timestamps or ROS
    // wall time. A monotonic clock is appropriate for measuring elapsed time.
    bool is_fresh(
        bool has_received,
        const SteadyTimePoint & last_receive_time,
        const std::chrono::milliseconds timeout,
        const SteadyTimePoint & now) const
    {
        return has_received && (now - last_receive_time <= timeout);
    }

    void status_callback(px4_msgs::msg::VehicleStatus::SharedPtr msg)
    {
        if (msg->system_id != target_system_ || msg->component_id != target_component_) {
            return;
        }
        armed_ =
            msg->arming_state ==
            px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
        nav_state_ = msg->nav_state;
        arming_state_ = msg->arming_state;
        nav_state_user_intention_ = msg->nav_state_user_intention;
        failsafe_ = msg->failsafe;
        pre_flight_checks_pass_ = msg->pre_flight_checks_pass;
        gcs_connection_lost_ = msg->gcs_connection_lost;
        safety_off_ = msg->safety_off;
        calibration_enabled_ = msg->calibration_enabled;
        flight_mode_ = flight_mode_name(nav_state_);

        last_status_rx_ = std::chrono::steady_clock::now();
        has_status_ = true;
        last_status_timestamp_ = msg->timestamp;
        check_command();
    }

    void local_position_callback(
        px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
    {
        // These flags come from PX4's estimator. Freshness is added later in
        // publish_state() so estimator validity and communication validity stay
        // conceptually separate.
        position_valid_ = msg->xy_valid && msg->z_valid &&
            std::isfinite(msg->x) && std::isfinite(msg->y) && std::isfinite(msg->z);
        velocity_valid_ = msg->v_xy_valid && msg->v_z_valid;

        // VehicleLocalPosition is our single source of translational state.
        // Coordinates use NED: x North, y East, and z Down.
        x_ = msg->x;
        y_ = msg->y;
        z_ = msg->z;
        vx_ = msg->vx;
        vy_ = msg->vy;
        vz_ = msg->vz;

        last_local_position_rx_ = std::chrono::steady_clock::now();
        has_local_position_ = true;
    }

    void odometry_callback(px4_msgs::msg::VehicleOdometry::SharedPtr msg)
    {
        // Odometry is monitored for availability but does not overwrite the
        // local-position cache. Its attitude can be exposed in a later message.
        (void)msg;
        last_odometry_rx_ = std::chrono::steady_clock::now();
        has_odometry_ = true;
    }

    void gps_callback(GpsMessage::SharedPtr msg)
    {
        // A 3D or better fix plus finite coordinates is sufficient for this
        // simplified GPS-valid signal. New PX4 firmware also reports whether
        // the selected GNSS receiver passed its checks.
#if __has_include(<px4_msgs/msg/sensor_gps.hpp>)
        gps_valid_ =
            msg->fix_type >= px4_msgs::msg::SensorGps::FIX_TYPE_3D &&
            std::isfinite(msg->latitude_deg) &&
            std::isfinite(msg->longitude_deg);
#else
        gps_valid_ =
            msg->usable &&
            msg->receiver.fix_type >= px4_msgs::msg::SensorGnss::FIX_TYPE_3D &&
            std::isfinite(msg->receiver.latitude) &&
            std::isfinite(msg->receiver.longitude);
#endif

        last_gps_rx_ = std::chrono::steady_clock::now();
        has_gps_ = true;
    }

    void battery_callback(px4_msgs::msg::BatteryStatus::SharedPtr msg)
    {
        const bool valid_remaining =
            msg->connected &&
            std::isfinite(msg->remaining) &&
            msg->remaining >= 0.0F &&
            msg->remaining <= 1.0F;

        battery_remaining_ = valid_remaining
            ? msg->remaining
            : std::numeric_limits<float>::quiet_NaN();
        battery_warning_ = msg->warning;
        battery_fault_ = msg->faults != 0 || msg->is_powering_off;

        last_battery_rx_ = std::chrono::steady_clock::now();
        has_battery_ = true;
    }

    void publish_state()
    {
        const auto steady_now = std::chrono::steady_clock::now();

        const bool status_fresh = is_fresh(
            has_status_, last_status_rx_, status_timeout_, steady_now);
        const bool local_position_fresh = is_fresh(
            has_local_position_,
            last_local_position_rx_,
            fast_data_timeout_,
            steady_now);
        const bool odometry_fresh = is_fresh(
            has_odometry_, last_odometry_rx_, fast_data_timeout_, steady_now);
        const bool gps_fresh = is_fresh(
            has_gps_, last_gps_rx_, slow_data_timeout_, steady_now);
        const bool battery_fresh = is_fresh(
            has_battery_, last_battery_rx_, slow_data_timeout_, steady_now);

        // Construct one clean snapshot from the most recent cached inputs.
        px4_interface_node::msg::DroneState state;
        state.last_update_time = now();

        // Armed and flight_mode remain the last-known values. Consumers must
        // check status_fresh before treating them as current truth.
        state.armed = armed_;
        state.flight_mode = flight_mode_;
        state.arming_state = arming_state_;
        state.nav_state = nav_state_;
        state.nav_state_user_intention = nav_state_user_intention_;
        state.failsafe = failsafe_;
        state.pre_flight_checks_pass = pre_flight_checks_pass_;
        state.gcs_connection_lost = gcs_connection_lost_ || failsafe_flags_.gcs_connection_lost;
        state.safety_off = safety_off_;
        state.calibration_enabled = calibration_enabled_;
        state.system_id = has_status_ ? target_system_ : 0;
        state.component_id = has_status_ ? target_component_ : 0;
        state.landed = landed_;
        state.land_detected_fresh = is_fresh(has_land_, last_land_rx_, status_timeout_, steady_now);
        const auto arm_guard = command_guard(CommandRequest::ARM, nullptr, steady_now);
        state.ready_to_arm = !armed_ && !pending_command_ && arm_guard.result == CommandResponse::SUCCESS;
        state.arming_block_reason = armed_ ? "Already armed" :
            (pending_command_ ? "Another command is pending" : arm_guard.message);
        state.command_pending = pending_command_ != nullptr;

        // Position is declared valid only when both PX4's estimator says it is
        // valid and the source topic is still arriving on time.
        state.position_valid = position_valid_ && local_position_fresh;
        state.x = x_;
        state.y = y_;
        state.z = z_;

        // DroneState currently has no separate velocity-valid field, so publish
        // NaN when PX4 reports invalid velocity. Freshness remains explicit.
        const float unknown = std::numeric_limits<float>::quiet_NaN();
        state.vx = velocity_valid_ ? vx_ : unknown;
        state.vy = velocity_valid_ ? vy_ : unknown;
        state.vz = velocity_valid_ ? vz_ : unknown;

        state.gps_valid = gps_valid_ && gps_fresh;
        state.battery_remaining = battery_remaining_;

        state.status_fresh = status_fresh;
        state.local_position_fresh = local_position_fresh;
        state.odometry_fresh = odometry_fresh;
        state.gps_fresh = gps_fresh;
        state.battery_fresh = battery_fresh;

        state.failsafe_flags_fresh = is_fresh(
            has_failsafe_flags_, last_failsafe_flags_rx_, slow_data_timeout_, steady_now);
        state.manual_control_signal_lost = failsafe_flags_.manual_control_signal_lost;
        state.offboard_control_signal_lost = failsafe_flags_.offboard_control_signal_lost;
        state.local_position_invalid = failsafe_flags_.local_position_invalid;
        state.local_velocity_invalid = failsafe_flags_.local_velocity_invalid;
        state.global_position_invalid = failsafe_flags_.global_position_invalid;
        state.home_position_invalid = failsafe_flags_.home_position_invalid;
        state.battery_unhealthy = failsafe_flags_.battery_unhealthy || battery_fault_;
        state.battery_low_remaining_time = failsafe_flags_.battery_low_remaining_time;
        state.critical_failure = failsafe_flags_.angular_velocity_invalid ||
            failsafe_flags_.attitude_invalid || failsafe_flags_.fd_critical_failure ||
            failsafe_flags_.fd_esc_arming_failure || failsafe_flags_.fd_motor_failure;
        state.geofence_breached = failsafe_flags_.geofence_breached;
        state.mission_failure = failsafe_flags_.mission_failure;
        // Preserve both sources. The consumer evaluates each warning explicitly;
        // CHARGING (7) is not a more severe low-battery alarm.
        state.battery_warning = battery_warning_;
        state.failsafe_battery_warning = failsafe_flags_.battery_warning;

        // Hand the fully assembled message to ROS 2/DDS for delivery.
        state_publisher_->publish(state);
    }

    // PX4 input subscriptions.
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr
        status_subscription_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr
        local_position_subscription_;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr
        odometry_subscription_;
    rclcpp::Subscription<GpsMessage>::SharedPtr
        gps_subscription_;
    rclcpp::Subscription<px4_msgs::msg::BatteryStatus>::SharedPtr
        battery_subscription_;
    rclcpp::Subscription<px4_msgs::msg::FailsafeFlags>::SharedPtr failsafe_subscription_;
    px4_msgs::msg::FailsafeFlags failsafe_flags_{};
    uint8_t battery_warning_{0};
    bool battery_fault_{false};
    bool has_failsafe_flags_{false};
    SteadyTimePoint last_failsafe_flags_rx_{};

    // Clean output publisher and its periodic trigger.
    rclcpp::Publisher<px4_interface_node::msg::DroneState>::SharedPtr
        state_publisher_;
    rclcpp::TimerBase::SharedPtr publish_timer_;

    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr command_publisher_;
    rclcpp::Subscription<CommandAck>::SharedPtr command_ack_subscription_;
    rclcpp::Service<SetFlightMode>::SharedPtr mode_service_;
    rclcpp::Service<ExecuteCommand>::SharedPtr command_service_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr land_subscription_;
    rclcpp::TimerBase::SharedPtr command_timer_;
    std::unique_ptr<PendingCommand> pending_command_;
    uint64_t request_sequence_{0};
    uint8_t target_system_{1};
    uint8_t target_component_{1};
    uint8_t source_system_{1};
    uint8_t source_component_{191};
    std::chrono::milliseconds ack_timeout_{2000};
    std::chrono::milliseconds transition_timeout_{5000};
    uint64_t last_status_timestamp_{0};

    // Cached normalized values. NaN means no valid measurement is known yet.
    bool armed_{false};
    uint8_t arming_state_{0};
    uint8_t nav_state_user_intention_{0};
    bool failsafe_{true};
    bool pre_flight_checks_pass_{false};
    bool gcs_connection_lost_{true};
    bool safety_off_{false};
    bool calibration_enabled_{false};
    bool has_land_{false};
    bool landed_{false};
    SteadyTimePoint last_land_rx_{};
    uint8_t nav_state_{0};
    std::string flight_mode_{"UNKNOWN"};

    bool position_valid_{false};
    bool velocity_valid_{false};
    float x_{std::numeric_limits<float>::quiet_NaN()};
    float y_{std::numeric_limits<float>::quiet_NaN()};
    float z_{std::numeric_limits<float>::quiet_NaN()};
    float vx_{std::numeric_limits<float>::quiet_NaN()};
    float vy_{std::numeric_limits<float>::quiet_NaN()};
    float vz_{std::numeric_limits<float>::quiet_NaN()};

    bool gps_valid_{false};
    float battery_remaining_{std::numeric_limits<float>::quiet_NaN()};

    // A timestamp alone cannot distinguish "never received" from an old
    // default time point, so each source also has an explicit has_* flag.
    bool has_status_{false};
    bool has_local_position_{false};
    bool has_odometry_{false};
    bool has_gps_{false};
    bool has_battery_{false};

    SteadyTimePoint last_status_rx_{};
    SteadyTimePoint last_local_position_rx_{};
    SteadyTimePoint last_odometry_rx_{};
    SteadyTimePoint last_gps_rx_{};
    SteadyTimePoint last_battery_rx_{};

    // Fast estimator topics should update within 500 ms. Status, GPS, and
    // battery can naturally update more slowly, so allow two seconds.
    const std::chrono::milliseconds fast_data_timeout_{500};
    const std::chrono::milliseconds status_timeout_{2000};
    const std::chrono::milliseconds slow_data_timeout_{2000};
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Px4InterfaceNode>());
    rclcpp::shutdown();
    return 0;
}
