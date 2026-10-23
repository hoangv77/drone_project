// ROS 2 C++ API: nodes, publishers, subscriptions, timers, and logging.
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>

// Our clean internal state message generated from msg/DroneState.msg.
#include <px4_interface_node/msg/drone_state.hpp>
#include <px4_interface_node/srv/set_flight_mode.hpp>

// PX4 telemetry and command messages.
#include <px4_msgs/msg/battery_status.hpp>
#include <px4_msgs/msg/failsafe_flags.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>
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
        mode_timeout_ = timeout_parameter("mode_timeout_ms", 5000);

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

        land_subscription_ = create_subscription<px4_msgs::msg::VehicleLandDetected>(
            "/fmu/out/vehicle_land_detected", px4_qos,
            [this](px4_msgs::msg::VehicleLandDetected::SharedPtr msg) {
                landed_ = msg->landed;
                has_land_ = true;
                last_land_rx_ = std::chrono::steady_clock::now();
            });
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
                request_flight_mode(header, request);
            });
        command_timer_ = create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&Px4InterfaceNode::check_mode_request, this));

        // Publish one assembled snapshot every 100 ms (10 Hz), independent of
        // the different arrival rates of the five PX4 topics.
        publish_timer_ = create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&Px4InterfaceNode::publish_state, this));

        RCLCPP_INFO(
            get_logger(),
            "PX4 interface started; publishing /drone/state at 10 Hz; "
            "flight-mode service: /drone/set_flight_mode.");
    }

private:
    using SteadyTimePoint = std::chrono::steady_clock::time_point;
    using SetFlightMode = px4_interface_node::srv::SetFlightMode;
    using ModeResponse = SetFlightMode::Response;
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

    struct PendingMode
    {
        std::shared_ptr<rmw_request_id_t> header;
        const FlightMode * mode;
        SteadyTimePoint deadline;
        uint64_t status_timestamp_at_request;
        uint64_t ack_timestamp{0};
        uint8_t ack_result{ModeResponse::NO_ACK};
        bool accepted{false};
    };

    void send_mode_response(
        const std::shared_ptr<rmw_request_id_t> & header, uint8_t result,
        bool accepted, uint8_t ack_result, const std::string & message)
    {
        ModeResponse response;
        response.success = result == ModeResponse::SUCCESS;
        response.accepted = accepted;
        response.result = result;
        response.ack_result = ack_result;
        response.current_mode = flight_mode_;
        response.status_fresh = is_fresh(
            has_status_, last_status_rx_, status_timeout_, std::chrono::steady_clock::now());
        response.message = message;
        mode_service_->send_response(*header, response);
    }

    void finish_mode_request(uint8_t result, const std::string & message)
    {
        auto completed = std::move(pending_mode_);
        RCLCPP_INFO(get_logger(), "Mode %s: %s", completed->mode->name, message.c_str());
        send_mode_response(
            completed->header, result, completed->accepted, completed->ack_result, message);
    }

    void request_flight_mode(
        const std::shared_ptr<rmw_request_id_t> & header,
        const SetFlightMode::Request::SharedPtr & request)
    {
        check_mode_request();
        const FlightMode * requested_mode = nullptr;
        for (const auto & mode : flight_modes()) {
            if (request->mode == mode.name) {
                requested_mode = &mode;
                break;
            }
        }
        if (!requested_mode) {
            send_mode_response(header, ModeResponse::INVALID_MODE, false,
                ModeResponse::NO_ACK, "Unsupported flight mode: " + request->mode);
            return;
        }
        if (pending_mode_) {
            send_mode_response(header, ModeResponse::BUSY, false,
                ModeResponse::NO_ACK, "Another flight-mode request is pending");
            return;
        }
        const auto steady_now = std::chrono::steady_clock::now();
        if (!is_fresh(has_status_, last_status_rx_, status_timeout_, steady_now)) {
            send_mode_response(header, ModeResponse::STATUS_UNAVAILABLE, false,
                ModeResponse::NO_ACK, "No fresh VehicleStatus from the target PX4");
            return;
        }

        pending_mode_ = std::make_unique<PendingMode>(PendingMode{
            header, requested_mode, steady_now + ack_timeout_, last_status_timestamp_});

        px4_msgs::msg::VehicleCommand command;
        command.timestamp = static_cast<uint64_t>(now().nanoseconds() / 1000);
        command.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE;
        command.param1 = 1.0F;  // MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
        command.param2 = requested_mode->main_mode;
        command.param3 = requested_mode->sub_mode;
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
        check_mode_request();
        if (!pending_mode_ || pending_mode_->accepted ||
            msg->command != px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE ||
            msg->target_system != source_system_ ||
            msg->target_component != source_component_ || msg->from_external ||
            msg->timestamp <= pending_mode_->status_timestamp_at_request ||
            msg->timestamp <= pending_mode_->ack_timestamp)
        {
            return;
        }
        pending_mode_->ack_result = msg->result;
        pending_mode_->ack_timestamp = msg->timestamp;
        if (msg->result == CommandAck::VEHICLE_CMD_RESULT_IN_PROGRESS) {
            // Keep waiting for a terminal ACK without extending the deadline.
            return;
        }
        if (msg->result != CommandAck::VEHICLE_CMD_RESULT_ACCEPTED) {
            finish_mode_request(ModeResponse::REJECTED,
                "PX4 rejected the mode command (ACK result " + std::to_string(msg->result) + ")");
            return;
        }
        pending_mode_->accepted = true;
        pending_mode_->deadline = std::chrono::steady_clock::now() + mode_timeout_;
        check_mode_request();
    }

    void check_mode_request()
    {
        if (!pending_mode_) {
            return;
        }
        const auto steady_now = std::chrono::steady_clock::now();
        if (steady_now >= pending_mode_->deadline) {
            const bool accepted = pending_mode_->accepted;
            finish_mode_request(
                accepted ? ModeResponse::MODE_TIMEOUT : ModeResponse::ACK_TIMEOUT,
                accepted ? "PX4 accepted, but fresh status did not confirm mode entry" :
                "Timed out waiting for a terminal PX4 acknowledgment");
            return;
        }
        // Status and ACKs can arrive in either order on different DDS topics.
        // Compare their PX4 timestamps, never PX4 timestamps against ROS time.
        if (pending_mode_->accepted &&
            is_fresh(has_status_, last_status_rx_, status_timeout_, steady_now) &&
            last_status_timestamp_ >= pending_mode_->ack_timestamp &&
            nav_state_ == pending_mode_->mode->nav_state)
        {
            finish_mode_request(ModeResponse::SUCCESS, "PX4 accepted and entered the requested mode");
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
        flight_mode_ = flight_mode_name(nav_state_);
        vehicle_status_ = *msg;

        last_status_rx_ = std::chrono::steady_clock::now();
        has_status_ = true;
        last_status_timestamp_ = msg->timestamp;
        check_mode_request();
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

        state.failsafe = vehicle_status_.failsafe;
        state.pre_flight_checks_pass = vehicle_status_.pre_flight_checks_pass;
        state.gcs_connection_lost = vehicle_status_.gcs_connection_lost ||
            failsafe_flags_.gcs_connection_lost;
        state.safety_off = vehicle_status_.safety_off;
        state.calibration_enabled = vehicle_status_.calibration_enabled;
        state.arming_state = vehicle_status_.arming_state;
        state.nav_state = vehicle_status_.nav_state;
        state.nav_state_user_intention = vehicle_status_.nav_state_user_intention;
        state.landed = landed_;
        state.land_detected_fresh = is_fresh(
            has_land_, last_land_rx_, slow_data_timeout_, steady_now);
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
    rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr land_subscription_;
    rclcpp::Subscription<px4_msgs::msg::FailsafeFlags>::SharedPtr failsafe_subscription_;
    px4_msgs::msg::VehicleStatus vehicle_status_{};
    px4_msgs::msg::FailsafeFlags failsafe_flags_{};
    uint8_t battery_warning_{0};
    bool battery_fault_{false};
    bool landed_{false};
    bool has_land_{false};
    bool has_failsafe_flags_{false};
    SteadyTimePoint last_land_rx_{};
    SteadyTimePoint last_failsafe_flags_rx_{};

    // Clean output publisher and its periodic trigger.
    rclcpp::Publisher<px4_interface_node::msg::DroneState>::SharedPtr
        state_publisher_;
    rclcpp::TimerBase::SharedPtr publish_timer_;

    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr command_publisher_;
    rclcpp::Subscription<CommandAck>::SharedPtr command_ack_subscription_;
    rclcpp::Service<SetFlightMode>::SharedPtr mode_service_;
    rclcpp::TimerBase::SharedPtr command_timer_;
    std::unique_ptr<PendingMode> pending_mode_;
    uint8_t target_system_{1};
    uint8_t target_component_{1};
    uint8_t source_system_{1};
    uint8_t source_component_{191};
    std::chrono::milliseconds ack_timeout_{2000};
    std::chrono::milliseconds mode_timeout_{5000};
    uint64_t last_status_timestamp_{0};

    // Cached normalized values. NaN means no valid measurement is known yet.
    bool armed_{false};
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
