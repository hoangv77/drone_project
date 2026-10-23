// ROS 2 C++ API: nodes, publishers, subscriptions, timers, and logging.
#include <rclcpp/rclcpp.hpp>

// Our clean internal state message generated from msg/DroneState.msg.
#include <px4_interface_node/msg/drone_state.hpp>

// PX4 telemetry message types used by this read-only interface.
#include <px4_msgs/msg/battery_status.hpp>
#include <px4_msgs/msg/sensor_gps.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <string>

class Px4InterfaceNode : public rclcpp::Node
{
public:
    Px4InterfaceNode()
        : Node("px4_interface_node")
    {
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
            create_subscription<px4_msgs::msg::SensorGps>(
                "/fmu/out/vehicle_gps_position",
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

        // This is the clean topic consumed by nodes we control. A depth of 10
        // means DDS may queue up to ten outgoing samples for a slow subscriber.
        state_publisher_ =
            create_publisher<px4_interface_node::msg::DroneState>(
                "/drone/state",
                rclcpp::QoS(10).reliable());

        // Publish one assembled snapshot every 100 ms (10 Hz), independent of
        // the different arrival rates of the five PX4 topics.
        publish_timer_ = create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&Px4InterfaceNode::publish_state, this));

        RCLCPP_INFO(
            get_logger(),
            "Read-only PX4 interface started; publishing /drone/state at 10 Hz.");
    }

private:
    using SteadyTimePoint = std::chrono::steady_clock::time_point;

    std::string flight_mode_name(uint8_t nav_state) const
    {
        using VehicleStatus = px4_msgs::msg::VehicleStatus;

        switch (nav_state) {
            case VehicleStatus::NAVIGATION_STATE_MANUAL:
                return "MANUAL";
            case VehicleStatus::NAVIGATION_STATE_ALTCTL:
                return "ALTITUDE";
            case VehicleStatus::NAVIGATION_STATE_POSCTL:
                return "POSITION";
            case VehicleStatus::NAVIGATION_STATE_AUTO_MISSION:
                return "MISSION";
            case VehicleStatus::NAVIGATION_STATE_AUTO_LOITER:
                return "LOITER";
            case VehicleStatus::NAVIGATION_STATE_AUTO_RTL:
                return "RTL";
            case VehicleStatus::NAVIGATION_STATE_OFFBOARD:
                return "OFFBOARD";
            case VehicleStatus::NAVIGATION_STATE_STAB:
                return "STABILIZED";
            case VehicleStatus::NAVIGATION_STATE_AUTO_TAKEOFF:
                return "TAKEOFF";
            case VehicleStatus::NAVIGATION_STATE_AUTO_LAND:
                return "LAND";
            case VehicleStatus::NAVIGATION_STATE_AUTO_PRECLAND:
                return "PRECISION_LAND";
            default:
                return "UNKNOWN";
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
        armed_ =
            msg->arming_state ==
            px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
        nav_state_ = msg->nav_state;
        flight_mode_ = flight_mode_name(nav_state_);

        last_status_rx_ = std::chrono::steady_clock::now();
        has_status_ = true;
    }

    void local_position_callback(
        px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
    {
        // These flags come from PX4's estimator. Freshness is added later in
        // publish_state() so estimator validity and communication validity stay
        // conceptually separate.
        position_valid_ = msg->xy_valid && msg->z_valid;
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

    void gps_callback(px4_msgs::msg::SensorGps::SharedPtr msg)
    {
        // A 3D or better fix plus finite coordinates is sufficient for this
        // first simplified GPS-valid signal.
        gps_valid_ =
            msg->fix_type >= px4_msgs::msg::SensorGps::FIX_TYPE_3D &&
            std::isfinite(msg->latitude_deg) &&
            std::isfinite(msg->longitude_deg);

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
    rclcpp::Subscription<px4_msgs::msg::SensorGps>::SharedPtr
        gps_subscription_;
    rclcpp::Subscription<px4_msgs::msg::BatteryStatus>::SharedPtr
        battery_subscription_;

    // Clean output publisher and its periodic trigger.
    rclcpp::Publisher<px4_interface_node::msg::DroneState>::SharedPtr
        state_publisher_;
    rclcpp::TimerBase::SharedPtr publish_timer_;

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