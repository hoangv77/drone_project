#ifndef STATE_MACHINE_MANAGER_INPUT_LEASE_HPP
#define STATE_MACHINE_MANAGER_INPUT_LEASE_HPP

#include <rclcpp/rclcpp.hpp>
#include <state_machine_manager/mission_machine.hpp>

#include <algorithm>
#include <optional>

namespace state_machine_manager
{

// ROS graph identity is a consistency check within a trusted DDS domain.
class InputLease
{
public:
  std::optional<Time> observe(rclcpp::Node & node, const char * topic,
    const builtin_interfaces::msg::Time & stamp, const rclcpp::MessageInfo & info,
    Milliseconds limit, const std::string & authority = "")
  {
    if (stamp.sec <= 0 || stamp.nanosec >= 1000000000u) {return std::nullopt;}
    const auto source = rclcpp::Time(stamp, RCL_SYSTEM_TIME).nanoseconds();
    const auto age = rclcpp::Clock(RCL_SYSTEM_TIME).now().nanoseconds() - source;
    if (source <= watermark_ || age < 0 || age > std::chrono::duration_cast<
        std::chrono::nanoseconds>(limit).count()) {return std::nullopt;}
    const auto endpoints = node.get_publishers_info_by_topic(topic);
    const auto & gid = info.get_rmw_message_info().publisher_gid;
    if (endpoints.size() != 1 ||
      (!authority.empty() &&
      (endpoints.front().node_namespace() != "/" || endpoints.front().node_name() != authority)))
    {return std::nullopt;}
    const auto endpoint_gid = endpoints.front().endpoint_gid();
    if (!std::equal(endpoint_gid.begin(), endpoint_gid.end(), gid.data)) {return std::nullopt;}
    watermark_ = source;
    return Clock::now() - std::chrono::nanoseconds(age);
  }

private:
  int64_t watermark_{0};
};

}  // namespace state_machine_manager
#endif
