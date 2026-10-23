# drone_project

ROS 2 interface for reading PX4 telemetry and publishing a custom `DroneState` message.

## Structure

```text
px4_interface_node/
├── CMakeLists.txt
├── package.xml
├── msg/DroneState.msg
└── src/px4_interface_node.cpp
```

## Prerequisites

- ROS 2 with `ament_cmake`, `rclcpp`, and ROS interface generation support
- `px4_msgs` compatible with the PX4 firmware in use
- A ROS 2 workspace and a PX4 telemetry source (such as PX4 SITL)

## Build

Place `px4_interface_node` inside the `src/` directory of your ROS 2 workspace, then run from the workspace root:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --packages-select px4_interface_node
source install/setup.bash
```

## Run

```bash
ros2 run px4_interface_node px4_interface_node
```

Inspect the custom message definition in `px4_interface_node/msg/DroneState.msg` and use `ros2 topic list` to find the node's published topic.

## Notes

The ROS 2 package currently contains placeholder maintainer metadata in `package.xml`. Update it before publishing a release. The code has not been compiled or flight-tested as part of this repository packaging.
