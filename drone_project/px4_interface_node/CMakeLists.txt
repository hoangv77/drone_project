# Require a reasonably modern CMake version. ROS 2 Humble supports this version.
cmake_minimum_required(VERSION 3.8)

# Define the package/project name. This must match the name in package.xml.
project(px4_interface_node)

# Enable useful compiler warnings when compiling with GCC or Clang.
# Warnings help us catch suspicious C++ without preventing the build.
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Locate the ROS 2 build system and every package used by our code.
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# Locate ROS's message-generation and C++ type-support packages.
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)

# Generate C++ code from our custom DroneState.msg definition.
#
# DEPENDENCIES lists message packages referenced inside DroneState.msg.
# We need builtin_interfaces because the message contains
# builtin_interfaces/Time last_update_time.
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DroneState.msg"
  DEPENDENCIES builtin_interfaces
)

# Compile src/px4_interface_node.cpp into an executable named
# px4_interface_node.
add_executable(px4_interface_node_executable
  src/px4_interface_node.cpp
)

set_target_properties(px4_interface_node_executable PROPERTIES
  OUTPUT_NAME px4_interface_node
)

# Link the ordinary ROS 2 dependencies used by the C++ source file.
ament_target_dependencies(px4_interface_node_executable
  rclcpp
  px4_msgs
  builtin_interfaces
)

# Obtain the C++ type-support target generated for DroneState.msg.
# This is necessary because the executable and custom message live in the
# same package.
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp"
)

# Link the executable to the generated DroneState C++ type support.
target_link_libraries(px4_interface_node_executable
  "${cpp_typesupport_target}"
)

# Install the executable where `ros2 run` expects ROS 2 executables.
# After building, it can be launched with:
#   ros2 run px4_interface_node px4_interface_node
install(TARGETS
  px4_interface_node_executable
  DESTINATION lib/${PROJECT_NAME}
)

# Tell packages that depend on us that our generated message needs the
# ROS interface runtime.
ament_export_dependencies(rosidl_default_runtime)

# Finish configuring this as an ament package.
# Keep this as the final command in the file.
ament_package()
