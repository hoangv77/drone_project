cmake_minimum_required(VERSION 3.8)
project(safety_monitor)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(px4_interface_node REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SafetyStatus.msg"
  DEPENDENCIES builtin_interfaces
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
add_executable(safety_monitor_node src/safety_monitor.cpp)
set_target_properties(safety_monitor_node PROPERTIES OUTPUT_NAME safety_monitor)
target_compile_features(safety_monitor_node PUBLIC cxx_std_17)
target_include_directories(safety_monitor_node PRIVATE include)
ament_target_dependencies(safety_monitor_node rclcpp rcl_interfaces px4_interface_node)
target_link_libraries(safety_monitor_node "${cpp_typesupport_target}")

# Export the reusable header with an interface target. Consumers of a package
# that also generates messages use imported targets rather than legacy includes.
add_library(safety_gate INTERFACE)
target_include_directories(safety_gate INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(safety_gate INTERFACE "${cpp_typesupport_target}")
install(TARGETS safety_gate EXPORT export_safety_gate)
ament_export_targets(export_safety_gate)

install(TARGETS safety_monitor_node DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS tools/sitl_observe.py DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
ament_export_include_directories(include)
ament_export_dependencies(rosidl_default_runtime px4_interface_node)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_safety_policy test/test_safety_policy.cpp)
  target_compile_features(test_safety_policy PUBLIC cxx_std_17)
  target_include_directories(test_safety_policy PRIVATE include)
  ament_target_dependencies(test_safety_policy px4_interface_node)
  target_link_libraries(test_safety_policy "${cpp_typesupport_target}")
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_safety_pipeline.py TIMEOUT 120)
endif()
ament_package()
