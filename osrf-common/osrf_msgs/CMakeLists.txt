cmake_minimum_required(VERSION 3.8)
project(osrf_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/JointCommands.msg"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  find_package(rclcpp REQUIRED)

  rosidl_get_typesupport_target(cpp_typesupport_target
    "${PROJECT_NAME}" "rosidl_typesupport_cpp")

  ament_add_gtest(test_osrf_msgs test/test_osrf_msgs.cpp)
  target_link_libraries(test_osrf_msgs "${cpp_typesupport_target}")
  ament_target_dependencies(test_osrf_msgs rclcpp std_msgs)
endif()

ament_package()
