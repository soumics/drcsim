cmake_minimum_required(VERSION 3.8)
project(sandia_hand_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RelativeJointCommands.msg"
  "msg/RawTactile.msg"
  "msg/RawPalmState.msg"
  "msg/RawMoboState.msg"
  "msg/RawFingerState.msg"
  "msg/RawFingerInertial.msg"
  "msg/Parameter.msg"
  "msg/CalFingerState.msg"
  "msg/SimpleGrasp.msg"
  "msg/RawFingerCommands.msg"
  "srv/SimpleGraspSrv.srv"
  "srv/SimpleGraspWithSlew.srv"
  "srv/SetParameters.srv"
  "srv/SetJointLimitPolicy.srv"
  "srv/SetFingerHome.srv"
  "srv/GetParameters.srv"
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

  ament_add_gtest(test_sandia_hand_msgs test/test_sandia_hand_msgs.cpp)
  target_link_libraries(test_sandia_hand_msgs "${cpp_typesupport_target}")
  ament_target_dependencies(test_sandia_hand_msgs rclcpp std_msgs)
endif()

ament_package()
