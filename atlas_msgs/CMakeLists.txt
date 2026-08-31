cmake_minimum_required(VERSION 3.8)
project(atlas_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AtlasBehaviorFeedback.msg"
  "msg/AtlasBehaviorManipulateFeedback.msg"
  "msg/AtlasBehaviorManipulateParams.msg"
  "msg/AtlasBehaviorPelvisServoParams.msg"
  "msg/AtlasBehaviorStandFeedback.msg"
  "msg/AtlasBehaviorStandParams.msg"
  "msg/AtlasBehaviorStepData.msg"
  "msg/AtlasBehaviorStepFeedback.msg"
  "msg/AtlasBehaviorStepParams.msg"
  "msg/AtlasBehaviorWalkFeedback.msg"
  "msg/AtlasBehaviorWalkParams.msg"
  "msg/AtlasCommand.msg"
  "msg/AtlasPositionData.msg"
  "msg/AtlasSimInterfaceCommand.msg"
  "msg/AtlasSimInterfaceState.msg"
  "msg/AtlasState.msg"
  "msg/ControllerStatistics.msg"
  "msg/ForceTorqueSensors.msg"
  "msg/SModelRobotInput.msg"
  "msg/SModelRobotOutput.msg"
  "msg/SynchronizationStatistics.msg"
  "msg/Test.msg"
  "msg/VRCScore.msg"
  "srv/AtlasFilters.srv"
  "srv/GetJointDamping.srv"
  "srv/ResetControls.srv"
  "srv/SetJointDamping.srv"
  "action/WalkDemo.action"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  find_package(rclcpp REQUIRED)

  rosidl_get_typesupport_target(cpp_typesupport_target
    "${PROJECT_NAME}" "rosidl_typesupport_cpp")

  ament_add_gtest(test_atlas_msgs test/test_atlas_msgs.cpp)
  target_link_libraries(test_atlas_msgs "${cpp_typesupport_target}")
  ament_target_dependencies(test_atlas_msgs rclcpp std_msgs geometry_msgs)
endif()

ament_package()
