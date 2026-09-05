cmake_minimum_required(VERSION 3.8)
project(drcsim_gazebo_ros_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(gz-common5 REQUIRED)
find_package(gz-math7 REQUIRED)

find_package(rclcpp REQUIRED)
find_package(atlas_msgs REQUIRED)
find_package(osrf_msgs REQUIRED)
find_package(sandia_hand_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(handle_msgs REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

add_library(ContactModelPlugin SHARED src/ContactModelPlugin.cpp)
target_link_libraries(ContactModelPlugin
  gz-sim8::gz-sim8
  gz-plugin2::register
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10
  gz-common5::gz-common5
)
install(TARGETS ContactModelPlugin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

add_library(SandiaHandPlugin SHARED src/SandiaHandPlugin.cpp)
target_link_libraries(SandiaHandPlugin
  gz-sim8::gz-sim8
  gz-plugin2::register
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10
  gz-common5::gz-common5
)
ament_target_dependencies(SandiaHandPlugin
  rclcpp
  atlas_msgs
  osrf_msgs
  sandia_hand_msgs
  sensor_msgs
)
install(TARGETS SandiaHandPlugin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

add_library(IRobotHandPlugin SHARED src/IRobotHandPlugin.cpp)
target_link_libraries(IRobotHandPlugin
  gz-sim8::gz-sim8
  gz-plugin2::register
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10
  gz-common5::gz-common5
)
ament_target_dependencies(IRobotHandPlugin
  rclcpp
  handle_msgs
  sensor_msgs
)
install(TARGETS IRobotHandPlugin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

add_library(RobotiqHandPlugin SHARED src/RobotiqHandPlugin.cpp)
target_link_libraries(RobotiqHandPlugin
  gz-sim8::gz-sim8
  gz-plugin2::register
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10
  gz-common5::gz-common5
  gz-math7::gz-math7
)
ament_target_dependencies(RobotiqHandPlugin
  rclcpp
  atlas_msgs
  sensor_msgs
)
install(TARGETS RobotiqHandPlugin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

add_library(MultiSenseSLPlugin SHARED src/MultiSenseSLPlugin.cpp)
target_link_libraries(MultiSenseSLPlugin
  gz-sim8::gz-sim8
  gz-plugin2::register
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10
  gz-common5::gz-common5
  gz-math7::gz-math7
)
ament_target_dependencies(MultiSenseSLPlugin
  rclcpp
  sensor_msgs
  std_msgs
)
install(TARGETS MultiSenseSLPlugin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

# Let gz-sim find these plugin libraries at runtime.
ament_environment_hooks(
  "${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in"
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.hpp"
)

if(BUILD_TESTING)
  # Not using ament_lint_auto_find_test_dependencies() here: this package
  # is being ported one plugin per branch (see CLAUDE.md), so most of
  # include/ and src/ is still the original ROS1/Gazebo-Classic source,
  # not yet touched. Exclude everything except what's actually been
  # ported from the C++ linters -- each subsequent plugin's branch should
  # remove its own newly-ported files from this list as it lands.
  file(GLOB_RECURSE AMENT_LINT_ALL_SOURCE
    "${CMAKE_CURRENT_SOURCE_DIR}/include/*"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*"
  )
  set(AMENT_LINT_PORTED
    "${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/ContactModelPlugin.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/ContactModelPlugin.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/SandiaHandPlugin.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/SandiaHandPlugin.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/IRobotHandPlugin.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/IRobotHandPlugin.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/RobotiqHandPlugin.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/RobotiqHandPlugin.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/MultiSenseSLPlugin.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/MultiSenseSLPlugin.cpp"
  )
  set(AMENT_LINT_EXCLUDE ${AMENT_LINT_ALL_SOURCE})
  list(REMOVE_ITEM AMENT_LINT_EXCLUDE ${AMENT_LINT_PORTED})

  find_package(ament_cmake_copyright REQUIRED)
  ament_copyright(EXCLUDE ${AMENT_LINT_EXCLUDE})

  find_package(ament_cmake_cppcheck REQUIRED)
  ament_cppcheck(EXCLUDE ${AMENT_LINT_EXCLUDE})

  find_package(ament_cmake_cpplint REQUIRED)
  ament_cpplint(EXCLUDE ${AMENT_LINT_EXCLUDE})

  find_package(ament_cmake_uncrustify REQUIRED)
  ament_uncrustify(EXCLUDE ${AMENT_LINT_EXCLUDE})

  find_package(ament_cmake_lint_cmake REQUIRED)
  ament_lint_cmake()

  find_package(ament_cmake_xmllint REQUIRED)
  ament_xmllint()

  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_contact_model_plugin
    test/test_contact_model_plugin.cc)
  target_link_libraries(test_contact_model_plugin
    gz-sim8::gz-sim8
    gz-transport13::gz-transport13
    gz-msgs10::gz-msgs10
  )
  target_compile_definitions(test_contact_model_plugin PRIVATE
    "PLUGIN_BUILD_DIR=\"$<TARGET_FILE_DIR:ContactModelPlugin>\""
    "TEST_WORLD_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/test/worlds\""
  )
  add_dependencies(test_contact_model_plugin ContactModelPlugin)

  ament_add_gtest(test_sandia_hand_plugin
    test/test_sandia_hand_plugin.cpp)
  target_link_libraries(test_sandia_hand_plugin
    gz-sim8::gz-sim8
  )
  ament_target_dependencies(test_sandia_hand_plugin rclcpp atlas_msgs)
  target_compile_definitions(test_sandia_hand_plugin PRIVATE
    "PLUGIN_BUILD_DIR=\"$<TARGET_FILE_DIR:SandiaHandPlugin>\""
    "TEST_WORLD_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/test/worlds\""
  )
  add_dependencies(test_sandia_hand_plugin SandiaHandPlugin)

  ament_add_gtest(test_sandia_hand_plugin_joints
    test/test_sandia_hand_plugin_joints.cpp)
  target_link_libraries(test_sandia_hand_plugin_joints
    gz-sim8::gz-sim8
  )
  ament_target_dependencies(test_sandia_hand_plugin_joints
    rclcpp
    osrf_msgs
    sandia_hand_msgs
    sensor_msgs
  )
  target_compile_definitions(test_sandia_hand_plugin_joints PRIVATE
    "PLUGIN_BUILD_DIR=\"$<TARGET_FILE_DIR:SandiaHandPlugin>\""
    "TEST_WORLD_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/test/worlds\""
  )
  add_dependencies(test_sandia_hand_plugin_joints SandiaHandPlugin)

  ament_add_gtest(test_irobot_hand_plugin
    test/test_irobot_hand_plugin.cpp)
  target_link_libraries(test_irobot_hand_plugin
    gz-sim8::gz-sim8
  )
  ament_target_dependencies(test_irobot_hand_plugin rclcpp)
  target_compile_definitions(test_irobot_hand_plugin PRIVATE
    "PLUGIN_BUILD_DIR=\"$<TARGET_FILE_DIR:IRobotHandPlugin>\""
    "TEST_WORLD_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/test/worlds\""
  )
  add_dependencies(test_irobot_hand_plugin IRobotHandPlugin)

  ament_add_gtest(test_robotiq_hand_plugin
    test/test_robotiq_hand_plugin.cpp)
  target_link_libraries(test_robotiq_hand_plugin
    gz-sim8::gz-sim8
  )
  ament_target_dependencies(test_robotiq_hand_plugin
    rclcpp
    atlas_msgs
    sensor_msgs
  )
  target_compile_definitions(test_robotiq_hand_plugin PRIVATE
    "PLUGIN_BUILD_DIR=\"$<TARGET_FILE_DIR:RobotiqHandPlugin>\""
    "TEST_WORLD_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/test/worlds\""
  )
  add_dependencies(test_robotiq_hand_plugin RobotiqHandPlugin)

  ament_add_gtest(test_multisense_sl_plugin
    test/test_multisense_sl_plugin.cpp)
  target_link_libraries(test_multisense_sl_plugin
    gz-sim8::gz-sim8
  )
  ament_target_dependencies(test_multisense_sl_plugin
    rclcpp
    sensor_msgs
    std_msgs
  )
  target_compile_definitions(test_multisense_sl_plugin PRIVATE
    "PLUGIN_BUILD_DIR=\"$<TARGET_FILE_DIR:MultiSenseSLPlugin>\""
    "TEST_WORLD_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/test/worlds\""
  )
  add_dependencies(test_multisense_sl_plugin MultiSenseSLPlugin)
endif()

ament_package()
