cmake_minimum_required(VERSION 3.8)
project(drcsim_gazebo_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz-math7 REQUIRED)
find_package(gz-common5 REQUIRED)

include_directories(include)

add_library(DRCVehiclePlugin SHARED src/DRCVehiclePlugin.cpp)
target_link_libraries(DRCVehiclePlugin
  gz-sim8::gz-sim8
  gz-plugin2::register
  gz-math7::gz-math7
  gz-common5::gz-common5
)
# Usage requirements for downstream packages that link against this target
# (drcsim_gazebo_ros_plugins' DRCVehicleROSPlugin subclasses DRCVehiclePlugin)
# -- plain include_directories() below is enough to build this package's own
# targets, but doesn't propagate through an exported target.
target_include_directories(DRCVehiclePlugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
install(TARGETS DRCVehiclePlugin
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

add_library(DRCBuildingPlugin SHARED src/DRCBuildingPlugin.cpp)
target_link_libraries(DRCBuildingPlugin
  gz-sim8::gz-sim8
  gz-plugin2::register
  gz-math7::gz-math7
  gz-common5::gz-common5
)
install(TARGETS DRCBuildingPlugin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

add_library(FollowCameraPlugin SHARED src/FollowCameraPlugin.cpp)
target_link_libraries(FollowCameraPlugin
  gz-sim8::gz-sim8
  gz-plugin2::register
  gz-math7::gz-math7
  gz-common5::gz-common5
)
install(TARGETS FollowCameraPlugin
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
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_drc_building_plugin
    test/test_drc_building_plugin.cc)
  target_link_libraries(test_drc_building_plugin gz-sim8::gz-sim8)
  target_compile_definitions(test_drc_building_plugin PRIVATE
    "PLUGIN_BUILD_DIR=\"$<TARGET_FILE_DIR:DRCBuildingPlugin>\""
    "TEST_WORLD_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/test/worlds\""
  )
  add_dependencies(test_drc_building_plugin
    DRCBuildingPlugin DRCVehiclePlugin)

  ament_add_gtest(test_drc_vehicle_plugin
    test/test_drc_vehicle_plugin.cc)
  target_link_libraries(test_drc_vehicle_plugin gz-sim8::gz-sim8)
  target_compile_definitions(test_drc_vehicle_plugin PRIVATE
    "PLUGIN_BUILD_DIR=\"$<TARGET_FILE_DIR:DRCVehiclePlugin>\""
    "TEST_WORLD_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/test/worlds\""
  )
  add_dependencies(test_drc_vehicle_plugin
    DRCBuildingPlugin DRCVehiclePlugin)

  ament_add_gtest(test_follow_camera_plugin
    test/test_follow_camera_plugin.cpp)
  target_link_libraries(test_follow_camera_plugin gz-sim8::gz-sim8)
  target_compile_definitions(test_follow_camera_plugin PRIVATE
    "PLUGIN_BUILD_DIR=\"$<TARGET_FILE_DIR:FollowCameraPlugin>\""
    "TEST_WORLD_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/test/worlds\""
  )
  add_dependencies(test_follow_camera_plugin FollowCameraPlugin)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(gz-sim8 gz-plugin2 gz-math7 gz-common5)

ament_package()
