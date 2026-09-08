cmake_minimum_required(VERSION 3.8)
project(drcsim_gazebo)

find_package(ament_cmake REQUIRED)

# launch/, config/, test/, and scripts/ still hold most of the original
# ROS1 roslaunch/rostest/rospy source (194 rostest files, 63 more launch
# files, most of the old scripts) -- not yet ported, left in place
# untouched rather than deleted (see CLAUDE.md for what's deferred vs.
# what was confirmed dead and actually removed). Installing them
# wholesale is harmless: nothing in a ROS 2 workspace will invoke a
# ROS1-format .launch/rostest file.
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  # Only lint what's actually been ported in this branch -- see the same
  # rationale/pattern already established in
  # drcsim_gazebo_ros_plugins/CMakeLists.txt for why not
  # ament_lint_auto_find_test_dependencies() here.
  find_package(ament_cmake_copyright REQUIRED)
  find_package(ament_cmake_flake8 REQUIRED)
  find_package(ament_cmake_pep257 REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)

  set(AMENT_LINT_PORTED
    "${CMAKE_CURRENT_SOURCE_DIR}/launch/atlas.launch.py"
    "${CMAKE_CURRENT_SOURCE_DIR}/test/test_atlas_launch.py"
  )
  ament_copyright(${AMENT_LINT_PORTED})
  ament_flake8(${AMENT_LINT_PORTED})
  ament_pep257(${AMENT_LINT_PORTED})

  # test/CMakeLists.txt, test/CheckDisplayCapabilities.cmake, and
  # scripts/CMakeLists.txt are all leftover ROS1/catkin build files for
  # the not-yet-ported rostest suite -- not referenced by this
  # CMakeLists.txt at all (no add_subdirectory), left in place
  # untouched, excluded from lint the same way the un-ported C++ files
  # are in drcsim_gazebo_ros_plugins/CMakeLists.txt.
  ament_lint_cmake(
    EXCLUDE
    "${CMAKE_CURRENT_SOURCE_DIR}/test/CMakeLists.txt"
    "${CMAKE_CURRENT_SOURCE_DIR}/test/CheckDisplayCapabilities.cmake"
    "${CMAKE_CURRENT_SOURCE_DIR}/scripts/CMakeLists.txt"
  )

  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_atlas_launch.py TIMEOUT 180)
endif()

ament_package()
