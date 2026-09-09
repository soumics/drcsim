cmake_minimum_required(VERSION 3.8)
project(joint_commands_gui)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/joint_command_gui.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_copyright REQUIRED)
  find_package(ament_cmake_flake8 REQUIRED)
  find_package(ament_cmake_pep257 REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)

  set(AMENT_LINT_PORTED
    "${CMAKE_CURRENT_SOURCE_DIR}/scripts/joint_command_gui.py"
    "${CMAKE_CURRENT_SOURCE_DIR}/test/test_parse_free_joints.py"
  )
  ament_copyright(${AMENT_LINT_PORTED})
  ament_flake8(${AMENT_LINT_PORTED})
  ament_pep257(${AMENT_LINT_PORTED})
  # See drcsim_gazebo/CMakeLists.txt's note on why this takes a path list,
  # not an EXCLUDE keyword.
  ament_lint_cmake("${CMAKE_CURRENT_SOURCE_DIR}/CMakeLists.txt")

  ament_add_pytest_test(test_parse_free_joints test/test_parse_free_joints.py)
endif()

ament_package()
