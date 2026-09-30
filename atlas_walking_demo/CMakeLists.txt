cmake_minimum_required(VERSION 3.8)
project(atlas_walking_demo)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/balance_controller.py
  scripts/free_walk.py
  scripts/gait_controller.py
  scripts/harness_gait.py
  scripts/teleop_extras.py
  scripts/walk_keyboard.py
  scripts/zmp_walk.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_copyright REQUIRED)
  find_package(ament_cmake_flake8 REQUIRED)
  find_package(ament_cmake_pep257 REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)

  set(AMENT_LINT_PORTED
    "${CMAKE_CURRENT_SOURCE_DIR}/scripts/balance_controller.py"
    "${CMAKE_CURRENT_SOURCE_DIR}/scripts/free_walk.py"
    "${CMAKE_CURRENT_SOURCE_DIR}/scripts/gait_controller.py"
    "${CMAKE_CURRENT_SOURCE_DIR}/scripts/harness_gait.py"
    "${CMAKE_CURRENT_SOURCE_DIR}/scripts/teleop_extras.py"
    "${CMAKE_CURRENT_SOURCE_DIR}/scripts/walk_keyboard.py"
    "${CMAKE_CURRENT_SOURCE_DIR}/scripts/zmp_walk.py"
    "${CMAKE_CURRENT_SOURCE_DIR}/test/test_balance_controller.py"
    "${CMAKE_CURRENT_SOURCE_DIR}/test/test_gait_controller.py"
    "${CMAKE_CURRENT_SOURCE_DIR}/test/test_harness_gait.py"
    "${CMAKE_CURRENT_SOURCE_DIR}/test/test_teleop_extras.py"
    "${CMAKE_CURRENT_SOURCE_DIR}/test/test_zmp_walk.py"
  )
  ament_copyright(${AMENT_LINT_PORTED})
  ament_flake8(${AMENT_LINT_PORTED})
  ament_pep257(${AMENT_LINT_PORTED})
  # See drcsim_gazebo/CMakeLists.txt's note on why this takes a path list,
  # not an EXCLUDE keyword.
  ament_lint_cmake("${CMAKE_CURRENT_SOURCE_DIR}/CMakeLists.txt")

  ament_add_pytest_test(test_balance_controller test/test_balance_controller.py)
  ament_add_pytest_test(test_gait_controller test/test_gait_controller.py)
  ament_add_pytest_test(test_harness_gait test/test_harness_gait.py)
  ament_add_pytest_test(test_teleop_extras test/test_teleop_extras.py)
  ament_add_pytest_test(test_zmp_walk test/test_zmp_walk.py)
endif()

ament_package()
