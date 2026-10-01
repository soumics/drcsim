cmake_minimum_required(VERSION 3.8)
project(atlas_learning)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/atlas_env.py
  scripts/build_mjcf.py
  scripts/evaluate_mj.py
  scripts/policy_io.py
  scripts/policy_stand.py
  scripts/train_push.py
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY models DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_copyright REQUIRED)
  find_package(ament_cmake_flake8 REQUIRED)
  find_package(ament_cmake_pep257 REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)

  file(GLOB LINTED
    "${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.py"
    "${CMAKE_CURRENT_SOURCE_DIR}/test/*.py")
  ament_copyright(${LINTED})
  ament_flake8(${LINTED})
  ament_pep257(${LINTED})
  ament_lint_cmake("${CMAKE_CURRENT_SOURCE_DIR}/CMakeLists.txt")

  ament_add_pytest_test(test_policy_io test/test_policy_io.py)
endif()

ament_package()
