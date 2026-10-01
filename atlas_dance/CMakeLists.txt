cmake_minimum_required(VERSION 3.8)
project(atlas_dance)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/dance_player.py
  scripts/extract_pose.py
  scripts/retarget.py
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

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

  ament_add_pytest_test(test_retarget test/test_retarget.py)
endif()

ament_package()
