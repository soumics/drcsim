cmake_minimum_required(VERSION 3.8)
project(sandia_hand_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  config launch meshes urdf robots
  DESTINATION share/${PROJECT_NAME}
)

install(FILES
  model.config
  dummy.urdf
  DESTINATION share/${PROJECT_NAME}
)

# Let Gazebo Harmonic (gz-sim) find this package's models via
# GZ_SIM_RESOURCE_PATH, same role the old GAZEBO_MODEL_PATH hook played.
ament_environment_hooks(
  "${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_xacro_parses test/test_xacro_parses.py)
endif()

ament_package()
