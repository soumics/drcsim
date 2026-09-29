cmake_minimum_required(VERSION 3.8)
project(atlas_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  urdf meshes meshes_v3 meshes_v4 meshes_unplugged materials robots raw
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
  ament_add_pytest_test(test_hand_mounts test/test_hand_mounts.py)
endif()

ament_package()
