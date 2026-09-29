cmake_minimum_required(VERSION 3.8)
project(atlas_svh_hands)

find_package(ament_cmake REQUIRED)

install(DIRECTORY robots config DESTINATION share/${PROJECT_NAME})

# gz-sim resolves the SVH's package://schunk_svh_description meshes through
# GZ_SIM_RESOURCE_PATH, and that separately fetched package ships no hook of
# its own: point at its share dir (the default isolated install layout,
# install/<package>/share, next to this package's prefix).
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_svh_mounts test/test_svh_mounts.py)
endif()

ament_package()
