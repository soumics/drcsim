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

add_library(DRCVehiclePlugin SHARED src/DRCVehiclePlugin.cc)
target_link_libraries(DRCVehiclePlugin
  gz-sim8::gz-sim8
  gz-plugin2::register
  gz-math7::gz-math7
  gz-common5::gz-common5
)
install(TARGETS DRCVehiclePlugin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

add_library(DRCBuildingPlugin SHARED src/DRCBuildingPlugin.cc)
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

# Let gz-sim find these plugin libraries at runtime.
ament_environment_hooks(
  "${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in"
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.hh"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
