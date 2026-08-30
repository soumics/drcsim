cmake_minimum_required(VERSION 3.8)
project(drcsim_model_resources)

find_package(ament_cmake REQUIRED)

# AtlasSimInterface shim libraries (versions 1, 2, 3): OSRF's own
# open-source stand-in implementation for BDI's proprietary
# AtlasSimInterface control library, used when the real (hardware-vendor)
# binary isn't available. The original package could optionally link a
# real prebuilt v1.1.1 binary (AtlasSimInterface_1.1.1/lib64/
# libAtlasSimInterface.so.1.1.1) instead of its shim; that file is a
# decade-plus-old stripped ELF .so with real glibc/ABI compatibility risk
# on a modern system, and versions 2/3 never had a binary option to begin
# with (shim-only). This port always uses the shim for all three
# versions, for a build with no proprietary-binary-linking risk at all.
# The v1 binary file is left untouched in the source tree, just unused.
function(add_atlas_sim_interface_shim version)
  set(target_name "AtlasSimInterface${version}")
  set(source_dir "${CMAKE_CURRENT_SOURCE_DIR}/AtlasSimInterface_${ARGN}")

  add_library(${target_name} SHARED
    "${source_dir}/src/AtlasSimInterface.cc")
  target_include_directories(${target_name} PUBLIC
    "$<BUILD_INTERFACE:${source_dir}/include/AtlasSimInterface_${ARGN}>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}/AtlasSimInterface_${ARGN}>")
  set_target_properties(${target_name} PROPERTIES SOVERSION ${ARGN})

  install(TARGETS ${target_name}
    EXPORT export_${PROJECT_NAME}
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
  )
  install(DIRECTORY "${source_dir}/include/AtlasSimInterface_${ARGN}"
    DESTINATION include/${PROJECT_NAME}
  )
endfunction()

add_atlas_sim_interface_shim(1 1.1.1)
add_atlas_sim_interface_shim(2 2.10.2)
add_atlas_sim_interface_shim(3 3.0.2)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

# Gazebo worlds, models, and shared media -- installed wholesale (no
# per-model/per-world CMake logic needed, they were all plain
# install(FILES ...) in the original).
install(DIRECTORY worlds DESTINATION share/${PROJECT_NAME})
install(DIRECTORY gazebo_models DESTINATION share/${PROJECT_NAME})
install(DIRECTORY media DESTINATION share/${PROJECT_NAME})

# Let Gazebo Harmonic (gz-sim) find these worlds/models via
# GZ_SIM_RESOURCE_PATH.
ament_environment_hooks(
  "${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in"
)

if(BUILD_TESTING)
  # Not using ament_lint_auto_find_test_dependencies() here: the
  # AtlasSimInterface_*/ shim source is OSRF's own legacy vendored code
  # (not maintained under this repo's ROS 2 style), so the C++ linters
  # need an explicit EXCLUDE rather than the blanket auto-discovery.
  set(AMENT_LINT_EXCLUDE
    "${CMAKE_CURRENT_SOURCE_DIR}/AtlasSimInterface_1.1.1"
    "${CMAKE_CURRENT_SOURCE_DIR}/AtlasSimInterface_2.10.2"
    "${CMAKE_CURRENT_SOURCE_DIR}/AtlasSimInterface_3.0.2"
  )

  find_package(ament_cmake_copyright REQUIRED)
  ament_copyright(EXCLUDE ${AMENT_LINT_EXCLUDE})

  find_package(ament_cmake_cppcheck REQUIRED)
  ament_cppcheck(EXCLUDE ${AMENT_LINT_EXCLUDE})

  find_package(ament_cmake_cpplint REQUIRED)
  ament_cpplint(EXCLUDE ${AMENT_LINT_EXCLUDE})

  find_package(ament_cmake_uncrustify REQUIRED)
  ament_uncrustify(EXCLUDE ${AMENT_LINT_EXCLUDE})

  find_package(ament_cmake_lint_cmake REQUIRED)
  ament_lint_cmake()

  find_package(ament_cmake_xmllint REQUIRED)
  ament_xmllint()

  find_package(ament_cmake_flake8 REQUIRED)
  ament_flake8()

  find_package(ament_cmake_pep257 REQUIRED)
  ament_pep257()

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_sdf_files_check test/test_sdf_files_check.py)
endif()

ament_package()
