cmake_minimum_required(VERSION 3.20)
project(Puffysics VERSION 0.1.0 LANGUAGES C CXX)

# Header-only core: one INTERFACE target, include root, no sources.
# Optional modules keep their file-stem prefixes; see README.md.
option(PUFFYSICS_BUILD_TESTS "Build local headless CPU tests (when available)" OFF)
option(PUFFYSICS_ENABLE_CUDA "Enable CUDA language for device targets" OFF)
option(PUFFYSICS_BUILD_EXAMPLES "Build headless examples" OFF)
option(PUFFYSICS_BUILD_BENCHMARKS "Build CPU benchmarks" OFF)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

add_library(puffysics INTERFACE)
add_library(Puffysics::puffysics ALIAS puffysics)
target_include_directories(puffysics INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_compile_features(puffysics INTERFACE c_std_99 cxx_std_11)
if(UNIX)
  target_link_libraries(puffysics INTERFACE m)
endif()

if(PUFFYSICS_ENABLE_CUDA)
  enable_language(CUDA)
endif()

if(PUFFYSICS_BUILD_TESTS)
  if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/CMakeLists.txt")
    message(FATAL_ERROR "This source distribution does not include tests. Configure with PUFFYSICS_BUILD_TESTS=OFF.")
  endif()
  include(CTest)
  add_subdirectory(tests)
endif()

if(PUFFYSICS_BUILD_EXAMPLES)
  add_subdirectory(examples)
endif()

if(PUFFYSICS_BUILD_BENCHMARKS)
  if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/benchmarks/CMakeLists.txt")
    message(FATAL_ERROR "This source distribution does not include benchmarks. Configure with PUFFYSICS_BUILD_BENCHMARKS=OFF.")
  endif()
  add_subdirectory(benchmarks)
endif()

install(FILES puffysics.cuh b3_art.cuh b3_batch.cuh nbody.cuh nbody_fields.cuh nbody_batch.cuh nbody_tree.cuh nbody_tree_gpu.cuh nbody_rigid.cuh
  ambient.h mjcf.h stl.h dat_cloth.h dat_cloth_gpu.cuh b3_loose.cuh
  atan_approx.cuh box3d.cuh
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(FILES puffysics.cuh
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/puffysics)
install(TARGETS puffysics EXPORT PuffysicsTargets)
install(EXPORT PuffysicsTargets NAMESPACE Puffysics::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/Puffysics)
configure_package_config_file(cmake/PuffysicsConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/PuffysicsConfig.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/Puffysics)
write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/PuffysicsConfigVersion.cmake
  VERSION ${PROJECT_VERSION} COMPATIBILITY SameMajorVersion)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/PuffysicsConfig.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/PuffysicsConfigVersion.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/Puffysics)
