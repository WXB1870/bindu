cmake_minimum_required(VERSION 3.16)
project(bindu_execution_native LANGUAGES CXX)

# The Python extension builds these same sources. Standalone servo consumers
# may add_subdirectory this directory and link bindu::execution_native.
add_library(bindu_execution_native STATIC interpolation.cpp adaptive_stream.cpp)
add_library(bindu::execution_native ALIAS bindu_execution_native)
set_target_properties(bindu_execution_native PROPERTIES EXPORT_NAME execution_native
  POSITION_INDEPENDENT_CODE ON)
target_compile_features(bindu_execution_native PUBLIC cxx_std_17)
target_include_directories(bindu_execution_native PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<INSTALL_INTERFACE:include/bindu_execution>)

install(TARGETS bindu_execution_native EXPORT bindu_execution_nativeTargets ARCHIVE DESTINATION lib)
install(FILES interpolation.hpp adaptive_stream.hpp curve_math.hpp DESTINATION include/bindu_execution)
install(EXPORT bindu_execution_nativeTargets NAMESPACE bindu::
  FILE bindu_execution_nativeConfig.cmake DESTINATION lib/cmake/bindu_execution_native)
