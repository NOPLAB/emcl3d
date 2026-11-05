cmake_minimum_required(VERSION 3.16)
project(emcl3d LANGUAGES C CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenMP REQUIRED COMPONENTS CXX)
find_package(PCL REQUIRED COMPONENTS common io filters kdtree)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/GlobalLocalization.action"
  DEPENDENCIES geometry_msgs)
rosidl_get_typesupport_target(emcl3d_typesupport ${PROJECT_NAME} rosidl_typesupport_cpp)
add_library(emcl3d_core src/map.cpp src/map_index.cpp src/particle_filter.cpp src/global_localizer.cpp)
target_include_directories(emcl3d_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS})
target_link_libraries(emcl3d_core PUBLIC Eigen3::Eigen ${PCL_LIBRARIES} OpenMP::OpenMP_CXX)
target_compile_options(emcl3d_core PRIVATE -Wall -Wextra -Wpedantic)
target_compile_options(emcl3d_core PRIVATE $<$<CONFIG:RelWithDebInfo>:-O3>)
option(EMCL3D_ENABLE_VULKAN "Build the optional Vulkan distance backend" OFF)
if(EMCL3D_ENABLE_VULKAN)
  find_package(Vulkan REQUIRED)
  find_program(EMCL3D_GLSLANG_VALIDATOR glslangValidator)
  if(NOT EMCL3D_GLSLANG_VALIDATOR)
    message(FATAL_ERROR "Vulkan backend requires glslangValidator (glslang-tools)")
  endif()
  set(shader_dir "${CMAKE_CURRENT_BINARY_DIR}/generated")
  file(MAKE_DIRECTORY "${shader_dir}")
  set(shader_binary "${shader_dir}/map_distance.spv")
  set(shader_header "${shader_dir}/map_distance_spirv.hpp")
  add_custom_command(OUTPUT "${shader_header}"
    COMMAND "${EMCL3D_GLSLANG_VALIDATOR}" -V --target-env vulkan1.1
      -o "${shader_binary}" "${CMAKE_CURRENT_SOURCE_DIR}/shaders/map_distance.comp"
    COMMAND "${CMAKE_COMMAND}" "-DINPUT=${shader_binary}" "-DOUTPUT=${shader_header}"
      -P "${CMAKE_CURRENT_SOURCE_DIR}/cmake/embed_spirv.cmake"
    DEPENDS shaders/map_distance.comp cmake/embed_spirv.cmake
    BYPRODUCTS "${shader_binary}"
    VERBATIM)
  target_sources(emcl3d_core PRIVATE src/vulkan_context.cpp src/vulkan_distance.cpp "${shader_header}")
  target_include_directories(emcl3d_core PRIVATE "${shader_dir}")
  target_compile_definitions(emcl3d_core PRIVATE EMCL3D_ENABLE_VULKAN=1)
  target_link_libraries(emcl3d_core PRIVATE Vulkan::Vulkan)
  ament_export_dependencies(Vulkan)
endif()
add_executable(emcl3d_node src/emcl3d_node.cpp)
target_link_libraries(emcl3d_node emcl3d_core "${emcl3d_typesupport}")
ament_target_dependencies(emcl3d_node rclcpp rclcpp_action nav_msgs geometry_msgs sensor_msgs std_msgs
  diagnostic_msgs tf2 tf2_ros tf2_geometry_msgs pcl_conversions)
target_compile_options(emcl3d_node PRIVATE -Wall -Wextra -Wpedantic)
option(BUILD_BENCHMARKS "Build the deterministic two-thread core benchmark" OFF)
if(BUILD_BENCHMARKS)
  add_executable(benchmark_core benchmark/core.cpp)
  target_link_libraries(benchmark_core emcl3d_core)
endif()
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_global_localizer test/test_global_localizer.cpp)
  target_link_libraries(test_global_localizer emcl3d_core)
  target_compile_definitions(test_global_localizer PRIVATE
    TEST_MAP_PATH="${CMAKE_CURRENT_SOURCE_DIR}/test/map.pcd")
  ament_add_gtest(test_particle_filter test/test_particle_filter.cpp)
  target_link_libraries(test_particle_filter emcl3d_core)
  target_compile_definitions(test_particle_filter PRIVATE
    TEST_MAP_PATH="${CMAKE_CURRENT_SOURCE_DIR}/test/map.pcd")
  if(EMCL3D_ENABLE_VULKAN)
    target_compile_definitions(test_particle_filter PRIVATE EMCL3D_ENABLE_VULKAN=1)
    ament_add_gtest(test_vulkan test/test_vulkan.cpp)
    target_link_libraries(test_vulkan emcl3d_core)
    target_compile_definitions(test_vulkan PRIVATE
      TEST_MAP_PATH="${CMAKE_CURRENT_SOURCE_DIR}/test/map.pcd")
  endif()
  ament_add_gtest(test_ros_node test/test_ros_node.cpp TIMEOUT 90
    APPEND_LIBRARY_DIRS "${CMAKE_CURRENT_BINARY_DIR}")
  target_link_libraries(test_ros_node emcl3d_core "${emcl3d_typesupport}")
  ament_target_dependencies(test_ros_node rclcpp rclcpp_action nav_msgs geometry_msgs sensor_msgs std_msgs
    diagnostic_msgs tf2 tf2_ros tf2_msgs tf2_geometry_msgs pcl_conversions)
  target_compile_definitions(test_ros_node PRIVATE
    TEST_MAP_PATH="${CMAKE_CURRENT_SOURCE_DIR}/test/map.pcd"
    TEST_CONFIG_PATH="${CMAKE_CURRENT_SOURCE_DIR}/config/emcl3d.yaml")
  if(EMCL3D_ENABLE_VULKAN)
    target_compile_definitions(test_ros_node PRIVATE EMCL3D_ENABLE_VULKAN=1)
  endif()
endif()
install(TARGETS emcl3d_core emcl3d_node
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(FILES LICENSE COPYING DESTINATION share/${PROJECT_NAME})
ament_export_include_directories(include)
ament_export_libraries(emcl3d_core)
ament_export_dependencies(Eigen3 PCL OpenMP)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
