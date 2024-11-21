cmake_minimum_required(VERSION 3.14)
project(visual_localization)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON CACHE INTERNAL "")
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_options(-g -O3)  # Common compile options including dependencies
set(COMMON_COMPILE_FLAGS -g -Wall -Wextra -Wpedantic -Werror)
set(PROFILER_COMPILE_FLAGS -fprofile-arcs -ftest-coverage)

# add_compile_options(-g -Wall -Wextra -Wpedantic -Werror -fprofile-arcs -ftest-coverage)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(fmt REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 3.3 REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)

# COLMAP option. Build fails without this setup,
option(UNINSTALL_ENABLED OFF)
add_subdirectory(colmap)

# --------------------- LIBRARIES ---------------------
set(SRC_FILES
  src/bitmap.cc
  src/camera_parameter.cc
  src/conversion.cc
  src/cv_colmap.cc
  src/cv_eigen.cc
  src/eigen_utils.cc
  src/feature.cc
  src/pnp.cc
  src/projection.cc
  src/ros/header_message.cc
  src/ros/image_message.cc
  src/ros/point32_message.cc
  src/ros/pointcloud_message.cc
  src/ros/point_message.cc
  src/ros/pose_message.cc
  src/ros/pose_stamped_message.cc
  src/ros/quaternion_message.cc
  src/triangulation.cc
  src/vector_search.cc)

add_library(vision_library ${SRC_FILES})
target_compile_options(vision_library PRIVATE
  ${COMMON_COMPILE_FLAGS}
  ${PROFILER_COMPILE_FLAGS})

target_include_directories(vision_library PRIVATE include)

target_link_libraries(vision_library Eigen3::Eigen)
target_link_libraries(vision_library gcov)

target_include_directories(vision_library PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(vision_library ${OpenCV_LIBS})

target_include_directories(vision_library PUBLIC colmap/src)
target_compile_options(vision_library PRIVATE -fPIC)
target_link_libraries(vision_library colmap)

set(ROS_DEPENDENT_MODULES header_message image_message point_message pointcloud_message pose_message
  pose_stamped_message quaternion_message)

target_include_directories(vision_library PUBLIC
   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
   $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(vision_library cv_bridge geometry_msgs sensor_msgs std_msgs)

# --------------------- TESTS ---------------------
include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
)

# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

enable_testing()

include(GoogleTest)


set(TEST_FILES
  tests/bitmap.cc
  tests/camera_parameter.cc
  tests/conversion.cc
  tests/cv_colmap.cc
  tests/cv_eigen.cc
  tests/eigen_utils.cc
  tests/feature.cc
  tests/pnp.cc
  tests/projection.cc
  tests/ros/header_message.cc
  tests/ros/image_message.cc
  tests/ros/point32_message.cc
  tests/ros/pointcloud_message.cc
  tests/ros/point_message.cc
  tests/ros/pose_message.cc
  tests/ros/pose_stamped_message.cc
  tests/ros/quaternion_message.cc
  tests/triangulation.cc
  tests/vector_search.cc)

foreach(filename ${TEST_FILES})
  get_filename_component(module_name ${filename} NAME_WE)

  add_executable(test_${module_name} ${filename})
  target_compile_options(test_${module_name} PRIVATE
    ${COMMON_COMPILE_FLAGS}
    ${PROFILER_COMPILE_FLAGS})

  target_include_directories(test_${module_name} PRIVATE include)

  target_link_libraries(test_${module_name} vision_library)
  target_link_libraries(test_${module_name} GTest::gtest_main GTest::gmock_main)
  target_link_libraries(test_${module_name} gcov)
  gtest_discover_tests(test_${module_name})
endforeach()


# --------------------- ROS APPS ---------------------

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(relocalization apps/relocalization.cc)
target_compile_options(relocalization PRIVATE ${COMMON_COMPILE_FLAGS})

target_include_directories(relocalization PRIVATE ${PCL_INCLUDE_DIRS})

target_link_libraries(relocalization vision_library)
target_link_libraries(relocalization ${PCL_LIBRARIES})
target_link_libraries(relocalization fmt)

target_include_directories(relocalization PRIVATE colmap/src)
target_compile_options(relocalization PRIVATE -fPIC)
target_link_libraries(relocalization colmap)

ament_target_dependencies(relocalization cv_bridge geometry_msgs rclcpp sensor_msgs std_msgs pcl_conversions)

install(TARGETS relocalization DESTINATION lib/${PROJECT_NAME})

ament_auto_package(INSTALL_TO_SHARE launch config)
