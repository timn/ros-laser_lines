cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

find_package(PkgConfig)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)
pkg_check_modules(EIGEN REQUIRED eigen3)

rosbuild_genmsg()
#rosbuild_gensrv()

add_definitions(-Wno-unused-local-typedefs)
include_directories(${YAML_CPP_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIRS})

if (${YAML_CPP_VERSION} VERSION_GREATER 0.4)
  add_definitions(-DHAVE_YAMLCPP_0_5)
endif()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
rosbuild_add_executable(laser_lines src/laser_lines.cpp)
set_property(TARGET laser_lines PROPERTY COMPILE_DEFINITION -DBOOST_SIGNALS_NO_DEPRECATION_WARNING)
#target_link_libraries(example ${PROJECT_NAME})

rosbuild_add_executable(landmark_frame src/landmark_frame.cpp)
target_link_libraries(landmark_frame ${YAML_CPP_LIBRARIES} ${EIGEN_LIBRARIES})
