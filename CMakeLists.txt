cmake_minimum_required(VERSION 2.8.3)

project(laser_lines)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  message_generation
  tf
  nav_msgs
  laser_geometry
)
find_package(Boost REQUIRED COMPONENTS system filesystem thread)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
#find_package(YAML_CPP REQUIRED)


catkin_python_setup()

## Add include directories
include_directories(include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
#generate_dynamic_reconfigure_options(cfg/CalibrateAngular.cfg cfg/CalibrateLinear.cfg)

#generate_dynamic_reconfigure_options(cfg/CalibrateAngular.cfg cfg/CalibrateLinear.cfg)
#

add_message_files(
   FILES
   LaserLine.msg
   LaserLines.msg
)
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

#catkin_package(CATKIN_DEPENDS dynamic_reconfigure)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES beginner_tutorials
  CATKIN_DEPENDS roscpp std_msgs message_runtime geometry_msgs dynamic_reconfigure
#  DEPENDS system_lib
  DEPENDS
    Boost
    Eigen3
    PCL
)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)
if (${YAML_CPP_VERSION} VERSION_GREATER 0.4)
    add_definitions(-DHAVE_YAMLCPP_0_5)
endif()



add_executable(laser_lines src/laser_lines.cpp)
target_link_libraries(laser_lines ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${Eigen3_LIBRARIES} ${PCL_LIBRARIES})
set_property(TARGET laser_lines PROPERTY COMPILE_DEFINITION -DBOOST_SIGNALS_NO_DEPRECATION_WARNING)

add_executable(landmark_frame src/landmark_frame.cpp)
target_link_libraries(landmark_frame ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${Eigen3_LIBRARIES} ${PCL_LIBRARIES}
        ${YAML_CPP_LIBRARIES})

# Deps to the messages
add_dependencies(laser_lines ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(landmark_frame ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

