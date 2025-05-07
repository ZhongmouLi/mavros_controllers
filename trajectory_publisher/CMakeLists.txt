cmake_minimum_required(VERSION 2.8.3)
project(trajectory_generator)

add_definitions(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  mavros
  mavros_extras
  mavros_msgs
  mavlink
  controller_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy std_msgs nav_msgs geometry_msgs tf mav_planning_msgs controller_msgs mavros_msgs
)

include_directories(
  include
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)
############
# LIBRARIES #
############
add_library(${PROJECT_NAME}_base
        lib/base/trajectory.cpp
        lib/base/polynomialtrajectory.cpp
        lib/base/shapetrajectory.cpp
)
target_compile_options(${PROJECT_NAME}_base PRIVATE -std=c++17 -Wall -Wextra)

target_link_libraries(${PROJECT_NAME}_base  ${catkin_LIBRARIES})

add_library(${PROJECT_NAME}_lib
        lib/trajectory_generator.cpp
)
target_compile_options(${PROJECT_NAME}_lib PRIVATE -std=c++17 -Wall -Wextra)

target_link_libraries(${PROJECT_NAME}_lib  ${catkin_LIBRARIES} ${PROJECT_NAME}_base)

add_library(${PROJECT_NAME}_ros
        lib/trajectory_generator_ros.cpp
)
target_compile_options(${PROJECT_NAME}_ros PRIVATE -std=c++17 -Wall -Wextra)

target_link_libraries(${PROJECT_NAME}_ros  ${catkin_LIBRARIES} ${PROJECT_NAME}_lib)

############
# BINARIES #
############
add_executable(${PROJECT_NAME}_node
        src/trajectoryPublisher_node.cpp
)
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${PROJECT_NAME}_ros)

##########
# EXPORT #
##########
#cs_install()
#cs_export()