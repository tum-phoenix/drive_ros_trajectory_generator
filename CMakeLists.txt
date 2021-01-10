cmake_minimum_required(VERSION 2.8.3)
project(drive_ros_trajectory_generator)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c99")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Drestrict=__restrict__") #use restrict in c++, I wonder if this works
SET(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  drive_ros_msgs
  drive_ros_uavcan
  dynamic_reconfigure
)

## System dependencies are found with CMake's conventions
find_package(CPPAD)

generate_dynamic_reconfigure_options(
    cfg/TrajectoryLineCreation.cfg
)



# enable to get visual display of the obstacle
#option(SUBSCRIBE_DEBUG "Add debug output" ON)
#add_definitions(-std=c++11)



catkin_package(
        INCLUDE_DIRS include
        CATKIN_DEPENDS roscpp std_msgs drive_ros_msgs drive_ros_uavcan dynamic_reconfigure
        DEPENDS CPPAD
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${CPPAD_INCLUDE_DIRS}
)


################################################################################
# Trajectory Line Creator Node
################################################################################
add_executable(trajectory_line_creator_node
        # src/trajectory_line_creator.cpp
        src/trajectory_line_creator_node.cpp
        src/trajectory_creator_base.cpp
        src/trajectory_creator_polynomial.cpp
        )

add_dependencies(trajectory_line_creator_node ${drive_ros_msgs_generate_messages_cpp}
    ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(trajectory_line_creator_node
        ${catkin_LIBRARIES}
        )

install(TARGETS trajectory_line_creator_node
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

################################################################################
# Install other files
################################################################################
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".svn" EXCLUDE
        )
