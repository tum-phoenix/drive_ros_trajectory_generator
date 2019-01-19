cmake_minimum_required(VERSION 2.8.3)
project(drive_ros_trajectory_generator)

#SET(CMAKE_BUILD_TYPE Release)
SET(CMAKE_BUILD_TYPE Debug)

# enable to get visual display of the obstacle
option(SUBSCRIBE_DEBUG "Add debug output" ON)
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        drive_ros_msgs
        dynamic_reconfigure
        drive_ros_uavcan
        drive_ros_image_recognition
        image_transport
        sensor_msgs
        cv_bridge
        )

if (SUBSCRIBE_DEBUG)
    add_definitions(-DSUBSCRIBE_DEBUG)
    find_package(OpenCV REQUIRED)
endif(SUBSCRIBE_DEBUG)

generate_dynamic_reconfigure_options(
    cfg/TrajectoryLineCreation.cfg
)

catkin_package(
        INCLUDE_DIRS include
        CATKIN_DEPENDS roscpp std_msgs drive_ros_msgs dynamic_reconfigure drive_ros_uavcan cv_bridge sensor_msgs image_transport drive_ros_image_recognition
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

if (SUBSCRIBE_DEBUG)
    include_directories(${OpenCV_INCLUDE_DIRS})
endif(SUBSCRIBE_DEBUG)

################################################################################
# Trajectory Line Creator Node
################################################################################
add_executable(trajectory_line_creator_node
        src/trajectory_line_creator.cpp
        src/trajectory_line_creator_node.cpp
        )

add_dependencies(trajectory_line_creator_node ${drive_ros_msgs_generate_messages_cpp}
    ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(trajectory_line_creator_node
        ${catkin_LIBRARIES}
        )

if (SUBSCRIBE_DEBUG)
    target_link_libraries(trajectory_line_creator_node
            ${OpenCV_LIBS}
            )
endif(SUBSCRIBE_DEBUG)

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

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        FILES_MATCHING PATTERN "*.launch"
        )

install(DIRECTORY config/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        FILES_MATCHING PATTERN "*.yaml"
        )
