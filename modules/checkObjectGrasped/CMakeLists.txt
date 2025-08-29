#
# Copyright (C) 2016 iCub Facility - IIT Istituto Italiano di Tecnologia
# Author: Raffaele Colombo raffaele.colombo@iit.it
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
#

project(checkObjectGrasped)

file(GLOB folder_source *.cpp)
file(GLOB folder_header *.h)

source_group("Source Files" FILES ${folder_source})
source_group("Header Files" FILES ${folder_header})

# Find YARP
if(NAVIGATION_USE_ROS2)
    find_package(YARP REQUIRED COMPONENTS sig dev os math)
else()
    find_package(YARP REQUIRED COMPONENTS sig dev os math rosmsg)
endif()

# Find ROS2 packages
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(${ICUB_INCLUDE_DIRS})
add_executable(${PROJECT_NAME} ${folder_source} ${folder_header})

# Link libraries
target_link_libraries(${PROJECT_NAME} 
    ${YARP_LIBRARIES}
    rclcpp::rclcpp
    tf2_ros::tf2_ros
    tf2_geometry_msgs::tf2_geometry_msgs
    ${geometry_msgs_TARGETS}
)

set_property(TARGET checkObjectGrasped PROPERTY FOLDER "Modules")
install(TARGETS ${PROJECT_NAME} DESTINATION bin)
