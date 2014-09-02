cmake_minimum_required(VERSION 2.8.3)
project(usb_camera)

add_definitions(-std=c++11 -Wall)

find_package(catkin REQUIRED COMPONENTS
    roscpp nodelet image_transport
    sensor_msgs camera_info_manager
    dynamic_reconfigure diagnostic_updater
    )
find_package(OpenCV)

generate_dynamic_reconfigure_options(cfg/UsbCameraDyn.cfg)

catkin_package(
    #  INCLUDE_DIRS include
    #  LIBRARIES usb_camera
    #  CATKIN_DEPENDS roscpp
    #  DEPENDS system_lib
    )

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    )

add_library(${PROJECT_NAME}
    src/ros_usb_camera.cpp
    src/usb_camera_node.cpp
    src/usb_camera_nodelet.cpp
    )
target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    )

# Node
add_executable(${PROJECT_NAME}_node
    src/usb_camera_main.cpp
    )
target_link_libraries(${PROJECT_NAME}_node
    ${PROJECT_NAME}
    )

add_dependencies(${PROJECT_NAME}
    ${catkin_EXPORTED_TARGETS}
    ${PROJECT_NAME}_gencfg
    ${PROJECT_NAME}_gencpp
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    )
