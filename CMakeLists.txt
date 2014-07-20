cmake_minimum_required(VERSION 2.8.3)
project(usb_camera)

add_definitions("-std=c++11 -Wall")

find_package(catkin REQUIRED
    COMPONENTS roscpp image_transport
               cv_bridge camera_info_manager
               sensor_msgs dynamic_reconfigure
               nodelet
    )
find_package(OpenCV)

generate_dynamic_reconfigure_options(cfg/UsbCameraDyn.cfg)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES usb_camera
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# camera node
add_library(${PROJECT_NAME}
    src/usb_camera.cpp
    )
target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    )
add_executable(${PROJECT_NAME}_node
    src/usb_camera_node.cpp
    )
target_link_libraries(${PROJECT_NAME}_node
    ${catkin_LIBRARIES}
    ${PROJECT_NAME}
    )

# camera nodelet
add_library(${PROJECT_NAME}_nodelet
    src/usb_camera_nodelet.cpp
    )
target_link_libraries(${PROJECT_NAME}_nodelet
    ${PROJECT_NAME}
    )

add_dependencies(${PROJECT_NAME}
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${PROJECT_NAME}_gencfg
    )
