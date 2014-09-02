#include <ros/ros.h>

#include "usb_camera/usb_camera_node.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "usb_camera");
  ros::NodeHandle nh("~");

  try {
    usb_camera::UsbCameraNode usb_camera_node(nh);
    usb_camera_node.Run();
    ros::spin();
    usb_camera_node.End();
  }
  catch (const std::exception &e) {
    ROS_ERROR_STREAM(nh.getNamespace() << e.what());
    return -1;
  }
}
