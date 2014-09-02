#include "usb_camera/usb_camera_node.h"

namespace usb_camera {

void UsbCameraNode::Acquire() {
  while (is_acquire() && ros::ok()) {
    ros::Time time = ros::Time::now();
    ros_usb_camera_.Publish(time);
    Sleep();
  }
}

void UsbCameraNode::Setup(usb_camera::UsbCameraDynConfig& config) {
  ros_usb_camera_.set_fps(config.fps);
}

}  // namespace usb_camera
