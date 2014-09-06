#include "usb_camera/usb_camera_node.h"

namespace usb_camera {

void UsbCameraNode::Acquire() {
  while (is_acquire() && ros::ok()) {
    ros::Time time = ros::Time::now();
    usb_camera_ros_.PublishCamera(time);
    Sleep();
  }
}

void UsbCameraNode::Setup(UsbCameraDynConfig& config) {
  usb_camera_ros_.set_fps(config.fps);
  usb_camera_ros_.set_color(config.color);
}

}  // namespace usb_camera
