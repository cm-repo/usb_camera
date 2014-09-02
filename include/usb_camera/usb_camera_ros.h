#ifndef ROS_USB_CAMERA_H_
#define ROS_USB_CAMERA_H_

#include "usb_camera/usb_camera.h"
#include "camera_base/camera_ros_base.h"

namespace usb_camera {

class UsbCameraRos : public CameraRosBase {
 public:
  UsbCameraRos(const ros::NodeHandle &nh) : CameraRosBase{nh} {
    std::string device;
    nh.param<std::string>("device", device, "0");
    usb_camera_.reset(new usb_camera::UsbCamera{std::stoi(device)});
  }

  virtual bool Grab(const sensor_msgs::ImagePtr &image_msg) override;

 private:
  std::unique_ptr<UsbCamera> usb_camera_;
};

}  // namespace usb_camera

#endif  // ROS_USB_CAMERA_H_
