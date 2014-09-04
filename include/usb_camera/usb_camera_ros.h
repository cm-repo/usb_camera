#ifndef ROS_USB_CAMERA_H_
#define ROS_USB_CAMERA_H_

#include "usb_camera/usb_camera.h"
#include "camera_base/camera_ros_base.h"

namespace usb_camera {

class UsbCameraRos : public CameraRosBase {
 public:
  UsbCameraRos(const ros::NodeHandle &nh)
      : CameraRosBase{nh},
        usb_camera_{identifier().empty() ? 0 : std::stoi(identifier())} {
    SetHardwareId(usb_camera_.device());
  }

  virtual bool Grab(const sensor_msgs::ImagePtr &image_msg) override;

 private:
  UsbCamera usb_camera_;
};

}  // namespace usb_camera

#endif  // ROS_USB_CAMERA_H_
