#ifndef USB_CAMERA_NODE_H_
#define USB_CAMERA_NODE_H_

#include "usb_camera/ros_usb_camera.h"
#include "usb_camera/UsbCameraDynConfig.h"
#include "camera_base/camera_node_base.h"

namespace usb_camera {

class UsbCameraNode : public CameraNodeBase<usb_camera::UsbCameraDynConfig> {
 public:
  UsbCameraNode(const ros::NodeHandle& nh)
      : CameraNodeBase{nh}, ros_usb_camera_{nh} {}

  virtual void Acquire() override;
  virtual void Setup(usb_camera::UsbCameraDynConfig& config) override;

 private:
  usb_camera::RosUsbCamera ros_usb_camera_;
};

}  // namespace usb_camera

#endif  // USB_CAMERA_NODE_H_
