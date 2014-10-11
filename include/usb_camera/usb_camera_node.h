#ifndef USB_CAMERA_NODE_H_
#define USB_CAMERA_NODE_H_

#include "usb_camera/usb_camera_ros.h"
#include "usb_camera/UsbCameraDynConfig.h"
#include "camera_base/camera_node_base.h"

namespace usb_camera {

class UsbCameraNode : public camera_base::CameraNodeBase<UsbCameraDynConfig> {
 public:
  UsbCameraNode(const ros::NodeHandle& nh)
      : CameraNodeBase{nh}, usb_camera_ros_{nh} {}

  virtual void Acquire() override;
  virtual void Setup(UsbCameraDynConfig& config) override;

 private:
  UsbCameraRos usb_camera_ros_;
};

}  // namespace usb_camera

#endif  // USB_CAMERA_NODE_H_
