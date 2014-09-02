#include "usb_camera/usb_camera_node.h"

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

namespace usb_camera {

class UsbCameraNodelet : public nodelet::Nodelet {
 public:
  UsbCameraNodelet() : nodelet::Nodelet() {}
  ~UsbCameraNodelet() { usb_camera_node_->End(); }

  virtual void onInit() {
    try {
      usb_camera_node_.reset(new UsbCameraNode(getPrivateNodeHandle()));
      usb_camera_node_->Run();
    }
    catch (const std::exception &e) {
      NODELET_ERROR_STREAM(e.what());
    }
  }

 private:
  std::unique_ptr<UsbCameraNode> usb_camera_node_;
};

PLUGINLIB_DECLARE_CLASS(usb_camera, UsbCameraNodelet,
                        usb_camera::UsbCameraNodelet, nodelet::Nodelet)

}  // namespace usb_camera
