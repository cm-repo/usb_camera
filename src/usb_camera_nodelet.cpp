#include "usb_camera/usb_camera.h"

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

namespace usb_camera {

class UsbCameraNodelet : public nodelet::Nodelet {
 public:
  UsbCameraNodelet() : nodelet::Nodelet() {}
  ~UsbCameraNodelet() { usb_camera_->End(); }

  virtual void onInit() {
    usb_camera_.reset(new UsbCamera(getPrivateNodeHandle()));
    usb_camera_->Run();
  }

 private:
  std::unique_ptr<usb_camera::UsbCamera> usb_camera_;
};

PLUGINLIB_DECLARE_CLASS(usb_camera, UsbCameraNodelet,
                        usb_camera::UsbCameraNodelet, nodelet::Nodelet)

}  // namespace usb_camera
