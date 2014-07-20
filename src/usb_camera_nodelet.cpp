#include "usb_camera/usb_camera_nodelet.h"

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_DECLARE_CLASS(usb_camera, UsbCameraNodelet,
                        usb_camera::UsbCameraNodelet, nodelet::Nodelet)

namespace usb_camera {

UsbCameraNodelet::UsbCameraNodelet() : nodelet::Nodelet() {}

UsbCameraNodelet::~UsbCameraNodelet() {
  usb_camera_->End();
}

void UsbCameraNodelet::onInit() {
  usb_camera_.reset(new UsbCamera(getPrivateNodeHandle()));
  usb_camera_->Run();
}

}  // namespace usb_camera
