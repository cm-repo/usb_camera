#ifndef USB_CAMERA_USB_CAMERA_NODELET_H_
#define USB_CAMERA_USB_CAMERA_NODELET_H_

#include <nodelet/nodelet.h>
#include "usb_camera/usb_camera.h"

namespace usb_camera {

class UsbCameraNodelet : public nodelet::Nodelet {
 public:
  UsbCameraNodelet();
  ~UsbCameraNodelet();

  UsbCameraNodelet(const UsbCameraNodelet&) = delete;
  UsbCameraNodelet &operator=(const UsbCameraNodelet&) = delete;

  virtual void onInit();

 private:
  std::unique_ptr<usb_camera::UsbCamera> usb_camera_;
};

}  // namespace usb_camera

#endif  // USB_CAMERA_USB_CAMERA_NODELET_H_
