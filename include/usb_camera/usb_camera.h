#ifndef USB_CAMERA_H_
#define USB_CAMERA_H_

#include <memory>
#include <thread>

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

namespace usb_camera {

class UsbCamera {
 public:
  UsbCamera(int device) : device_(std::to_string(device)), capture_(device) {
    if (!capture_.isOpened()) {
      throw std::runtime_error(std::string("Invalid device id: ") + device_);
    }
  }

  UsbCamera(const UsbCamera&) = delete;
  UsbCamera& operator=(const UsbCamera&) = delete;

  const std::string& device() const { return device_; }
  int width() { return capture_.get(CV_CAP_PROP_FRAME_WIDTH); }
  int height() { return capture_.get(CV_CAP_PROP_FRAME_HEIGHT); }

  bool GrabImage(cv::Mat& image) { return capture_.read(image); }

 private:
  std::string device_;
  cv::VideoCapture capture_;
};

}  // namespace usb_camera

#endif  // USB_CAMERA_H_
