#ifndef USB_CAMERA_USB_CAMERA_H_
#define USB_CAMERA_USB_CAMERA_H_

#include <memory>
#include <thread>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <camera_info_manager/camera_info_manager.h>
#include <dynamic_reconfigure/server.h>

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#include "usb_camera/UsbCameraDynConfig.h"

namespace usb_camera {

struct UsbCameraConfig {
  bool color{false};
};

class UsbCamera {
 private:
  // ROS related
  ros::NodeHandle nh_;
  std::string frame_id_;
  std::unique_ptr<ros::Rate> rate_;
  image_transport::ImageTransport it_;
  image_transport::CameraPublisher camera_pub_;
  sensor_msgs::ImagePtr image_;
  sensor_msgs::CameraInfoPtr cinfo_;
  dynamic_reconfigure::Server<usb_camera::UsbCameraDynConfig> server_;

  // Video capture
  std::string label_{"\033[0;34m[ USBC]:\033[0m "};
  std::unique_ptr<cv::VideoCapture> camera_;
  int device_{0};
  bool color_{false};
  bool acquire_{false};
  typedef std::unique_ptr<std::thread> ThreadPtr;
  ThreadPtr image_thread_;

  void Connect();
  void Configure(const UsbCameraConfig &config);
  void Start();
  void Stop();
  void Disconnect();
  void AcquireImages();
  const bool IsAcquire() const { return acquire_; }

  int width();
  int height();

 public:
  UsbCamera(const ros::NodeHandle &nh);
  UsbCamera(const UsbCamera &) = delete;
  UsbCamera &operator=(const UsbCamera &) = delete;

  void Run();
  void End();
  void PublishImage(const cv::Mat &image);
  void ReconfigureCallback(usb_camera::UsbCameraDynConfig &config, int level);
};  // class Camera

}  // namespace usb_camera

#endif  // USB_CAMERA_USB_CAMERA_H_
