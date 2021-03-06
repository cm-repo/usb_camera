#include "usb_camera/usb_camera_ros.h"

namespace usb_camera {

bool UsbCameraRos::Grab(const sensor_msgs::ImagePtr &image_msg,
                        const sensor_msgs::CameraInfoPtr &cinfo_msg) {
  cv::Mat image;
  if (!usb_camera_.GrabImage(image)) {
    return false;
  }
  // Color option
  if (color() && (image.channels() == 1)) {
    cv::cvtColor(image, image, CV_GRAY2BGR);
  } else if (!color() && (image.channels() == 3)) {
    cv::cvtColor(image, image, CV_BGR2GRAY);
  }
  // Assemble image_msg
  const auto channels = image.channels();
  image_msg->height = image.rows;
  image_msg->width = image.cols;
  image_msg->step = image_msg->width * channels;
  if (channels == 1) {
    image_msg->encoding = sensor_msgs::image_encodings::MONO8;
  } else if (channels == 3) {
    image_msg->encoding = sensor_msgs::image_encodings::BGR8;
  }
  auto data_size = image_msg->height * image_msg->step;
  if (image_msg->data.size() != data_size) {
    image_msg->data.resize(data_size);
  }
  memcpy(&image_msg->data[0], image.data, data_size);
  return true;
}

}  // namespace usb_camera
