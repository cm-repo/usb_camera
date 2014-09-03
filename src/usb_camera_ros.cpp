#include "usb_camera/usb_camera_ros.h"

namespace usb_camera {

bool UsbCameraRos::Grab(const sensor_msgs::ImagePtr &image_msg) {
  cv::Mat image;
  if (!usb_camera_.GrabImage(image)) {
    return false;
  }
  // Assemble image_msg
  auto channels = image.channels();
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
