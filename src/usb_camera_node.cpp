#include "usb_camera/usb_camera.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "usb_camera");
  ros::NodeHandle nh("~");

  usb_camera::UsbCamera usb_camera(nh);
  usb_camera.Run();
  ros::spin();
  usb_camera.End();

  return 0;
}
