#include <ros/ros.h>

#include <dd_control/control.h>

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "dd_control");

  ros::NodeHandle nh;
  ros::NodeHandle nh_priv("~");

  dd_control::Control control(nh, nh_priv);

  ros::spin();

  return 0;
}
