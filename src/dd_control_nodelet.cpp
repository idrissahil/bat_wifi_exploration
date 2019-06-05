#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <dd_control/control.h>

namespace dd_control
{
class ControlNodelet : public nodelet::Nodelet
{
private:
  Control* control_;

public:
  virtual void onInit()
  {
    control_ = new Control(getNodeHandle(), getPrivateNodeHandle());
  }
};
}

PLUGINLIB_EXPORT_CLASS(dd_control::ControlNodelet, nodelet::Nodelet)
