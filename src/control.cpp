#include <dd_control/control.h>
#include <tf2/utils.h>

namespace dd_control
{
Control::Control(ros::NodeHandle& nh, ros::NodeHandle& nh_priv)
  : nh_(nh), nh_priv_(nh_priv), cs_(nh_priv_), error_prior_xy_(0), integral_xy_(0), error_prior_z_(0), integral_z_(0)
{
  // Set up dynamic reconfigure server
  f_ = boost::bind(&Control::configCallback, this, _1, _2);
  cs_.setCallback(f_);

  std::string setpoint_topic = nh_priv_.param<std::string>("setpoint_topic", "/setpoint");
  std::string pose_topic = nh_priv_.param<std::string>("pose_topic", "/mavros/"
                                                                     "local_position/"
                                                                     "pose");
  std::string velocity_topic = nh_priv_.param<std::string>("velocity_topic", "/mavros/local_position/"
                                                                             "velocity_local");
  std::string out_topic = nh_priv_.param<std::string>("out_topic", "/mavros/"
                                                                   "setpoint_position/"
                                                                   "local");

  tf_listener_ = new tf2_ros::TransformListener(tf_buffer_);

  setpoint_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>(setpoint_topic, 10, &Control::setpointCallback, this);
  cancel_sub_ = nh_.subscribe<std_msgs::Header>(setpoint_topic + "/cancel", 10, &Control::cancelCallback, this);
  pose_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>(pose_topic, 10, &Control::poseCallback, this);
  velocity_sub_ = nh_.subscribe<geometry_msgs::TwistStamped>(velocity_topic, 10, &Control::velocityCallback, this);

  control_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(out_topic, 10);
  velocity_pub_ = nh_priv_.advertise<dd_control::Velocity>("velocity", 10);
}

void Control::setpointCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  pub_timer_.stop();

  last_setpoint_ = *msg;
  setpointPublish(last_setpoint_);

  pub_timer_.start();
}

void Control::cancelCallback(const std_msgs::Header::ConstPtr& msg)
{
  last_setpoint_ = current_pose_;
}

void Control::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  current_pose_ = *msg;
}

void Control::velocityCallback(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
  current_velocity_ = *msg;
}

void Control::timerPublish(const ros::TimerEvent& event)
{
  last_setpoint_.header.stamp = ros::Time::now();
  setpointPublish(last_setpoint_);
}

// error_prior = 0
// integral = 0
// KP = Some value you need to come up (see tuning section below)
// KI = Some value you need to come up (see tuning section below)
// KD = Some value you need to come up (see tuning section below)

// while(1) {
//     error = desired_value – actual_value
//     integral = integral + (error*iteration_time)
//     derivative = (error – error_prior)/iteration_time
//     output = KP*error + KI*integral + KD*derivative + bias
//     error_prior = error
//     sleep(iteration_time)
// }

void Control::setpointPublish(const geometry_msgs::PoseStamped& setpoint)
{
  geometry_msgs::PoseStamped out;
  try
  {
    geometry_msgs::TransformStamped transform =
        tf_buffer_.lookupTransform(current_pose_.header.frame_id, setpoint.header.frame_id, ros::Time(0));

    tf2::doTransform(setpoint, out, transform);
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN_THROTTLE(1, "DD Control: %s", ex.what());
    out.header.frame_id = setpoint.header.frame_id;
    out.pose = current_pose_.pose;
  }
  out.header.stamp = setpoint.header.stamp;

  // http:  // robotsforroboticists.com/pid-control/

  // XY
  double desired_x = out.pose.position.x - current_pose_.pose.position.x;
  double desired_y = out.pose.position.y - current_pose_.pose.position.y;
  double desired_xy = std::min(std::hypot(desired_x, desired_y), max_xy_vel_);
  double desired_xy_dir = std::atan2(desired_x, desired_y);

  double current_xy = std::hypot(current_velocity_.twist.linear.x, current_velocity_.twist.linear.y);

  double error_xy = desired_xy - current_xy;
  integral_xy_ += (error_xy * (1.0 / frequency_));
  double derivate_xy_ = (error_xy - error_prior_xy_) / (1.0 / frequency_);
  error_prior_xy_ = error_xy;
  double output_xy = (k_p_xy_ * error_xy) + (k_i_xy_ * integral_xy_) + (k_d_xy_ * derivate_xy_) + bias_xy_;

  out.pose.position.x = current_pose_.pose.position.x + (output_xy * std::sin(desired_xy_dir));
  out.pose.position.y = current_pose_.pose.position.y + (output_xy * std::cos(desired_xy_dir));

  // Z
  double desired_z = clamp(out.pose.position.z - current_pose_.pose.position.z, -max_z_vel_, max_z_vel_);
  double error_z = desired_z - current_velocity_.twist.linear.z;
  integral_z_ += (error_z * (1.0 / frequency_));
  double derivate_z_ = (error_z - error_prior_z_) / (1.0 / frequency_);
  error_prior_z_ = error_z;
  out.pose.position.z =
      current_pose_.pose.position.z + ((k_p_z_ * error_z) + (k_i_z_ * integral_z_) + (k_d_z_ * derivate_z_) + bias_z_);

  control_pub_.publish(out);

  dd_control::Velocity vel_msg;
  vel_msg.header.stamp = out.header.stamp;
  vel_msg.header.frame_id = out.header.frame_id;
  vel_msg.current_xy_vel = current_xy;
  vel_msg.current_z_vel = current_velocity_.twist.linear.z;
  vel_msg.desired_xy_vel = desired_xy;
  vel_msg.desired_z_vel = desired_z;
  velocity_pub_.publish(vel_msg);
}

double Control::clamp(double value, double min, double max)
{
  return std::max(std::min(value, max), min);
}

void Control::configCallback(dd_control::ControlConfig& config, uint32_t level)
{
  max_xy_vel_ = config.max_xy_vel;
  max_z_vel_ = config.max_z_vel;

  k_p_xy_ = config.k_p_xy;
  k_i_xy_ = config.k_i_xy;
  k_d_xy_ = config.k_d_xy;
  bias_xy_ = config.bias_xy;

  k_p_z_ = config.k_p_z;
  k_i_z_ = config.k_i_z;
  k_d_z_ = config.k_d_z;
  bias_z_ = config.bias_z;

  frequency_ = config.frequency;

  if (0 == frequency_)
  {
    pub_timer_.stop();
  }
  else
  {
    pub_timer_ = nh_.createTimer(ros::Rate(frequency_), &Control::timerPublish, this, false, true);
  }
}
}  // namespace dd_control
