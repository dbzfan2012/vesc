// -*- mode:c++; fill-column: 100; -*-

#include "vesc_ackermann/vesc_to_odom.h"

#include <cmath>
#include <functional>

#include <geometry_msgs/msg/transform_stamped.hpp>

namespace vesc_ackermann
{

VescToOdom::VescToOdom(const rclcpp::NodeOptions & options) :
  rclcpp::Node("vesc_to_odom", options),
  odom_frame_("odom"), base_frame_("base_link"),
  use_servo_cmd_(true), publish_tf_(false), x_(0.0), y_(0.0), yaw_(0.0)
{
  // get ROS parameters
  this->declare_parameter<std::string>("odom_frame", odom_frame_);
  this->declare_parameter<std::string>("base_frame", base_frame_);
  this->declare_parameter<bool>("use_servo_cmd_to_calc_angular_velocity", use_servo_cmd_);
  this->declare_parameter<double>("speed_to_erpm_gain", 0.0);
  this->declare_parameter<double>("speed_to_erpm_offset", 0.0);
  this->declare_parameter<double>("steering_angle_to_servo_gain", 0.0);
  this->declare_parameter<double>("steering_angle_to_servo_offset", 0.0);
  this->declare_parameter<double>("wheelbase", 0.0);
  this->declare_parameter<bool>("publish_tf", publish_tf_);

  odom_frame_ = this->get_parameter("odom_frame").as_string();
  base_frame_ = this->get_parameter("base_frame").as_string();
  use_servo_cmd_ = this->get_parameter("use_servo_cmd_to_calc_angular_velocity").as_bool();
  speed_to_erpm_gain_ = this->get_parameter("speed_to_erpm_gain").as_double();
  speed_to_erpm_offset_ = this->get_parameter("speed_to_erpm_offset").as_double();
  publish_tf_ = this->get_parameter("publish_tf").as_bool();

  if (speed_to_erpm_gain_ == 0.0) {
    RCLCPP_FATAL(get_logger(), "VescToOdom: Parameter speed_to_erpm_gain is required.");
  }

  if (use_servo_cmd_) {
    steering_to_servo_gain_ = this->get_parameter("steering_angle_to_servo_gain").as_double();
    steering_to_servo_offset_ = this->get_parameter("steering_angle_to_servo_offset").as_double();
    wheelbase_ = this->get_parameter("wheelbase").as_double();
    if (steering_to_servo_gain_ == 0.0) {
      RCLCPP_FATAL(get_logger(), "VescToOdom: Parameter steering_angle_to_servo_gain is required.");
    }
    if (wheelbase_ == 0.0) {
      RCLCPP_FATAL(get_logger(), "VescToOdom: Parameter wheelbase is required.");
    }
  }

  // create odom publisher
  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);

  // create tf broadcaster
  if (publish_tf_) {
    tf_pub_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }

  // subscribe to vesc state and. optionally, servo command
  vesc_state_sub_ = this->create_subscription<vesc_msgs::msg::VescStateStamped>(
    "sensors/core", 10,
    std::bind(&VescToOdom::vescStateCallback, this, std::placeholders::_1));
  if (use_servo_cmd_) {
    servo_sub_ = this->create_subscription<std_msgs::msg::Float64>(
      "sensors/servo_position_command", 10,
      std::bind(&VescToOdom::servoCmdCallback, this, std::placeholders::_1));
  }
}

void VescToOdom::vescStateCallback(const vesc_msgs::msg::VescStateStamped::SharedPtr state)
{
  // check that we have a last servo command if we are depending on it for angular velocity
  if (use_servo_cmd_ && !last_servo_cmd_)
    return;

  // convert to engineering units
  double current_speed = ( state->state.speed - speed_to_erpm_offset_ ) / speed_to_erpm_gain_;
  double current_steering_angle(0.0), current_angular_velocity(0.0);
  if (use_servo_cmd_) {
    current_steering_angle =
      ( last_servo_cmd_->data - steering_to_servo_offset_ ) / steering_to_servo_gain_;
    current_angular_velocity = current_speed * tan(current_steering_angle) / wheelbase_;
  }

  // use current state as last state if this is our first time here
  if (!last_state_)
    last_state_ = state;

  // calc elapsed time
  rclcpp::Duration dt = rclcpp::Time(state->header.stamp) - rclcpp::Time(last_state_->header.stamp);

  /** @todo could probably do better propigating odometry, e.g. trapezoidal integration */

  // propigate odometry
  double x_dot = current_speed * cos(yaw_);
  double y_dot = current_speed * sin(yaw_);
  x_ += x_dot * dt.seconds();
  y_ += y_dot * dt.seconds();
  if (use_servo_cmd_)
    yaw_ += current_angular_velocity * dt.seconds();

  // save state for next time
  last_state_ = state;

  // publish odometry message
  auto odom = nav_msgs::msg::Odometry();
  odom.header.frame_id = odom_frame_;
  odom.header.stamp = state->header.stamp;
  odom.child_frame_id = base_frame_;

  // Position
  odom.pose.pose.position.x = x_;
  odom.pose.pose.position.y = y_;
  odom.pose.pose.orientation.x = 0.0;
  odom.pose.pose.orientation.y = 0.0;
  odom.pose.pose.orientation.z = sin(yaw_/2.0);
  odom.pose.pose.orientation.w = cos(yaw_/2.0);

  // Position uncertainty
  /** @todo Think about position uncertainty, perhaps get from parameters? */
  odom.pose.covariance[0]  = 0.2; ///< x
  odom.pose.covariance[7]  = 0.2; ///< y
  odom.pose.covariance[35] = 0.4; ///< yaw

  // Velocity ("in the coordinate frame given by the child_frame_id")
  odom.twist.twist.linear.x = current_speed;
  odom.twist.twist.linear.y = 0.0;
  odom.twist.twist.angular.z = current_angular_velocity;

  // Velocity uncertainty
  /** @todo Think about velocity uncertainty */

  if (publish_tf_) {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.frame_id = odom_frame_;
    tf.child_frame_id = base_frame_;
    tf.header.stamp = this->now();
    tf.transform.translation.x = x_;
    tf.transform.translation.y = y_;
    tf.transform.translation.z = 0.0;
    tf.transform.rotation = odom.pose.pose.orientation;
    tf_pub_->sendTransform(tf);
  }

  odom_pub_->publish(odom);
}

void VescToOdom::servoCmdCallback(const std_msgs::msg::Float64::SharedPtr servo)
{
  last_servo_cmd_ = servo;
}

} // namespace vesc_ackermann
