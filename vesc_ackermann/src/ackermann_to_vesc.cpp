// -*- mode:c++; fill-column: 100; -*-

#include "vesc_ackermann/ackermann_to_vesc.h"

#include <cmath>
#include <sstream>
#include <functional>

namespace vesc_ackermann
{

AckermannToVesc::AckermannToVesc(const rclcpp::NodeOptions & options)
  : rclcpp::Node("ackermann_to_vesc", options)
{
  // get conversion parameters
  this->declare_parameter<double>("speed_to_erpm_gain", 0.0);
  this->declare_parameter<double>("speed_to_erpm_offset", 0.0);
  this->declare_parameter<double>("steering_angle_to_servo_gain", 0.0);
  this->declare_parameter<double>("steering_angle_to_servo_offset", 0.0);

  speed_to_erpm_gain_ = this->get_parameter("speed_to_erpm_gain").as_double();
  speed_to_erpm_offset_ = this->get_parameter("speed_to_erpm_offset").as_double();
  steering_to_servo_gain_ = this->get_parameter("steering_angle_to_servo_gain").as_double();
  steering_to_servo_offset_ = this->get_parameter("steering_angle_to_servo_offset").as_double();

  if (speed_to_erpm_gain_ == 0.0) {
    RCLCPP_FATAL(get_logger(), "AckermannToVesc: Parameter speed_to_erpm_gain is required.");
  }
  if (steering_to_servo_gain_ == 0.0) {
    RCLCPP_FATAL(get_logger(), "AckermannToVesc: Parameter steering_angle_to_servo_gain is required.");
  }

  // create publishers to vesc electric-RPM (speed) and servo commands
  erpm_pub_ = this->create_publisher<std_msgs::msg::Float64>("commands/motor/speed", 10);
  servo_pub_ = this->create_publisher<std_msgs::msg::Float64>("commands/servo/position", 10);

  // subscribe to ackermann topic
  ackermann_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
    "ackermann_cmd", 10,
    std::bind(&AckermannToVesc::ackermannCmdCallback, this, std::placeholders::_1));
}

void AckermannToVesc::ackermannCmdCallback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr cmd)
{
  // calc vesc electric RPM (speed)
  auto erpm_msg = std_msgs::msg::Float64();
  erpm_msg.data = speed_to_erpm_gain_ * cmd->drive.speed + speed_to_erpm_offset_;

  // calc steering angle (servo)
  auto servo_msg = std_msgs::msg::Float64();
  servo_msg.data = steering_to_servo_gain_ * cmd->drive.steering_angle + steering_to_servo_offset_;

  // publish
  erpm_pub_->publish(erpm_msg);
  servo_pub_->publish(servo_msg);
}

} // namespace vesc_ackermann
