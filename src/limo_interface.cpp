// Copyright 2017-2019 Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <limo_interface/limo_interface.hpp>

#include <algorithm>
#include <limits>
#include <memory>
#include <utility>

LimoInterface::LimoInterface()
: Node("limo_interface"), vehicle_info_(vehicle_info_util::VehicleInfoUtil(*this).getVehicleInfo())
{
  /* setup parameters */
  base_frame_id_ = declare_parameter("base_frame_id", "base_link");
  loop_rate_ = declare_parameter("loop_rate", 30.0);

  /* parameters for vehicle specifications */
  wheel_base_ = vehicle_info_.wheel_base_m;

  /* subscribers */
  using std::placeholders::_1;
  using std::placeholders::_2;

  // From autoware
  control_cmd_sub_ = create_subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>(
    "/control/command/control_cmd", 1, std::bind(&LimoInterface::callbackControlCmd, this, _1));

  // From limo
  limo_status_sub_ = create_subscription<limo_msgs::msg::LimoStatus>(
    "/limo_status", 1, std::bind(&LimoInterface::callbackLimoStatus, this, _1));
  limo_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "/odom", 1, std::bind(&LimoInterface::callbackLimoOdom, this, _1));

  /* publisher */
  // To limo
  autoware_cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/autoware_cmd", rclcpp::QoS{1});

  // To Autoware
  control_mode_pub_ = create_publisher<autoware_auto_vehicle_msgs::msg::ControlModeReport>(
    "/vehicle/status/control_mode", rclcpp::QoS{1});
  vehicle_twist_pub_ = create_publisher<autoware_auto_vehicle_msgs::msg::VelocityReport>(
    "/vehicle/status/velocity_status", rclcpp::QoS{1});
  steering_status_pub_ = create_publisher<autoware_auto_vehicle_msgs::msg::SteeringReport>(
    "/vehicle/status/steering_status", rclcpp::QoS{1});
  gear_status_pub_ = create_publisher<autoware_auto_vehicle_msgs::msg::GearReport>(
    "/vehicle/status/gear_status", rclcpp::QoS{1});

  // Timer
  const auto period_ns = rclcpp::Rate(loop_rate_).period();
  timer_ = rclcpp::create_timer(
    this, get_clock(), period_ns, std::bind(&LimoInterface::publishCommands, this));
}

void LimoInterface::callbackControlCmd(
  const autoware_auto_control_msgs::msg::AckermannControlCommand::ConstSharedPtr msg)
{
  control_cmd_ptr_ = msg;
}

void LimoInterface::callbackLimoStatus(const limo_msgs::msg::LimoStatus::ConstSharedPtr msg)
{
  // TODO
}

void LimoInterface::callbackLimoOdom(const nav_msgs::msg::Odometry::ConstSharedPtr limo_odom)
{
  std_msgs::msg::Header header;
  header.frame_id = base_frame_id_;
  header.stamp = get_clock()->now();

  const auto current_velocity = limo_odom->twist.twist.linear.x;
  const auto current_steer = limo_odom->twist.twist.angular.z;

  /* publish vehicle status control_mode */
  {
    autoware_auto_vehicle_msgs::msg::ControlModeReport control_mode_msg;
    control_mode_msg.stamp = header.stamp;
    control_mode_msg.mode = autoware_auto_vehicle_msgs::msg::ControlModeReport::AUTONOMOUS;
    control_mode_pub_->publish(control_mode_msg);
  }

  /* publish vehicle status twist */
  {
    autoware_auto_vehicle_msgs::msg::VelocityReport twist;
    twist.header = header;
    twist.longitudinal_velocity = current_velocity;                                 // [m/s]
    twist.heading_rate = current_velocity * std::tan(current_steer) / wheel_base_;  // [rad/s]
    vehicle_twist_pub_->publish(twist);
  }

  /* publish current shift */
  {
    using autoware_auto_vehicle_msgs::msg::GearReport;
    GearReport gear_report_msg;
    gear_report_msg.stamp = header.stamp;
    gear_report_msg.report = GearReport::DRIVE;
    gear_status_pub_->publish(gear_report_msg);
  }

  /* publish current status */
  {
    autoware_auto_vehicle_msgs::msg::SteeringReport steer_msg;
    steer_msg.stamp = header.stamp;
    steer_msg.steering_tire_angle = current_steer;
    steering_status_pub_->publish(steer_msg);
  }
}

void LimoInterface::publishCommands()
{
  if (!control_cmd_ptr_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "No control command received.");
    return;
  }

  geometry_msgs::msg::Twist autoware_cmd;
  autoware_cmd.linear.x = control_cmd_ptr_->longitudinal.speed;
  autoware_cmd.linear.y = 0;
  autoware_cmd.linear.z = 0;
  autoware_cmd.angular.x = 0;
  autoware_cmd.angular.y = 0;
  autoware_cmd.angular.z = control_cmd_ptr_->lateral.steering_tire_angle;
  autoware_cmd_pub_->publish(autoware_cmd);
}
