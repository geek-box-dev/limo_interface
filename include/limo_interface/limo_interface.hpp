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

#ifndef LIMO_INTERFACE__LIMO_INTERFACE_HPP_
#define LIMO_INTERFACE__LIMO_INTERFACE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <vehicle_info_util/vehicle_info_util.hpp>

#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/control_mode_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/gear_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/steering_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/velocity_report.hpp>
#include <limo_msgs/msg/limo_status.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <memory>
#include <optional>
#include <string>

class LimoInterface : public rclcpp::Node
{
public:
  LimoInterface();

private:
  /* subscribers */
  // From Autoware
  rclcpp::Subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>::SharedPtr
    control_cmd_sub_;

  // From Limo
  rclcpp::Subscription<limo_msgs::msg::LimoStatus>::SharedPtr limo_status_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr limo_odom_sub_;

  /* publishers */
  // To Limo
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr autoware_cmd_pub_;

  // To Autoware
  rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::ControlModeReport>::SharedPtr
    control_mode_pub_;
  rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::VelocityReport>::SharedPtr vehicle_twist_pub_;
  rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::SteeringReport>::SharedPtr
    steering_status_pub_;
  rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::GearReport>::SharedPtr gear_status_pub_;

  rclcpp::TimerBase::SharedPtr timer_;

  /* ros param */
  double loop_rate_;  // [Hz]
  std::string base_frame_id_;
  double wheel_base_;  // [m]

  vehicle_info_util::VehicleInfo vehicle_info_;
  autoware_auto_control_msgs::msg::AckermannControlCommand::ConstSharedPtr control_cmd_ptr_;

  /* callbacks */

  void callbackControlCmd(
    const autoware_auto_control_msgs::msg::AckermannControlCommand::ConstSharedPtr msg);

  void callbackLimoStatus(const limo_msgs::msg::LimoStatus::ConstSharedPtr msg);
  void callbackLimoOdom(const nav_msgs::msg::Odometry::ConstSharedPtr msg);

  /*  functions */
  void publishCommands();
};

#endif  // LIMO_INTERFACE__LIMO_INTERFACE_HPP_
