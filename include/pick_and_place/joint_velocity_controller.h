// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <franka_hw/franka_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <ros/ros.h>

#include <realtime_tools/realtime_buffer.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

namespace controllers {

class JointVelocityController : public controller_interface::MultiInterfaceController<
                                           hardware_interface::VelocityJointInterface,
                                           franka_hw::FrankaStateInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) override;
  void update(const ros::Time&, const ros::Duration& period) override;
  void starting(const ros::Time&) override;
  void stopping(const ros::Time&) override;
  void commandCB(const trajectory_msgs::JointTrajectoryPointConstPtr& msg);

 private:
  hardware_interface::VelocityJointInterface* velocity_joint_interface_;
  std::vector<hardware_interface::JointHandle> velocity_joint_handles_;
  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  ros::Duration elapsed_time_;
  double gain = 0.1;
  
  std::array<double,7> q_commands_;
  std::array<double,7> qd_commands_;
  ros::Subscriber sub_command_;
};

}  // namespace controllers
