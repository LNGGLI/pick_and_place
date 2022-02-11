// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <array>
#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <realtime_tools/realtime_buffer.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <ros/node_handle.h>
#include <ros/time.h>

namespace controllers {

class PickAndPlaceController : public controller_interface::MultiInterfaceController<
                                           hardware_interface::PositionJointInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;
  void commandCB(const trajectory_msgs::JointTrajectoryPointConstPtr& msg);

 private:
  hardware_interface::PositionJointInterface* position_joint_interface_;
  std::vector<hardware_interface::JointHandle> position_joint_handles_;
  ros::Duration elapsed_time_;
  

  unsigned int n_joints_ = 7;
  realtime_tools::RealtimeBuffer<std::vector<double>> commands_buffer_;
  ros::Subscriber sub_command_;

  
};

}  // namespace controllers
