// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <array>
#include <memory>
#include <string>

#include <controller_interface/multi_interface_controller.h>
#include <franka_hw/franka_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <Eigen/Dense>

#include <realtime_tools/realtime_publisher.h>

#include <franka_hw/franka_cartesian_command_interface.h>

#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

namespace controllers {

class CartesianPoseController
    : public controller_interface::MultiInterfaceController<franka_hw::FrankaPoseCartesianInterface,
                                                            franka_hw::FrankaStateInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;

 private:
  franka_hw::FrankaPoseCartesianInterface* cartesian_pose_interface_;
  std::unique_ptr<franka_hw::FrankaCartesianPoseHandle> cartesian_pose_handle_;
  ros::Duration elapsed_time_;
  std::array<double, 16> pose_{};
  std::array<double, 16> initial_pose_{};
  bool start = true;


  Eigen::Vector3d position_d_;
  Eigen::Quaterniond orientation_d_;

  ros::Subscriber sub_cartesian_trajectory_;
  void CartesianTrajectoryCB(const trajectory_msgs::MultiDOFJointTrajectoryPointConstPtr& msg);
  
};

}  // namespace controllers
