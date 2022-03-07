// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once
#include <pick_and_place/SetTraj.h>

#include <array>
#include <memory>
#include <string>

#include <controller_interface/multi_interface_controller.h>
#include <franka_hw/franka_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <Eigen/Dense>


#include <realtime_tools/realtime_buffer.h>

#include <franka_hw/franka_cartesian_command_interface.h>

#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include <sun_traj_lib/Cartesian_Independent_Traj.h>
#include <sun_traj_lib/Quintic_Poly_Traj.h>
#include <sun_traj_lib/Line_Segment_Traj.h>
#include <sun_traj_lib/Rotation_Const_Axis_Traj.h>

namespace controllers {

  // Variabili handlers
  franka_hw::FrankaPoseCartesianInterface* cartesian_pose_interface_;
  std::unique_ptr<franka_hw::FrankaCartesianPoseHandle> cartesian_pose_handle_;
  
  // Variabili per la traiettoria
  std::array<double, 16> pose_{};
  std::array<double, 16> initial_pose_{};

  sun::Cartesian_Independent_Traj* cartesian_traj_;
  
  // Variabili ROS
  ros::Publisher command_pb_;
  ros::ServiceServer server_set_traj_;

  // Variabili di controllo
  bool start = false;
  ros::Duration elapsed_time_;

  bool set_traj(pick_and_place::SetTraj::Request &req, pick_and_place::SetTraj::Response &resp);



class CartesianPoseController
    : public controller_interface::MultiInterfaceController<franka_hw::FrankaPoseCartesianInterface,
                                                            franka_hw::FrankaStateInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;
    

};

}  // namespace controllers
