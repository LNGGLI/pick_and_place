// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <pick_and_place/SetTraj.h>

#include <array>
#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <franka_hw/franka_state_interface.h>

#include <hardware_interface/robot_hw.h>
#include <realtime_tools/realtime_buffer.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <Eigen/Dense>

#include <sun_traj_lib/Cartesian_Independent_Traj.h>
#include <sun_traj_lib/Quintic_Poly_Traj.h>
#include <sun_traj_lib/Line_Segment_Traj.h>
#include <sun_traj_lib/Rotation_Const_Axis_Traj.h>

namespace JointController {

  // Variabili per la traiettoria
  std::array<double, 16> current_pose_{};
  std::array<double,7> current_configuration_{};

  sun::Cartesian_Independent_Traj* cartesian_traj_;
  
  // Variabili ROS
  ros::Publisher command_pb_;
  ros::ServiceServer server_set_traj_;

  // Variabili di controllo
  bool start = false;
  ros::Duration elapsed_time_;

  // Handler per il controller
  hardware_interface::PositionJointInterface* position_joint_interface_;
  std::vector<hardware_interface::JointHandle> position_joint_handles_;
  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;


  bool set_traj(pick_and_place::SetTraj::Request &req, pick_and_place::SetTraj::Response &resp);




class JointController : public controller_interface::MultiInterfaceController<
                            hardware_interface::PositionJointInterface,
                            franka_hw::FrankaStateInterface> {
public:
  bool init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;
  
  
};

}  // namespace demo
