// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <pick_and_place/SetTraj.h>
#include <pick_and_place/Panda.h>

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
  
  // Oggetto panda
  sun::Panda panda(TooN::makeVector(1.0,1.0,1.0,1.0).as_diagonal(),1.0,"panda");

  // Variabili per la traiettoria
  TooN::Matrix<4, 4, double> current_pose_;
  std::array<double,7> initial_configuration_;

  sun::Cartesian_Independent_Traj* cartesian_traj_;
  
  // Variabili ROS
  ros::ServiceServer server_set_traj_;
  ros::Publisher pub_command_;

  // Variabili di controllo
  bool start = false;
  ros::Duration elapsed_time_;

  // Variabili del CLIK
  double Ts = 0.001;  // periodo s
  double fs = 1 / Ts; // frequenza Hz
  double gain = 0.5 * fs;
  TooN::Vector<> qdot = TooN::Zeros(7); // velocità di giunto ritorno
  TooN::Vector<6, int> mask = 
      TooN::Ones; // maschera, se l'i-esimo elemento è zero allora l'i-esima
                  // componente cartesiana non verrà usata per il calcolo
                  // dell'errore
  TooN::Vector<3> xd = TooN::Zeros; // velocità in translazione desiderata
  TooN::Vector<3> w = TooN::Zeros;  // velocità angolare desiderata
  TooN::Vector<6> error = TooN::Ones; // questo va "resettato" ogni volta prima del clik
  TooN::Vector<7> qDH_k;
  sun::UnitQuaternion oldQ;

  sun::UnitQuaternion unit_quat_d_;
  TooN::Vector<3, double> posizione_d_;


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
