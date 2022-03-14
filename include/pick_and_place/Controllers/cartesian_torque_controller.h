// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once


#include <pick_and_place/SetTraj.h>

#include <memory>
#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <realtime_tools/realtime_publisher.h>
#include <ros/node_handle.h>
#include <ros/time.h>


#include <Eigen/Dense>
#include <franka_example_controllers/JointTorqueComparison.h>
#include <franka_hw/franka_cartesian_command_interface.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/trigger_rate.h>

// sun
#include "sun_math_toolbox/PortingFunctions.h"
#include <sun_traj_lib/Cartesian_Independent_Traj.h>
#include <sun_traj_lib/Line_Segment_Traj.h>
#include <sun_traj_lib/Quintic_Poly_Traj.h>
#include <sun_traj_lib/Rotation_Const_Axis_Traj.h>

namespace TorqueController {

  // Variabili handlers
  std::unique_ptr<franka_hw::FrankaCartesianPoseHandle> cartesian_pose_handle_;
  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
  std::vector<hardware_interface::JointHandle> joint_handles_;


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

  // Variabili calcolo coppie
  double coriolis_factor_{1.0};
  static constexpr double kDeltaTauMax{1.0};
  
  std::vector<double> k_gains_;
  std::vector<double> d_gains_;
  std::array<double, 7> dq_filtered_;
 
  // Variabili pubblicazione torques
  franka_hw::TriggerRate rate_trigger_{1.0};
  std::array<double, 7> last_tau_d_{};
  realtime_tools::RealtimePublisher<franka_example_controllers::JointTorqueComparison> torques_publisher_; 

  // Funzione srv
  bool set_traj(pick_and_place::SetTraj::Request &req, pick_and_place::SetTraj::Response &resp);




class CartesianTorqueController : public controller_interface::MultiInterfaceController<
                                            franka_hw::FrankaModelInterface,
                                            hardware_interface::EffortJointInterface,
                                            franka_hw::FrankaPoseCartesianInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;

 private:
  // Saturation
  std::array<double, 7> saturateTorqueRate(
      const std::array<double, 7>& tau_d_calculated,
      const std::array<double, 7>& tau_J_d);  // NOLINT (readability-identifier-naming)


  
};

}  // namespace controllers
