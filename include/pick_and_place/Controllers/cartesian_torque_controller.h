// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <memory>
#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <realtime_tools/realtime_publisher.h>
#include <ros/node_handle.h>
#include <ros/time.h>

#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>


#include <Eigen/Dense>
#include <franka_example_controllers/JointTorqueComparison.h>
#include <franka_hw/franka_cartesian_command_interface.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/trigger_rate.h>

namespace controllers {

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


  std::unique_ptr<franka_hw::FrankaCartesianPoseHandle> cartesian_pose_handle_;
  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
  std::vector<hardware_interface::JointHandle> joint_handles_;

  double coriolis_factor_{1.0};
  static constexpr double kDeltaTauMax{1.0};
  std::array<double, 16> initial_pose_{};
  std::array<double, 16> pose_{};
  franka_hw::TriggerRate rate_trigger_{1.0};

  Eigen::Vector3d position_d_;
  Eigen::Quaterniond orientation_d_;

  std::vector<double> k_gains_;
  std::vector<double> d_gains_;
  std::array<double, 7> dq_filtered_;
 

  ros::Subscriber sub_cartesian_trajectory_;
  void CartesianTrajectoryCB(const trajectory_msgs::MultiDOFJointTrajectoryPointConstPtr& msg);

  std::array<double, 7> last_tau_d_{};
  realtime_tools::RealtimePublisher<franka_example_controllers::JointTorqueComparison> torques_publisher_;
};

}  // namespace controllers
