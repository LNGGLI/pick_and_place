// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <pick_and_place/cartesian_pose_controller.h>

#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include <controller_interface/controller_base.h>
#include <franka_hw/franka_cartesian_command_interface.h>
#include <hardware_interface/hardware_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace controllers
{

  bool CartesianPoseController::init(hardware_interface::RobotHW *robot_hardware,
                                     ros::NodeHandle &node_handle)
  {
    cartesian_pose_interface_ = robot_hardware->get<franka_hw::FrankaPoseCartesianInterface>();
    if (cartesian_pose_interface_ == nullptr)
    {
      ROS_ERROR(
          "CartesianPoseController: Could not get Cartesian Pose "
          "interface from hardware");
      return false;
    }

    std::string arm_id;
    if (!node_handle.getParam("arm_id", arm_id))
    {
      ROS_ERROR("CartesianPoseController: Could not get parameter arm_id");
      return false;
    }

    try
    {
      cartesian_pose_handle_ = std::make_unique<franka_hw::FrankaCartesianPoseHandle>(
          cartesian_pose_interface_->getHandle(arm_id + "_robot"));
    }
    catch (const hardware_interface::HardwareInterfaceException &e)
    {
      ROS_ERROR_STREAM(
          "CartesianPoseController: Exception getting Cartesian handle: " << e.what());
      return false;
    }

    auto state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
    if (state_interface == nullptr)
    {
      ROS_ERROR("CartesianPoseController: Could not get state interface from hardware");
      return false;
    }

    return true;
  }

  void CartesianPoseController::starting(const ros::Time & /* time */)
  {
    initial_pose_ = cartesian_pose_handle_->getRobotState().O_T_EE_d;
    elapsed_time_ = ros::Duration(0.0);
  }

  void CartesianPoseController::update(const ros::Time & /* time */,
                                       const ros::Duration &period)
  {
    elapsed_time_ += period;

    // auto state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
    // auto state_handle = state_interface->getHandle(arm_id + "_robot");
    std::array<double, 7> joint_position = cartesian_pose_handle_->getRobotState().q_d;

    double radius = 0.3;
    double angle = M_PI / 4 * (1 - std::cos(M_PI / 5.0 * elapsed_time_.toSec()));
    double delta_x = radius * std::sin(angle);
    double delta_z = radius * (std::cos(angle) - 1);
    std::array<double, 16> new_pose = initial_pose_;
    new_pose[12] -= delta_x;
    new_pose[14] -= delta_z;
    cartesian_pose_handle_->setCommand(new_pose);
  }

  void CartesianPoseController::CartesianTrajectoryCB(
    const trajectory_msgs::MultiDOFJointTrajectoryPointConstPtr& msg)
    {
        position_d_ << msg->transforms[0].translation.x, msg->transforms[0].translation.y, msg->transforms[0].translation.z;

        Eigen::Quaterniond last_orientation_d(orientation_d_);

        orientation_d_.coeffs() << msg->transforms[0].rotation.x, msg->transforms[0].rotation.y,
            msg->transforms[0].rotation.z, msg->transforms[0].rotation.w;

        if (last_orientation_d.coeffs().dot(orientation_d_.coeffs()) < 0.0)
        {
            orientation_d_.coeffs() << -orientation_d_.coeffs();
        }
    }
} // namespace controllers

PLUGINLIB_EXPORT_CLASS(controllers::CartesianPoseController,
                       controller_interface::ControllerBase)
