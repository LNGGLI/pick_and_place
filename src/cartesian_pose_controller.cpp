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

#include <sun_traj_lib/Cartesian_Independent_Traj.h>
#include <sun_traj_lib/Quintic_Poly_Traj.h>
#include <sun_traj_lib/Line_Segment_Traj.h>
#include <sun_traj_lib/Rotation_Const_Axis_Traj.h>

#include <Eigen/Geometry>
#include <sun_math_toolbox/UnitQuaternion.h>

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


    command_pb = node_handle.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>(
        "/cartesian_trajectory_command_internal", 1);

    return true;
  }

  void CartesianPoseController::starting(const ros::Time & /* time */)
  {
    initial_pose_ = cartesian_pose_handle_->getRobotState().O_T_EE;
    double Tf = 10;

    elapsed_time_ = ros::Duration(0.0);
    TooN::Vector<3> position_d_ = TooN::makeVector(initial_pose_[12], initial_pose_[13], initial_pose_[14]);

    TooN::Vector<3, double> pf({0.3, 0.3, 0.3});
    TooN::Vector<3, double> axis({0, 0, 1});
    sun::UnitQuaternion init_quat(TooN::Vector<3>(TooN::makeVector(0,0,0)),1);

    sun::Quintic_Poly_Traj qp(Tf, 0, 1); 
    sun::Line_Segment_Traj line_traj(position_d_, pf, qp);
    sun::Rotation_Const_Axis_Traj quat_traj(init_quat, axis, qp);
    sun::Cartesian_Independent_Traj local_traj(line_traj, quat_traj);
    cartesian_traj_ = local_traj.clone();
  }

  void CartesianPoseController::update(const ros::Time & /* time */, const ros::Duration &period)
  {
    // Eigen::Matrix3d R = orientation_d_.toRotationMatrix(); Da un valore sbagliato
    pose_ = initial_pose_;
    elapsed_time_ += period;
    pose_[12] = cartesian_traj_->getPosition(elapsed_time_.toSec())[0]; // x
    pose_[13] = cartesian_traj_->getPosition(elapsed_time_.toSec())[1]; // y
    pose_[14] = cartesian_traj_->getPosition(elapsed_time_.toSec())[2]; // z

    trajectory_msgs::MultiDOFJointTrajectoryPoint msg;
    msg.transforms.resize(1);
    msg.transforms[0].translation.x = pose_[12];
    msg.transforms[0].translation.y = pose_[13];
    msg.transforms[0].translation.z = pose_[14];
    command_pb.publish(msg);
    
    cartesian_pose_handle_->setCommand(pose_);
  }

} // namespace controllers

PLUGINLIB_EXPORT_CLASS(controllers::CartesianPoseController,
                       controller_interface::ControllerBase)
