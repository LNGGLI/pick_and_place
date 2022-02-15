// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <pick_and_place/joint_velocity_controller.h>

#include <cmath>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <Eigen/Core>
namespace controllers
{

  bool JointVelocityController::init(hardware_interface::RobotHW *robot_hardware,
                                     ros::NodeHandle &node_handle)
  {

    std::vector<std::string> joint_names;
    if (!node_handle.getParam("joint_names", joint_names))
    {
      ROS_ERROR("JointVelocityController: Could not parse joint names");
    }

    std::string arm_id;
    if (!node_handle.getParam("arm_id", arm_id))
    {
      ROS_ERROR_STREAM("CartesianImpedanceExampleController: Could not read parameter arm_id");
      return false;
    }

    velocity_joint_interface_ = robot_hardware->get<hardware_interface::VelocityJointInterface>();
    if (velocity_joint_interface_ == nullptr)
    {
      ROS_ERROR(
          "JointVelocityController: Error getting velocity joint interface from hardware!");
      return false;
    }
    if (joint_names.size() != 7)
    {
      ROS_ERROR_STREAM("JointVelocityController: Wrong number of joint names, got "
                       << joint_names.size() << " instead of 7 names!");
      return false;
    }
    velocity_joint_handles_.resize(7);
    for (size_t i = 0; i < 7; ++i)
    {
      try
      {
        velocity_joint_handles_[i] = velocity_joint_interface_->getHandle(joint_names[i]);
      }
      catch (const hardware_interface::HardwareInterfaceException &ex)
      {
        ROS_ERROR_STREAM(
            "JointVelocityController: Exception getting joint handles: " << ex.what());
        return false;
      }
    }

    auto *state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
    if (state_interface == nullptr)
    {
      ROS_ERROR_STREAM(
          "CartesianImpedanceExampleController: Error getting state interface from hardware");
      return false;
    }
    try
    {
      state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
          state_interface->getHandle(arm_id + "_robot"));
    }
    catch (hardware_interface::HardwareInterfaceException &ex)
    {
      ROS_ERROR_STREAM(
          "CartesianImpedanceExampleController: Exception getting state handle from interface: "
          << ex.what());
      return false;
    }

    commands_buffer_.writeFromNonRT(std::vector<double>(7, 0.0));
    velocity_buffer_.writeFromNonRT(std::vector<double>(7, 0.0));
    sub_command_ = node_handle.subscribe<trajectory_msgs::JointTrajectoryPoint>("/joint_commands", 1, &JointVelocityController::commandCB, this, ros::TransportHints().reliable().tcpNoDelay());

    return true;
  }

  void JointVelocityController::starting(const ros::Time & /* time */)
  {
    std::vector<double> initial_position;
    initial_position.resize(7);
    std::vector<double> initial_velocity;
    initial_position.resize(7);
    franka::RobotState initial_state = state_handle_->getRobotState();
    Eigen::Map<Eigen::Matrix<double, 7, 1>> q_initial(initial_state.q.data());
    Eigen::Map<Eigen::Matrix<double, 7, 1>> qd_initial(initial_state.dq.data());


    for (std::size_t i = 0; i < 7; ++i)
    {
      initial_position[i] = q_initial[i];
      initial_velocity[i] = qd_initial[i];
    }
    commands_buffer_.initRT(initial_position);
    velocity_buffer_.initRT(initial_velocity);

  }

  void JointVelocityController::update(const ros::Time & /* time */,
                                       const ros::Duration &period)
  {

    std::vector<double> & commands = *commands_buffer_.readFromRT();
    franka::RobotState state = state_handle_->getRobotState();

    Eigen::Map<Eigen::Matrix<double, 7, 1>> q_attuale(state.q.data());
    Eigen::Map<Eigen::Matrix<double, 7, 1>> q_comandata(commands.data());
    Eigen::Map<Eigen::Matrix<double, 7, 1>> qp_comand(commands.data());
    velocity_buffer_

    Eigen::Matrix<double, 7, 1> omega =  gain*(q_comandata-q_attuale);

    
    for (int i = 0; i < 7; i++)
    {
      velocity_joint_handles_[i].setCommand(omega[i]);
    }
  }

  void JointVelocityController::stopping(const ros::Time & /*time*/)
  {
    // WARNING: DO NOT SEND ZERO VELOCITIES HERE AS IN CASE OF ABORTING DURING MOTION
    // A JUMP TO ZERO WILL BE COMMANDED PUTTING HIGH LOADS ON THE ROBOT. LET THE DEFAULT
    // BUILT-IN STOPPING BEHAVIOR SLOW DOWN THE ROBOT.
  }

  void JointVelocityController::commandCB(const trajectory_msgs::JointTrajectoryPointConstPtr &msg)
  {
    if (msg->positions.size() != 7)
    {
      ROS_ERROR_STREAM("Dimension of command (" << msg->positions.size() << ") does not match number of joints. Not executing!");
      return;
    }
    commands_buffer_.writeFromNonRT(msg->positions);
    velocity_buffer_.writeFromNonRT(msg->velocities);
  }

} // namespace controllers

PLUGINLIB_EXPORT_CLASS(controllers::JointVelocityController,
                       controller_interface::ControllerBase)
