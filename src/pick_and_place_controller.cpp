// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE

#include <pick_and_place/pick_and_place_controller.h>

#include <cmath>

#include <realtime_tools/realtime_buffer.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace controllers {


bool PickAndPlaceController::init(hardware_interface::RobotHW* robot_hardware,
                                          ros::NodeHandle& node_handle) {
  position_joint_interface_ = robot_hardware->get<hardware_interface::PositionJointInterface>();
  if (position_joint_interface_ == nullptr) {
    ROS_ERROR(
        "PickAndPlaceController: Error getting position joint interface from hardware!");
    return false;
  }
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names)) {
    ROS_ERROR("PickAndPlaceController: Could not parse joint names");
  }
  if (joint_names.size() != 7) {
    ROS_ERROR_STREAM("PickAndPlaceController: Wrong number of joint names, got "
                     << joint_names.size() << " instead of 7 names!");
    return false;
  }
  position_joint_handles_.resize(7);
  for (size_t i = 0; i < 7; ++i) {
    try {
      position_joint_handles_[i] = position_joint_interface_->getHandle(joint_names[i]);
    } catch (const hardware_interface::HardwareInterfaceException& e) {
      ROS_ERROR_STREAM(
          "PickAndPlaceController: Exception getting joint handles: " << e.what());
      return false;
    }
  }


  // inizializzazione del buffer realtime e del subscriber
  sub_command_ = node_handle.subscribe<trajectory_msgs::JointTrajectoryPoint>("/joint_commands", 1, &PickAndPlaceController::commandCB, this);
  

  return true;
}

void PickAndPlaceController::starting(const ros::Time& /* time */) {

  elapsed_time_ = ros::Duration(0.0);
}

void PickAndPlaceController::update(const ros::Time& time,
                                            const ros::Duration& period) {
  for(int i = 0;i<7;i++){
    position_joint_handles_[i].setCommand(command_[i]);
  }

    
}
  
  void PickAndPlaceController::commandCB(const trajectory_msgs::JointTrajectoryPointConstPtr& msg){
    
    if(msg->positions.size()!=7)
    {
      ROS_ERROR_STREAM("Dimension of command (" << msg->positions.size() << ") does not match number of joints. Not executing!");
      return;
    }
    
    for(int i =0; i< 7; i++)
      command_[i] = msg->positions[i];
    
  }


}  // namespace controllers

PLUGINLIB_EXPORT_CLASS(controllers::PickAndPlaceController,
                       controller_interface::ControllerBase)

// PLUGINLIB_EXPORT_CLASS(name_of_your_controller_package::NameOfYourControllerClass,
//                      controller_interface::ControllerBase)
