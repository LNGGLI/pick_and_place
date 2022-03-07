// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <pick_and_place/cartesian_pose_controller.h>
#include <pick_and_place/SetTraj.h>

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
#include "sun_math_toolbox/PortingFunctions.h"


#include <Eigen/Geometry>
#include <sun_math_toolbox/UnitQuaternion.h>



#include <TooN/TooN.h>

namespace controllers
{
  bool set_traj(pick_and_place::SetTraj::Request &req, pick_and_place::SetTraj::Response &resp){
    
    // Lettura posa finale e tempo della traiettoria
    TooN::Vector<3,double> final_position = TooN::makeVector(req.goal_position.x,req.goal_position.y,req.goal_position.z); 
    TooN::Vector<4,double> final_quaternion= TooN::makeVector(req.goal_quaternion.w,req.goal_quaternion.x,req.goal_quaternion.y,req.goal_quaternion.z);
    double Tf = req.Tf;

    // Lettura posa attuale
    std::array<double,16> current_pose = cartesian_pose_handle_->getRobotState().O_T_EE_c;

    TooN::Matrix<4,4,double> toon_current_pose = TooN::Data(current_pose[0], current_pose[4], current_pose[8], current_pose[12],
                                                    current_pose[1],current_pose[5],current_pose[9],current_pose[13],
                                                    current_pose[2],current_pose[6],current_pose[10],current_pose[14],
                                                    current_pose[3],current_pose[7],current_pose[11],current_pose[15]);
    
    // Ricavo posizione e quaternione attuale in base alla posa letta 
    TooN::Vector<3> current_position = sun::transl(toon_current_pose); 
    sun::UnitQuaternion current_quat(toon_current_pose);

    // Calcolo Delta_quaternione per la traiettoria in orientamento
    sun::UnitQuaternion final_quat(final_quaternion);
    sun::UnitQuaternion delta_quat = final_quat*inv(current_quat); // errore in terna base
    sun::AngVec angvec = delta_quat.toangvec();


    // Generazione della traiettoria
    sun::Quintic_Poly_Traj qp_position(Tf, 0.0, 1.0); // polinomio quintico utilizzato per line_traj 
    sun::Quintic_Poly_Traj qp_orientation(Tf, 0.0, angvec.getAng()); // polinomio quintico utilizzato per quat_traj
    sun::Line_Segment_Traj line_traj(current_position, final_position, qp_position); // Traiettoria in posizione su percorso di tipo segmento
    sun::Rotation_Const_Axis_Traj quat_traj(current_quat, angvec.getVec(), qp_orientation);  // Traiettoria in orientamento: rotazione attorno asse fisso nel tempo
    sun::Cartesian_Independent_Traj local_traj(line_traj, quat_traj);

    cartesian_traj_ = local_traj.clone();
    if(cartesian_traj_!=nullptr){
      resp.success = true;
      start = true;
      elapsed_time_ = ros::Duration(0.0); 
    }
    else{
      std::cout << "Errore nella creazione della traiettoria in \"set_traj\" \n";
      resp.success = false;
    }

    return true;

  }

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


    command_pb_ = node_handle.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>(
        "/cartesian_trajectory_command_internal", 1);

    server_set_traj_ = node_handle.advertiseService("/set_traj", set_traj);
    
    
    return true;
  }

  void CartesianPoseController::starting(const ros::Time & /* time */)
  {
    pose_ = cartesian_pose_handle_->getRobotState().O_T_EE_c;
    elapsed_time_ = ros::Duration(0.0); // Ogni volta che il controller viene avviato
  }

  void CartesianPoseController::update(const ros::Time & /* time */, const ros::Duration &period)
  {
    elapsed_time_ += period;
    if(!start){ // il controller è partito ma non è stata ancora assegnata nessuna posa desiderata.
      cartesian_pose_handle_->setCommand(pose_);
    }
    else{

      // Posizione
      TooN::Vector<3,double> position =  cartesian_traj_->getPosition(elapsed_time_.toSec());
      pose_[12] = position[0]; // x
      pose_[13] = position[1]; // y
      pose_[14] = position[2]; // z
      
      // Orientamento
      TooN::Matrix<3,3,double> R = cartesian_traj_->getQuaternion(elapsed_time_.toSec()).R();
      
      pose_[0] = R[0][0];
      pose_[1] = R[1][0];
      pose_[2] = R[2][0];
      pose_[4] = R[0][1];
      pose_[5] = R[1][1];
      pose_[6] = R[2][1];
      pose_[8] = R[0][2];
      pose_[9] = R[1][2];
      pose_[10] = R[2][2];
      cartesian_pose_handle_->setCommand(pose_);

      // Pubblicazione della traiettoria calcolata
      trajectory_msgs::MultiDOFJointTrajectoryPoint msg;
      msg.transforms.resize(1);
      msg.transforms[0].translation.x = pose_[12];
      msg.transforms[0].translation.y = pose_[13];
      msg.transforms[0].translation.z = pose_[14];
      sun::UnitQuaternion q  = cartesian_traj_->getQuaternion(elapsed_time_.toSec());
      msg.transforms[0].rotation.w = q.getS();
      msg.transforms[0].rotation.x = q.getV()[0];
      msg.transforms[0].rotation.y = q.getV()[1];
      msg.transforms[0].rotation.z = q.getV()[2];
      
      command_pb_.publish(msg);
    }
    
    
    
  }


} // namespace controllers

PLUGINLIB_EXPORT_CLASS(controllers::CartesianPoseController,
                       controller_interface::ControllerBase)
