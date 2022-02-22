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
#include "sun_math_toolbox/PortingFunctions.h"


#include <Eigen/Geometry>
#include <sun_math_toolbox/UnitQuaternion.h>



#include <TooN/TooN.h>

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
    initial_pose_ = cartesian_pose_handle_->getRobotState().O_T_EE_c;

    TooN::Matrix<4,4,double> toon_pose = TooN::Data(initial_pose_[0], initial_pose_[4], initial_pose_[8], initial_pose_[12],
                                                    initial_pose_[1],initial_pose_[5],initial_pose_[9],initial_pose_[13],
                                                    initial_pose_[2],initial_pose_[6],initial_pose_[10],initial_pose_[14],
                                                    initial_pose_[3],initial_pose_[7],initial_pose_[11],initial_pose_[15]);
    double Tf = 15;
    std::cout << toon_pose;
    TooN::Vector<3> position_d_ = sun::transl(toon_pose);
    
    sun::UnitQuaternion init_quat(toon_pose);
    
    TooN::Matrix<3, 3> Rot_des = TooN::Data(1,0,0,0,0,1,0,-1,0); // orientamento desiderata
    
    sun::UnitQuaternion final_quat(Rot_des);
    sun::UnitQuaternion delta_quat = final_quat*inv(init_quat); // errore in terna base
    sun::AngVec angvec = delta_quat.toangvec();

    // Nota: la libreria vuole delta_quat definita in terna base.

    // Generazione della traiettoria su primitiva di percorso di tipo segmento:

    std::cout << "Generazione della traiettoria in cartesiano \n";

    TooN::Vector<3, double> pf({0.4,0.4,0.4});
    //TooN::Vector<3, double> pf({0.655105430989015, 0.1096259365986445, 0.06857646438044779-0.01});

    sun::Quintic_Poly_Traj qp_position(Tf, 0.0, 1.0); // polinomio quintico utilizzato per line_traj 
    sun::Quintic_Poly_Traj qp_orientation(Tf, 0.0, angvec.getAng()); // polinomio quintico utilizzato per quat_traj
    
    
    sun::Line_Segment_Traj line_traj(position_d_, pf, qp_position);
    sun::Rotation_Const_Axis_Traj quat_traj(init_quat, angvec.getVec(), qp_orientation);
    
    // Nota: la traiettoria in orientamento è quella definita dal Delta_quat.
    // Perchè vogliamo andare da init_quat (orientamento iniziale) a final_quat (orientamento finale).
    // Il metodo getquaternion(t) restituisce DeltaQuat(t) * init_quat. 
    
    sun::Cartesian_Independent_Traj local_traj(line_traj, quat_traj);
    cartesian_traj_ = local_traj.clone();


    elapsed_time_ = ros::Duration(0.0);
  }

  void CartesianPoseController::update(const ros::Time & /* time */, const ros::Duration &period)
  {
    
    pose_ = initial_pose_;
    elapsed_time_ += period;

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

    trajectory_msgs::MultiDOFJointTrajectoryPoint msg;
    msg.transforms.resize(1);
    msg.transforms[0].translation.x = pose_[12];
    msg.transforms[0].translation.y = pose_[13];
    msg.transforms[0].translation.z = pose_[14];
    command_pb.publish(msg);
    
    
  }

} // namespace controllers

PLUGINLIB_EXPORT_CLASS(controllers::CartesianPoseController,
                       controller_interface::ControllerBase)
