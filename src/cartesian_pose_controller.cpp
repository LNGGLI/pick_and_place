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

    sub_cartesian_trajectory_ = node_handle.subscribe<trajectory_msgs::MultiDOFJointTrajectoryPoint>("/cartesian_trajectory_command", 10, &CartesianPoseController::CartesianTrajectoryCB, this);

    joints_publisher_ = node_handle.advertise<trajectory_msgs::JointTrajectoryPoint>("/jointsIK", 1);

    return true;
  }


  void CartesianPoseController::starting(const ros::Time& /* time */) {
    initial_pose_ = cartesian_pose_handle_->getRobotState().O_T_EE;
    
    elapsed_time_ = ros::Duration(0.0);

  }



  void CartesianPoseController::update(const ros::Time & /* time */, const ros::Duration &period)
  {
    // Eigen::Matrix3d R = orientation_d_.toRotationMatrix(); Da un valore sbagliato
    pose_ = initial_pose_;
    std::array<double,16> actual_pose = cartesian_pose_handle_->getRobotState().O_T_EE;

    // TooN::Vector<4> quat = TooN::makeVector(orientation_d_.x(),orientation_d_.y(),orientation_d_.z(),orientation_d_.w());
    // sun::UnitQuaternion unit_quat(quat);
    // TooN::Matrix<3> R = unit_quat.R();
    

    // pose_[0] = R[0][0];
    // pose_[1] = R[1][0];
    // pose_[2] = R[2][0];
    // pose_[4] = R[0][1];
    // pose_[5] = R[1][1];
    // pose_[6] = R[2][1];
    // pose_[8] = R[0][2];
    // pose_[9] = R[1][2];
    // pose_[10] = R[2][2];

    
    pose_[12] = position_d_[0]; // x
    pose_[13] = position_d_[1]; // y
    pose_[14] = position_d_[2]; // z

    //Stampa solo la prima volta
    if(start){
      start = false;
      for(int i = 0; i< 16; i++){
        if(abs(initial_pose_[i]-pose_[i])>0.1){
          std::cout << " Posa ottenuta troppo differente dalla iniziale: elemento " << i+1 << "\n";
          std::cout << " Intial_pose_["<<i<<"] = " << initial_pose_[i] << " mentre pose_["<<i<<"] = " << pose_[i] << "\n";
        }
      }

      std::cout << "\n Pose: " << std::endl;
      for(int i = 0; i< 16; i++)
        std::cout << pose_[i] << " ";

      std::cout << "\n initial_pose: " << std::endl;
      for(int i = 0; i< 16; i++)
        std::cout << initial_pose_[i] << " ";

      // std::cout << "\n Matrice di rotazione letta nell'update():";
      // for(int i = 0; i < 3 ; i++ ){
      //             for(int j = 0 ; j<3 ; j++)
      //                 std::cout << R[i][j] << " ";
      //             std::cout << "\n";
      // }


    }

    // std::cout << "\nPose commanded: " << std::endl;
    //   for(int i = 0; i< 16; i++)
    //     std::cout << pose_[i] << " ";

    // std::cout << "\n Actual pose: " << std::endl;
    //   for(int i = 0; i< 16; i++)
    //   std::cout << actual_pose[i] << " ";

  
    cartesian_pose_handle_->setCommand(pose_);

    trajectory_msgs::JointTrajectoryPoint msg;
    last_q_= cartesian_pose_handle_->getRobotState().q_d;
    last_q_d_= cartesian_pose_handle_->getRobotState().dq_d;
    for(int i = 0; i < 7; i++){
      msg.positions.push_back(last_q_[i]);
      msg.velocities.push_back(last_q_d_[i]);
    }

    joints_publisher_.publish(msg);


  }

  void CartesianPoseController::CartesianTrajectoryCB(
    const trajectory_msgs::MultiDOFJointTrajectoryPointConstPtr& msg)
    {
        // Comando in posizione
        position_d_ << msg->transforms[0].translation.x,
                       msg->transforms[0].translation.y,
                       msg->transforms[0].translation.z;

        Eigen::Quaterniond last_orientation_d(orientation_d_);

        // Comando in orientamento
        orientation_d_.coeffs() << msg->transforms[0].rotation.x,
                                   msg->transforms[0].rotation.y,
                                   msg->transforms[0].rotation.z,
                                   msg->transforms[0].rotation.w;

        if (last_orientation_d.coeffs().dot(orientation_d_.coeffs()) < 0.0)
          orientation_d_.coeffs() << -orientation_d_.coeffs();
        
        
    }
} // namespace controllers

PLUGINLIB_EXPORT_CLASS(controllers::CartesianPoseController,
                       controller_interface::ControllerBase)
