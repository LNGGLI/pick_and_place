// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <pick_and_place/Controllers/cartesian_impedance_controller.h>

#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>

#include <pick_and_place/pseudo_inversion.h>

namespace controllers
{

  bool CartesianImpedanceController::init(hardware_interface::RobotHW *robot_hw,
                                          ros::NodeHandle &node_handle)
  {
    


    std::string arm_id;
    if (!node_handle.getParam("arm_id", arm_id))
    {
      ROS_ERROR_STREAM("CartesianImpedanceController: Could not read parameter arm_id");
      return false;
    }
    std::vector<std::string> joint_names;
    if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7)
    {
      ROS_ERROR(
          "CartesianImpedanceController: Invalid or no joint_names parameters provided, "
          "aborting controller init!");
      return false;
    }

    auto *model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
    if (model_interface == nullptr)
    {
      ROS_ERROR_STREAM(
          "CartesianImpedanceController: Error getting model interface from hardware");
      return false;
    }
    try
    {
      model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
          model_interface->getHandle(arm_id + "_model"));
    }
    catch (hardware_interface::HardwareInterfaceException &ex)
    {
      ROS_ERROR_STREAM(
          "CartesianImpedanceController: Exception getting model handle from interface: "
          << ex.what());
      return false;
    }

    auto *state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
    if (state_interface == nullptr)
    {
      ROS_ERROR_STREAM(
          "CartesianImpedanceController: Error getting state interface from hardware");
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
          "CartesianImpedanceController: Exception getting state handle from interface: "
          << ex.what());
      return false;
    }

    auto *effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
    if (effort_joint_interface == nullptr)
    {
      ROS_ERROR_STREAM(
          "CartesianImpedanceController: Error getting effort joint interface from hardware");
      return false;
    }
    for (size_t i = 0; i < 7; ++i)
    {
      try
      {
        joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
      }
      catch (const hardware_interface::HardwareInterfaceException &ex)
      {
        ROS_ERROR_STREAM(
            "CartesianImpedanceController: Exception getting joint handles: " << ex.what());
        return false;
      }
    }

    position_d_.setZero();
    orientation_d_.coeffs() << 0.0, 0.0, 0.0, 1.0;

    cartesian_stiffness_.setIdentity();
    cartesian_stiffness_(0,0) = 200.0;
    cartesian_stiffness_(1,1) = 10.0;
    cartesian_stiffness_(2,2) = 200.0;
    cartesian_stiffness_(3,3) = 10.0;
    cartesian_stiffness_(4,4) = 10.0;
    cartesian_stiffness_(5,5) = 30.0;

    cartesian_damping_ = cartesian_stiffness_;
    


    sub_cartesian_trajectory_ = node_handle.subscribe(
        "/cartesian_trajectory_command", 20, &CartesianImpedanceController::CartesianTrajectoryCB, this,
        ros::TransportHints().reliable().tcpNoDelay());

    return true;
  }

  void CartesianImpedanceController::starting(const ros::Time & /*time*/)
  {
    // compute initial velocity with jacobian and set x_attractor and q_d_nullspace
    // to initial configuration
    
    franka::RobotState initial_state = state_handle_->getRobotState();
    // get jacobian
    std::array<double, 42> jacobian_array =
        model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
    // convert to eigen
    Eigen::Map<Eigen::Matrix<double, 7, 1>> q_initial(initial_state.q.data());
    Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));

    // set equilibrium point to current state
    position_d_ = initial_transform.translation();
    orientation_d_ = Eigen::Quaterniond(initial_transform.linear());

     std::cout << "Posizione iniziale: \n"
               << "x:" <<  position_d_[0] << "\n"
               << "y:" <<  position_d_[1] << "\n"
               << "z:" <<  position_d_[2] << "\n";


  }

  void CartesianImpedanceController::update(const ros::Time & /*time*/,
                                            const ros::Duration & /*period*/)
  {
    // get state variables
    franka::RobotState robot_state = state_handle_->getRobotState();
    std::array<double, 7> coriolis_array = model_handle_->getCoriolis();
    std::array<double, 42> jacobian_array =
        model_handle_->getZeroJacobian(franka::Frame::kEndEffector);

    // convert to Eigen
    Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
    Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
    Eigen::Map<Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
    Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
    Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d( // NOLINT (readability-identifier-naming)
        robot_state.tau_J_d.data());
    Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
    Eigen::Vector3d position(transform.translation());
    Eigen::Quaterniond orientation(transform.linear());


    std::cout << "Posizione traiettoria: \n"
               << "x:" <<  position_d_[0] << "\n"
               << "y:" <<  position_d_[1] << "\n"
               << "z:" <<  position_d_[2] << "\n";
    
    

    // compute error to desired pose
    // position error
    Eigen::Matrix<double, 6, 1> error;
    error.head(3) << position - position_d_;

    // orientation error
    if (orientation_d_.coeffs().dot(orientation.coeffs()) < 0.0)
    {
      orientation.coeffs() << -orientation.coeffs();
    }
    // "difference" quaternion
    Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d_);
    error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
    // Transform to base frame
    error.tail(3) << -transform.linear() * error.tail(3);


    std::cout << "\nErrore traiettoria: \n";
    for(int i = 0 ; i < 6; i++)
      printf("La componente %d dell'errore e': %f \n",i,error(i,0));
           
    // compute control
    // allocate variables
    Eigen::VectorXd tau_task(7), tau_d(7);

    // Cartesian PD control with damping ratio = 1
    tau_task << jacobian.transpose() *
                    (-cartesian_stiffness_ * error - cartesian_damping_ * (jacobian * dq));


    std::cout << "Tau task: \n";
   
    for(int i = 0 ; i < 7; i++)
      printf("La componente %d di Tau task e': %f \n",i,tau_task(i,0));



    // Desired torque
    tau_d << tau_task;// + coriolis;
    // Saturate torque rate to avoid discontinuities
    tau_d << saturateTorqueRate(tau_d, tau_J_d);
    for (size_t i = 0; i < 7; ++i)
    {
      joint_handles_[i].setCommand(tau_d(i));
      std::cout << "Ho comandato: tau_d:" << tau_d(i) << "\n";
    }


   
  }

  Eigen::Matrix<double, 7, 1> CartesianImpedanceController::saturateTorqueRate(
      const Eigen::Matrix<double, 7, 1> &tau_d_calculated,
      const Eigen::Matrix<double, 7, 1> &tau_J_d)
  { // NOLINT (readability-identifier-naming)
    Eigen::Matrix<double, 7, 1> tau_d_saturated{};
    for (size_t i = 0; i < 7; i++)
    {
      double difference = tau_d_calculated[i] - tau_J_d[i];
      tau_d_saturated[i] =
          tau_J_d[i] + std::max(std::min(difference, delta_tau_max_), -delta_tau_max_);
    }
    return tau_d_saturated;
  }

  void CartesianImpedanceController::CartesianTrajectoryCB(
        const trajectory_msgs::MultiDOFJointTrajectoryPointConstPtr &msg)
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

PLUGINLIB_EXPORT_CLASS(controllers::CartesianImpedanceController,
                       controller_interface::ControllerBase)
