// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <pick_and_place/cartesian_torque_controller.h>

#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <franka/robot_state.h>

namespace controllers
{

    bool CartesianTorqueController::init(hardware_interface::RobotHW *robot_hw,
                                         ros::NodeHandle &node_handle)
    {

        // ESTRAZIONE DEI PARAMETRI DAL SERVER DEI PARAMETRI

        std::string arm_id;
        if (!node_handle.getParam("arm_id", arm_id))
        {
            ROS_ERROR("CartesianTorqueController: Could not read parameter arm_id");
            return false;
        }

        std::vector<std::string> joint_names;
        if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7)
        {
            ROS_ERROR(
                "CartesianTorqueController: Invalid or no joint_names parameters provided, aborting "
                "controller init!");
            return false;
        }

        double publish_rate(30.0);
        if (!node_handle.getParam("publish_rate", publish_rate))
        {
            ROS_INFO_STREAM("CartesianTorqueController: publish_rate not found. Defaulting to "
                            << publish_rate);
        }
        rate_trigger_ = franka_hw::TriggerRate(publish_rate);

        if (!node_handle.getParam("k_gains", k_gains_) || k_gains_.size() != 7)
        {
            ROS_ERROR(
                "JointImpedanceExampleController:  Invalid or no k_gain parameters provided, aborting "
                "controller init!");
            return false;
        }

        if (!node_handle.getParam("d_gains", d_gains_) || d_gains_.size() != 7)
        {
            ROS_ERROR(
                "JointImpedanceExampleController:  Invalid or no d_gain parameters provided, aborting "
                "controller init!");
            return false;
        }

        if (!node_handle.getParam("coriolis_factor", coriolis_factor_))
        {
            ROS_INFO_STREAM("JointImpedanceExampleController: coriolis_factor not found. Defaulting to "
                            << coriolis_factor_);
        }

        // CREAZIONE DEGLI HANDLER

        // 1. model_handle_
        auto *model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
        if (model_interface == nullptr)
        {
            ROS_ERROR_STREAM(
                "CartesianTorqueController: Error getting model interface from hardware");
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
                "CartesianTorqueController: Exception getting model handle from interface: "
                << ex.what());
            return false;
        }

        // 2. cartesian_pose_handle_
        auto *cartesian_pose_interface = robot_hw->get<franka_hw::FrankaPoseCartesianInterface>();
        if (cartesian_pose_interface == nullptr)
        {
            ROS_ERROR_STREAM(
                "CartesianTorqueController: Error getting cartesian pose interface from hardware");
            return false;
        }
        try
        {
            cartesian_pose_handle_ = std::make_unique<franka_hw::FrankaCartesianPoseHandle>(
                cartesian_pose_interface->getHandle(arm_id + "_robot"));
        }
        catch (hardware_interface::HardwareInterfaceException &ex)
        {
            ROS_ERROR_STREAM(
                "CartesianTorqueController: Exception getting cartesian pose handle from interface: "
                << ex.what());
            return false;
        }

        // 3. joint_handles_
        auto *effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
        if (effort_joint_interface == nullptr)
        {
            ROS_ERROR_STREAM(
                "CartesianTorqueController: Error getting effort joint interface from hardware");
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
                    "CartesianTorqueController: Exception getting joint handles: " << ex.what());
                return false;
            }
        }

        torques_publisher_.init(node_handle, "torque_comparison", 1);
        std::fill(dq_filtered_.begin(), dq_filtered_.end(), 0);
        return true;
    }

    void CartesianTorqueController::starting(const ros::Time & /*time*/)
    {
        initial_pose_ = cartesian_pose_handle_->getRobotState().O_T_EE_d;
    }

    void CartesianTorqueController::update(const ros::Time & /*time*/,
                                           const ros::Duration &period)
    {
        // Letto da topic position_d_  e orientation_d_ aggiornati

        pose_ = initial_pose_;
        pose_[12] = position_d_[0]; // x
        pose_[13] = position_d_[1]; // y
        pose_[14] = position_d_[2]; // z

        // Il comando in cartesiano non viene eseguito ma viene utilizzato per calcolare
        // le variabili di giunto corrispondenti attraverso inversione cinematica.
        cartesian_pose_handle_->setCommand(pose_);

        franka::RobotState robot_state = cartesian_pose_handle_->getRobotState();
        std::array<double, 7> coriolis = model_handle_->getCoriolis();
        std::array<double, 7> gravity = model_handle_->getGravity();

        std::array<double, 7> tau_d_calculated;
        
        double alpha = 0.99;

        for (size_t i = 0; i < 7; i++)
            dq_filtered_[i] = (1 - alpha) * dq_filtered_[i] + alpha * robot_state.dq[i];

        for (size_t i = 0; i < 7; ++i)
        {
            tau_d_calculated[i] = coriolis_factor_ * coriolis[i] +
                                  k_gains_[i] * (robot_state.q_d[i] - robot_state.q[i]) +
                                  d_gains_[i] * (robot_state.dq_d[i] - dq_filtered_[i]);
        }

        /*legge di controllo con 
                        - robot_state.q_d[i] , robot_state.q[i];  
                        - robot_state.dq_d[i] , robot_state.dq[i];
                        Nota: 
                        1. q_d è il valore della variabile di giunto desiderato
                                ottenuto dalla inversione cinematica.
                        2. q è il valore misurato della variabile di giunto.
        */


        // Maximum torque difference with a sampling rate of 1 kHz. The maximum torque rate is
        // 1000 * (1 / sampling_time).
        std::array<double, 7> tau_d_saturated = saturateTorqueRate(tau_d_calculated, robot_state.tau_J_d);

        for (size_t i = 0; i < 7; ++i)
            joint_handles_[i].setCommand(tau_d_saturated[i]);

        
        for (size_t i = 0; i < 7; ++i)
            joint_handles_[i].setCommand(tau_d_calculated[i]);

        // Esempio di pubblicazione realtime
        /*
        if (rate_trigger_() && publisher.trylock())
        {
            publisher.msg_.data = 2;
            publisher.unlockAndPublish();
        }
            */


        // Pubblicazione su topic delle coppie del robot

        // if (rate_trigger_() && torques_publisher_.trylock())
        // {
        //     std::array<double, 7> tau_j = robot_state.tau_J;
        //     std::array<double, 7> tau_error;
        //     double error_rms(0.0);
        //     for (size_t i = 0; i < 7; ++i)
        //     {
        //         tau_error[i] = last_tau_d_[i] - tau_j[i];
        //         error_rms += std::sqrt(std::pow(tau_error[i], 2.0)) / 7.0;
        //     }
        //     torques_publisher_.msg_.root_mean_square_error = error_rms;
        //     for (size_t i = 0; i < 7; ++i)
        //     {
        //         torques_publisher_.msg_.tau_commanded[i] = last_tau_d_[i];
        //         torques_publisher_.msg_.tau_error[i] = tau_error[i];
        //         torques_publisher_.msg_.tau_measured[i] = tau_j[i];
        //     }
        //     torques_publisher_.unlockAndPublish();
        // }

        // for (size_t i = 0; i < 7; ++i)
        // {
        //     last_tau_d_[i] = tau_d_saturated[i] + gravity[i];
        // }
    }

    std::array<double, 7> CartesianTorqueController::saturateTorqueRate(
        const std::array<double, 7> &tau_d_calculated,
        const std::array<double, 7> &tau_J_d)
    { // NOLINT (readability-identifier-naming)
        std::array<double, 7> tau_d_saturated{};
        for (size_t i = 0; i < 7; i++)
        {
            double difference = tau_d_calculated[i] - tau_J_d[i];
            tau_d_saturated[i] = tau_J_d[i] + std::max(std::min(difference, kDeltaTauMax), -kDeltaTauMax);
        }
        return tau_d_saturated;
    }

    void CartesianTorqueController::CartesianTrajectoryCB(
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

PLUGINLIB_EXPORT_CLASS(controllers::CartesianTorqueController,
                       controller_interface::ControllerBase)
