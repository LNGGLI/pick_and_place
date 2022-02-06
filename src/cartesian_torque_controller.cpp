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

        return true;
    }

    void CartesianTorqueController::starting(const ros::Time & /*time*/)
    {
        initial_pose_ = cartesian_pose_handle_->getRobotState().O_T_EE_d;
    }

    void CartesianTorqueController::update(const ros::Time & /*time*/,
                                           const ros::Duration &period)
    {

        std::array<double, 16> pose_desired = initial_pose_;

        // Il comando in cartesiano non viene eseguito ma viene utilizzato per calcolare
        // le variabili di giunto corrispondenti attraverso inversione cinematica.
        cartesian_pose_handle_->setCommand(pose_desired);

        franka::RobotState robot_state = cartesian_pose_handle_->getRobotState();
        std::array<double, 7> coriolis = model_handle_->getCoriolis();
        std::array<double, 7> gravity = model_handle_->getGravity();

        std::array<double, 7> tau_d_calculated;
        /*
        for (size_t i = 0; i < 7; ++i) {
        tau_d_calculated[i] = legge di controllo con 
                                - robot_state.q_d[i] , robot_state.q[i];  
                                - robot_state.dq_d[i] , robot_state.dq[i];
                                Nota: 
                                1. q_d è il valore della variabile di giunto desiderato
                                        ottenuto da inversione cinematica.
                                2. q è il valore misurato della variabile di giunto.
        }
        */

        for (size_t i = 0; i < 7; ++i)
        {
            joint_handles_[i].setCommand(tau_d_calculated[i]);
        }

        // Esempio di pubblicazione realtime
        /*
        if (rate_trigger_() && publisher.trylock())
        {
            publisher.msg_.data = 2;
            publisher.unlockAndPublish();
        }
            */
    }

    void CartesianTorqueController::CartesianTrajectoryCB(
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

PLUGINLIB_EXPORT_CLASS(controllers::CartesianTorqueController,
                       controller_interface::ControllerBase)
