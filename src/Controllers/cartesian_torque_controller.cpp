// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <pick_and_place/Controllers/cartesian_torque_controller.h>
#include <pick_and_place/SetTraj.h>

#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

// sun
#include "sun_math_toolbox/PortingFunctions.h"
#include <sun_traj_lib/Cartesian_Independent_Traj.h>
#include <sun_traj_lib/Line_Segment_Traj.h>
#include <sun_traj_lib/Quintic_Poly_Traj.h>
#include <sun_traj_lib/Rotation_Const_Axis_Traj.h>

#include <franka/robot_state.h>

namespace TorqueController {

bool set_traj(pick_and_place::SetTraj::Request &req,
              pick_and_place::SetTraj::Response &resp) {

  // Lettura posa finale e tempo della traiettoria
  TooN::Vector<3, double> final_position = TooN::makeVector(
      req.goal_position.x, req.goal_position.y, req.goal_position.z);
  TooN::Vector<4, double> final_quaternion =
      TooN::makeVector(req.goal_quaternion.w, req.goal_quaternion.x,
                       req.goal_quaternion.y, req.goal_quaternion.z);
  double Tf = req.Tf;

  // Lettura posa attuale
  std::array<double, 16> current_pose =
      cartesian_pose_handle_->getRobotState().O_T_EE; // Oppure O_T_EE_c

  TooN::Matrix<4, 4, double> toon_current_pose = TooN::Data(
      current_pose[0], current_pose[4], current_pose[8], current_pose[12],
      current_pose[1], current_pose[5], current_pose[9], current_pose[13],
      current_pose[2], current_pose[6], current_pose[10], current_pose[14],
      current_pose[3], current_pose[7], current_pose[11], current_pose[15]);

  // Ricavo posizione e quaternione attuale in base alla posa letta
  TooN::Vector<3> current_position = sun::transl(toon_current_pose);
  sun::UnitQuaternion current_quat(toon_current_pose);

  // Calcolo Delta_quaternione per la traiettoria in orientamento
  sun::UnitQuaternion final_quat(final_quaternion);
  sun::UnitQuaternion delta_quat =
      final_quat * inv(current_quat); // errore in terna base
  sun::AngVec angvec = delta_quat.toangvec();

  // Generazione della traiettoria
  sun::Quintic_Poly_Traj qp_position(
      Tf, 0.0, 1.0); // polinomio quintico utilizzato per line_traj
  sun::Quintic_Poly_Traj qp_orientation(
      Tf, 0.0, angvec.getAng()); // polinomio quintico utilizzato per quat_traj
  sun::Line_Segment_Traj line_traj(
      current_position, final_position,
      qp_position); // Traiettoria in posizione su percorso di tipo segmento
  sun::Rotation_Const_Axis_Traj quat_traj(
      current_quat, angvec.getVec(),
      qp_orientation); // Traiettoria in orientamento: rotazione attorno asse
                       // fisso nel tempo
  sun::Cartesian_Independent_Traj local_traj(line_traj, quat_traj);

  cartesian_traj_ = local_traj.clone();
  if (cartesian_traj_ != nullptr) {
    resp.success = true;
    start = true;
    elapsed_time_ = ros::Duration(0.0);
  } else {
    std::cout << "Errore nella creazione della traiettoria in \"set_traj\" \n";
    resp.success = false;
  }

  return true;
}
bool CartesianTorqueController::init(hardware_interface::RobotHW *robot_hw,
                                     ros::NodeHandle &node_handle) {

  // ESTRAZIONE DEI PARAMETRI DAL SERVER DEI PARAMETRI

  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("CartesianTorqueController: Could not read parameter arm_id");
    return false;
  }

  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names) ||
      joint_names.size() != 7) {
    ROS_ERROR("CartesianTorqueController: Invalid or no joint_names parameters "
              "provided, aborting "
              "controller init!");
    return false;
  }

  double publish_rate(30.0);
  if (!node_handle.getParam("publish_rate", publish_rate)) {
    ROS_INFO_STREAM(
        "CartesianTorqueController: publish_rate not found. Defaulting to "
        << publish_rate);
  }
  rate_trigger_ = franka_hw::TriggerRate(publish_rate);

  if (!node_handle.getParam("k_gains", k_gains_) || k_gains_.size() != 7) {
    ROS_ERROR("JointImpedanceExampleController:  Invalid or no k_gain "
              "parameters provided, aborting "
              "controller init!");
    return false;
  }

  if (!node_handle.getParam("d_gains", d_gains_) || d_gains_.size() != 7) {
    ROS_ERROR("JointImpedanceExampleController:  Invalid or no d_gain "
              "parameters provided, aborting "
              "controller init!");
    return false;
  }

  if (!node_handle.getParam("coriolis_factor", coriolis_factor_)) {
    ROS_INFO_STREAM("JointImpedanceExampleController: coriolis_factor not "
                    "found. Defaulting to "
                    << coriolis_factor_);
  }

  // CREAZIONE DEGLI HANDLER

  // 1. model_handle_
  auto *model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM("CartesianTorqueController: Error getting model interface "
                     "from hardware");
    return false;
  }
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException &ex) {
    ROS_ERROR_STREAM("CartesianTorqueController: Exception getting model "
                     "handle from interface: "
                     << ex.what());
    return false;
  }

  // 2. cartesian_pose_handle_
  auto *cartesian_pose_interface =
      robot_hw->get<franka_hw::FrankaPoseCartesianInterface>();
  if (cartesian_pose_interface == nullptr) {
    ROS_ERROR_STREAM("CartesianTorqueController: Error getting cartesian pose "
                     "interface from hardware");
    return false;
  }
  try {
    cartesian_pose_handle_ =
        std::make_unique<franka_hw::FrankaCartesianPoseHandle>(
            cartesian_pose_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException &ex) {
    ROS_ERROR_STREAM("CartesianTorqueController: Exception getting cartesian "
                     "pose handle from interface: "
                     << ex.what());
    return false;
  }

  // 3. joint_handles_
  auto *effort_joint_interface =
      robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM("CartesianTorqueController: Error getting effort joint "
                     "interface from hardware");
    return false;
  }

  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(
          effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException &ex) {
      ROS_ERROR_STREAM(
          "CartesianTorqueController: Exception getting joint handles: "
          << ex.what());
      return false;
    }
  }



   server_set_traj_ = node_handle.advertiseService("/set_traj", set_traj);
  torques_publisher_.init(node_handle, "torque_comparison", 1);
  std::fill(dq_filtered_.begin(), dq_filtered_.end(), 0);
  return true;
}

void CartesianTorqueController::starting(const ros::Time & /*time*/) {
  pose_ = cartesian_pose_handle_->getRobotState().O_T_EE;
  elapsed_time_ =
      ros::Duration(0.0); // Ogni volta che il controller viene avviato
}

void CartesianTorqueController::update(const ros::Time & /*time*/,
                                       const ros::Duration &period) {

  elapsed_time_ += period;

  // Il comando in cartesiano non viene eseguito ma viene utilizzato per
  // calcolare le variabili di giunto corrispondenti attraverso inversione
  // cinematica.

  if (!start) { // il controller è partito ma non è stata ancora assegnata
                // nessuna posa desiderata.
    cartesian_pose_handle_->setCommand(pose_);
  } else {

    // Posizione
    TooN::Vector<3, double> position =
        cartesian_traj_->getPosition(elapsed_time_.toSec());
    pose_[12] = position[0]; // x
    pose_[13] = position[1]; // y
    pose_[14] = position[2]; // z

    // Orientamento
    TooN::Matrix<3, 3, double> R =
        cartesian_traj_->getQuaternion(elapsed_time_.toSec()).R();

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
  }

  franka::RobotState robot_state = cartesian_pose_handle_->getRobotState();
  std::array<double, 7> coriolis = model_handle_->getCoriolis();
  std::array<double, 7> gravity = model_handle_->getGravity();
  
  std::array<double, 7> tau_d_calculated;

  double alpha = 0.99;

  for (size_t i = 0; i < 7; i++)
    dq_filtered_[i] = (1 - alpha) * dq_filtered_[i] + alpha * robot_state.dq[i];

  for (size_t i = 0; i < 7; ++i) {
    tau_d_calculated[i] =
        coriolis_factor_ * coriolis[i] +
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

  // Maximum torque difference with a sampling rate of 1 kHz. The maximum torque
  // rate is 1000 * (1 / sampling_time).
  std::array<double, 7> tau_d_saturated =
      saturateTorqueRate(tau_d_calculated, robot_state.tau_J_d);

  for (size_t i = 0; i < 7; ++i)
    joint_handles_[i].setCommand(tau_d_saturated[i]);


  // Esempio di pubblicazione realtime
  
  /*if (rate_trigger_() && publisher.trylock())
  {
      publisher.msg_.data = 2;
      publisher.unlockAndPublish();
  }*/
      

  // Reale pubblicazione realtime su topic

  if (rate_trigger_() && torques_publisher_.trylock())
  {
      std::array<double, 7> tau_j = robot_state.tau_J;
      std::array<double, 7> tau_error;
      double error_rms(0.0);
      for (size_t i = 0; i < 7; ++i)
      {
          tau_error[i] = last_tau_d_[i] - tau_j[i];
          error_rms += std::sqrt(std::pow(tau_error[i], 2.0)) / 7.0;
      }
      torques_publisher_.msg_.root_mean_square_error = error_rms;
      for (size_t i = 0; i < 7; ++i)
      {
          torques_publisher_.msg_.tau_commanded[i] = last_tau_d_[i];
          torques_publisher_.msg_.tau_error[i] = tau_error[i];
          torques_publisher_.msg_.tau_measured[i] = tau_j[i];
      }
      torques_publisher_.unlockAndPublish();
  }

  for (size_t i = 0; i < 7; ++i)
  {
      last_tau_d_[i] = tau_d_saturated[i] + gravity[i];
  }
}

std::array<double, 7> CartesianTorqueController::saturateTorqueRate(
    const std::array<double, 7> &tau_d_calculated,
    const std::array<double, 7>
        &tau_J_d) { // NOLINT (readability-identifier-naming)
  std::array<double, 7> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
    double difference = tau_d_calculated[i] - tau_J_d[i];
    tau_d_saturated[i] =
        tau_J_d[i] +
        std::max(std::min(difference, kDeltaTauMax), -kDeltaTauMax);
  }
  return tau_d_saturated;
}

} // namespace TorqueController

PLUGINLIB_EXPORT_CLASS(TorqueController::CartesianTorqueController,
                       controller_interface::ControllerBase)
