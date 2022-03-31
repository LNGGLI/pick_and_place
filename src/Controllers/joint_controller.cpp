// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE

#include <pick_and_place/Controllers/joint_controller.h>

#include <cmath>

#include <realtime_tools/realtime_buffer.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace JointController {


// Semaforo
  std::mutex traj_mutex;

void set_command(const std::array<double,7>& joint_configuration){
  for(int i = 0; i < 7; i++){
    position_joint_handles_[i].setCommand(joint_configuration[i]);
  }
}

bool set_traj(pick_and_place::SetTraj::Request &req,
              pick_and_place::SetTraj::Response &resp) {


  // Lock               
  traj_mutex.lock(); 

  // Lettura posa finale e tempo della traiettoria
  TooN::Vector<3, double> final_position = TooN::makeVector(
      req.goal_position.x, req.goal_position.y, req.goal_position.z);

  TooN::Vector<4, double> final_quaternion =
      TooN::makeVector(req.goal_quaternion.w, req.goal_quaternion.x,
                       req.goal_quaternion.y, req.goal_quaternion.z);
  double Tf = req.Tf;
  
  std::array<double, 16> pose = current_pose_;
  // cartesian_pose_handle_->getRobotState().O_T_EE_c; //  

  TooN::Matrix<4, 4, double> toon_current_pose = TooN::Data(
      pose[0], pose[4], pose[8], pose[12],
      pose[1], pose[5], pose[9], pose[13],
      pose[2], pose[6], pose[10], pose[14],
      pose[3], pose[7], pose[11], pose[15]);

  // Ricavo posizione e quaternione attuale in base alla posa letta
  TooN::Vector<3> current_position = sun::transl(toon_current_pose);
  sun::UnitQuaternion current_quat(toon_current_pose);

  // Calcolo Delta_quaternione per la traiettoria in orientamento
  sun::UnitQuaternion final_quat(final_quaternion);
  sun::UnitQuaternion delta_quat = final_quat * inv(current_quat); // errore in terna base
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

  delete cartesian_traj_;
  cartesian_traj_ = local_traj.clone();
  if (cartesian_traj_ != nullptr) {
    resp.success = true;
    start = true;
    elapsed_time_ = ros::Duration(0.0);
  } else {
    std::cout << "Errore nella creazione della traiettoria in \"set_traj\" \n";
    resp.success = false;
  }

  elapsed_time_ = ros::Duration(0.0);

  traj_mutex.unlock();
  return true;
}


bool JointController::init(hardware_interface::RobotHW* robot_hardware,
                                          ros::NodeHandle& node_handle) {
  position_joint_interface_ = robot_hardware->get<hardware_interface::PositionJointInterface>();
  if (position_joint_interface_ == nullptr) {
    ROS_ERROR(
        "JointController: Error getting position joint interface from hardware!");
    return false;
  }
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names)) {
    ROS_ERROR("JointController: Could not parse joint names");
  }
  if (joint_names.size() != 7) {
    ROS_ERROR_STREAM("JointController: Wrong number of joint names, got "
                     << joint_names.size() << " instead of 7 names!");
    return false;
  }
  position_joint_handles_.resize(7);
  for (size_t i = 0; i < 7; ++i) {
    try {
      position_joint_handles_[i] = position_joint_interface_->getHandle(joint_names[i]);
    } catch (const hardware_interface::HardwareInterfaceException& e) {
      ROS_ERROR_STREAM(
          "JointController: Exception getting joint handles: " << e.what());
      return false;
    }
  }

  std::string arm_id;
  auto *state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM(
        "JointController: Error getting state interface from hardware");
    return false;
  }
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException &ex) {
    ROS_ERROR_STREAM(
        "JointController: Exception getting state handle from interface: "
        << ex.what());
    return false;
  }

  
  server_set_traj_ = node_handle.advertiseService("/set_traj", set_traj);

  return true;
}

void JointController::starting(const ros::Time& /* time */) {

  franka::RobotState initial_state = state_handle_->getRobotState();
  current_pose_ = initial_state.O_T_EE;

  for (std::size_t i = 0; i < 7; ++i)
    current_configuration_[i] = position_joint_handles_[i].getPosition();

  set_command(current_configuration_);

  elapsed_time_ = ros::Duration(0.0);

}


void JointController::update(const ros::Time& /*time*/,
                                            const ros::Duration& period) {

  if (!start || !traj_mutex.try_lock()) 
  { 
    // Il controller è avviato ma non è stata ancora assegnata
    // nessuna posa desiderata. Oppure è in corso la creazione della
    // traiettoria.
    set_command(current_configuration_);
  } 
  else 
  {

    
    set_command(current_configuration_);
    traj_mutex.unlock();

  }




}
  
 


}  // namespace JointController

PLUGINLIB_EXPORT_CLASS(JointController::JointController,
                       controller_interface::ControllerBase)

// PLUGINLIB_EXPORT_CLASS(name_of_your_controller_package::NameOfYourControllerClass,
//                      controller_interface::ControllerBase)
