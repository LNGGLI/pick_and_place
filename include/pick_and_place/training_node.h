
#pragma once

#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

// Server , Actions
#include <actionlib/client/simple_action_client.h>
#include <controller_manager_msgs/ControllerState.h>
#include <controller_manager_msgs/SwitchController.h>
#include <franka_gripper/GraspAction.h>
#include <franka_gripper/HomingAction.h>
#include <franka_gripper/MoveAction.h>
#include <franka_gripper/StopAction.h>
#include <pick_and_place/SetTraj.h>

// Utils
#include <ros/ros.h>

// Sun
#include <sun_math_toolbox/UnitQuaternion.h>

// Messages
#include <TooN/TooN.h>
#include <big_head/Point2DStamped.h>
#include <franka_msgs/FrankaState.h>

// Controller manager srv
using controller_manager_msgs::SwitchControllerRequest;
using controller_manager_msgs::SwitchControllerResponse;

// Gripper action
using franka_gripper::HomingAction;
using HomingClient = actionlib::SimpleActionClient<HomingAction>;
using franka_gripper::MoveAction;
using MoveClient = actionlib::SimpleActionClient<MoveAction>;

namespace training {

// Variabili
struct CartesianGoal {
  TooN::Vector<3, double> goal_position;
  sun::UnitQuaternion goal_quaternion;
  double Tf;
};

CartesianGoal pose_goal;            // struct per definire il goal in cartesiano
ros::ServiceClient client_set_traj; // Client per srv set_traj
TooN::Vector<6, double> ext_wrench; // wrench misurato dal robot
bool traj_running = false; // false se non è in esecuzione nessuna traiettoria
TooN::Vector<2, double> contact_point;
TooN::Vector<2, double> force_indicator;

// Funzioni

void wait_movement() {

  std::cout << std::endl
            << "In attesa che il robot raggiunga la posa assegnata\n";
  ros::Rate loop_rate(100);
  while (ros::ok() && traj_running) {

    ros::spinOnce();
    loop_rate.sleep();
  }
  std::cout << "Il robot ha raggiunto la posa assegnata\n";
}

bool switch_controller(const std::string &start_controller,
                       const std::string &stop_controller) {

  SwitchControllerRequest switch_req;
  SwitchControllerResponse switch_res;

  switch_req.start_controllers.push_back(start_controller);
  switch_req.stop_controllers.push_back(stop_controller);
  switch_req.strictness = 1; // BEST_EFFORT means that even when something goes
                             // wrong with on controller,
  //     the service will still try to start/stop the remaining controllers "
  //  start the controllers as soon as their hardware dependencies are ready,
  // will "
  //   wait for all interfaces to be ready otherwise"
  switch_req.start_asap = true;
  switch_req.timeout = 0.0;
  ros::service::call("/controller_manager/switch_controller", switch_req,
                     switch_res);
  if (switch_res.ok == true)

    if (start_controller != "" && stop_controller != "")
      ROS_INFO_STREAM("Attivato " << start_controller << " e fermato "
                                  << stop_controller);

    else if (start_controller != "" && stop_controller == "")
      ROS_INFO_STREAM("Attivato " << start_controller);

    else if (start_controller == "" && stop_controller != "")
      ROS_INFO_STREAM("Fermato " << stop_controller);
    else
      ROS_INFO("Operazione switch non riuscita");

  return switch_res.ok;
}

// Chiama il servizio set_traj e aspetta che la traiettoria venga completata
bool set_goal_and_call_srv(const CartesianGoal &cartesian_goal) {

  pick_and_place::SetTraj set_traj_msg;

  set_traj_msg.request.goal_position.x = cartesian_goal.goal_position[0];
  set_traj_msg.request.goal_position.y = cartesian_goal.goal_position[1];
  set_traj_msg.request.goal_position.z = cartesian_goal.goal_position[2];

  set_traj_msg.request.goal_quaternion.w =
      cartesian_goal.goal_quaternion.getS();
  set_traj_msg.request.goal_quaternion.x =
      cartesian_goal.goal_quaternion.getV()[0];
  set_traj_msg.request.goal_quaternion.y =
      cartesian_goal.goal_quaternion.getV()[1];
  set_traj_msg.request.goal_quaternion.z =
      cartesian_goal.goal_quaternion.getV()[2];

  set_traj_msg.request.Tf = cartesian_goal.Tf;

  if (client_set_traj.call(set_traj_msg)) {
    if (set_traj_msg.response.success) {
      std::cout
          << "Il setting della traiettoria è stato effettuato correttamente\n";
      traj_running = true;
      wait_movement();

      return true;

    } else {
      std::cout << "Il setting della traiettoria non ha avuto successo \n";
      return false;
    }
  } else {
    std::cout << "Errore nella chiamata del servizio \n";
    return false;
  }
}

void stateCB(const franka_msgs::FrankaState::ConstPtr &msg) {

  double transform[16];
  for (int i = 0; i < 16; i++)
    transform[i] =
        msg->O_T_EE[i]; // ATTENZIONE la O_T_EE è passata per colonne!

  double R_array[9] = {transform[0], transform[1],
                       transform[2], // Array matrice di rotazione
                                     // (Column Major)
                       transform[4], transform[5], transform[6], transform[8],
                       transform[9], transform[10]};

  // Posizione attuale
  TooN::Vector<3, double> current_pos(
      {transform[12], transform[13], transform[14]});

  // Costruzione matrice di rotazione e quaternione
  TooN::Matrix<3, 3> R;
  for (int j = 0; j < 3; j++)
    for (int i = 0; i < 3; i++)
      R(i, j) = R_array[i + j * 3];

  sun::UnitQuaternion current_quat(R);

  // Calcolo errore
  double delta_p = TooN::norm(current_pos - pose_goal.goal_position);
  double delta_q_norm =
      TooN::norm((pose_goal.goal_quaternion * inv(current_quat)).getV());
  TooN::Vector<7, double> vel;
  for (int i = 0; i < 7; i++) {
    vel[i] = msg->dq[i];
  }

  // Calcolo norma accelerazione
  double vel_norm = TooN::norm(vel);

  // Misura forze di contatto
  for (int i = 0; i < 6; i++) {
    ext_wrench[i] = msg->K_F_ext_hat_K[i];
  }

  if (delta_p < 0.001 && delta_q_norm < 0.001 && vel_norm < 0.001)
    traj_running = false; // La traiettoria può considerarsi terminata
}

void forceCB(const big_head::Point2DStamped::ConstPtr &msg) {

  force_indicator[0] = msg->point.x;
  force_indicator[1] = msg->point.y;
}

bool press_y_gripper() {
  char carattere;

  while (ros::ok() && carattere != 'y') {
    std::cout << "Premere y per far muovere il gripper, n per abortire ";
    std::cin >> carattere;
    if (carattere == 'n')
      return false;

    std::cin.clear();
  };
}

// Homing del gripper
bool gripper_homing() {
  // Homing del gripper

  if (!press_y_gripper()) // premere y per continuare
    return false;
  HomingClient homing_client("/franka_gripper/homing");
  homing_client.waitForServer();
  homing_client.sendGoal(franka_gripper::HomingGoal());
  bool finished_before_timeout =
      homing_client.waitForResult(ros::Duration(20.0));
  if (finished_before_timeout) {
    franka_gripper::HomingResultConstPtr result = homing_client.getResult();
    if (result->success == true) {
      std::cout << "Homing avvenuto correttamente \n";
    } else {
      std::cout << "Homing fallito \n";
    }
    return result->success;
  } else {
    std::cout << "Homing action did not finish before the time out. \n";
    return false;
  }
}

// Move del gripper finchè le dita non sono a distanza width e alla velocità
// speed
bool gripper_move(const double &width,const double &speed) {

  MoveClient move_client("/franka_gripper/move");
  move_client.waitForServer();
  franka_gripper::MoveGoal move_goal;
  move_goal.width = width;
  move_goal.speed = speed;

  move_client.sendGoal(move_goal);
  bool finished_before_timeout = move_client.waitForResult(ros::Duration(10.0));

  if (finished_before_timeout) {
    franka_gripper::MoveResultConstPtr result = move_client.getResult();
    if (result->success == true) {
      std::cout << "Move avvenuta correttamente \n";
    } else {
      std::cout << "Move fallita \n";
    }

    return result->success;
  } else {
    std::cout << "Move action did not finish before the time out. \n";
    return false;
  }
}

// Grasp di un oggetto di larghezza width con velocità speed e forza force.
// Epsin e Epsout sono valori di tolleranza sulla larghezza dell'oggetto da
// graspare.
bool gripper_grasp(const double &width,const double &speed) {


  MoveClient move_client("/franka_gripper/move");
  move_client.waitForServer();
  franka_gripper::MoveGoal move_goal;
  move_goal.width = width;
  move_goal.speed = speed;

  move_client.sendGoal(move_goal);
  bool finished_before_timeout = move_client.waitForResult(ros::Duration(10.0));

  if (finished_before_timeout) {
    franka_gripper::MoveResultConstPtr result = move_client.getResult();
    if (result->success == true) {
      std::cout << "Grasp avvenuto correttamente \n";
    } else {
      std::cout << "Grasp fallito \n";
    }
    return result->success;
  } else {
    std::cout << "Grasp action did not finish before the time out. \n";
    return false;
  }
}

// Pick della vite lato filettato
bool pick_vite(const TooN::Vector<3, double> &pos,const  sun::UnitQuaternion &q,const double &Tf, const double &width) {

  pose_goal.goal_position = pos;
  pose_goal.goal_quaternion = q;
  pose_goal.Tf = Tf; // [s]

  set_goal_and_call_srv(pose_goal);

  // Grasp action del gripper lato filettato

  double grasp_width = width; // 0.006[m]
  double grasp_speed = 0.03;  // [m/s]
  

  if (!gripper_grasp(grasp_width, grasp_speed))
    return false;
}

TooN::Vector<2, double> compute_bias_tattile() {

  TooN::Vector<2, double> bias = TooN::Zeros;
  int Ncampioni = 100;
  ros::Rate lr(500);
  for (int i = 0; i < Ncampioni; i++) {
    ros::spinOnce();
    bias = bias + force_indicator;
    lr.sleep();
  }

  std::cout << "Force indicator bias computed \n";

  return bias / Ncampioni;
}

// Place della vite
bool place_vite(const TooN::Vector<3, double> &pos,const sun::UnitQuaternion &q, const  double &Tf) {

  TooN::Matrix<3, 3, double> goal_R = TooN::Data(1, 0, 0, 0, -1, 0, 0, 0, -1);
  pose_goal.goal_position = pos;
  pose_goal.goal_quaternion = sun::UnitQuaternion(goal_R);
  pose_goal.Tf = Tf; // [s]

  set_goal_and_call_srv(pose_goal);

  TooN::Vector<2, double> bias = compute_bias_tattile();
  while (TooN::norm(force_indicator - bias) < 0.3) {
    std::cout << "Norm force indicator: " << TooN::norm(force_indicator - bias)
              << "\n";
    pose_goal.goal_position -= TooN::makeVector(0.0, 0.0, 0.0001);
    pose_goal.Tf = 0.1;
    set_goal_and_call_srv(pose_goal);
    ros::spinOnce();
  }

  // Apertura del gripper
  double move_width = 0.05;  // [m]
  double move_speed = 0.015; // [m/s]
  if (!gripper_move(move_width, move_speed))
    return false;
}


bool start_training(TooN::Vector<3, double> &pos, sun::UnitQuaternion &q,  double &Tf){
  
  
  const double lunghezza_tattile = 0.0142;
  int x_samples = 10;
  int y_samples = 5;
  const double x_distance = lunghezza_tattile / x_samples;
  const double y_distance = (lunghezza_tattile/2 - 0.001) / y_samples;
  const TooN::Vector<6,double> angles = TooN::makeVector(0.0, 5.0 , 10.0 , 15.0 , 20.0 , 25.0);
  double current_angle = 0.0;
  const double width = 0.005;
  double current_width = width;
  const int move_width_samples = 1; 
  

  // Move the robot so that the screw's head touches the pad at (tactile frame): 
  // x = - 0.071 
  // y = + 0.001 
  TooN::Vector<3,double> current_pos = pos_vite + TooN::makeVector( (lunghezza_tattile / 2.0), 0.0, 0.001 );
  TooN::Matrix<3,3,double> current_orient = goal_R;
  
  pose_goal.goal_position = current_pos;
  pose_goal.goal_quaternion = sun::UnitQuaternion(current_orient);
  pose_goal.Tf = 10.0;
  set_goal_and_call_srv(pose_goal);  
  


  // Begin training loop
    
  
  //Positive rotation along the y axis (base frame)
  for (int x_index = 0; x_index < x_samples && ros::ok(); x_index++) {

    for (int y_index = 0; y_index < y_samples && ros::ok(); y_index++) {

        // The higher the contact point the wider the range of angles
        // we can use to grasp the screw.

        for(int angle_index = 0; angle_index <= y_index && ros::ok(); angle_index++){

            for (int width_index = 0; width_index < move_width_samples && ros::ok();
                width_index++) {

              if (width_index == 0) {

                current_pos =
                    pos_vite - TooN::makeVector(x_index * x_distance, 0.0,
                                                y_index * y_distance);

                current_width = width + width_index * 0.001;
                // Pick della vite
                if (!pick_vite(current_pos, q, Tf, current_width)) {
                  std::cout
                      << "Il robot non è riuscito a raccogliere la vite \n";
                  return -1;
                }

                // Movimento del robot per alzare la vite
                pose_goal.goal_position = High_center;
                pose_goal.goal_quaternion = sun::UnitQuaternion(goal_R);
                pose_goal.Tf = 5; // [s]
                set_goal_and_call_srv(pose_goal);

              } 
              else if (width_index < move_width_samples) {
                // If the screw has already been grasped you can grasp again
                // without placing the screw

                current_width = width + width_index * 0.001;
                gripper_grasp(current_width, 0.03); // [m] , [m/s]
                ros::spinOnce();
                // TODO: salvataggio su file

              } 
              
              else {
                // Place della vite
                if (!place_vite(pos_vite, q, Tf)) { // [s]
                  std::cout << "Il robot non è riuscito a poggiare la vite sul "
                               "banco \n";
                  return -1;
                }
              }

            }// width loop
        } // angle loop
    } // y loop
  } // x loop

  // Tornare con gripper ortogonale al piano

  // Negative rotation along the y axis (base frame)
  for (int angle_index = 0; angle_index < angles.size(); angle_index++) {


    }


  
}

} // namespace training