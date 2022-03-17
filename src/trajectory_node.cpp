
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include <pick_and_place/Panda.h>
#include <pick_and_place/trajectory_node.h>

// Server , Actions
#include <actionlib/client/simple_action_client.h>
#include <franka_gripper/GraspAction.h>
#include <franka_gripper/HomingAction.h>
#include <franka_gripper/MoveAction.h>
#include <franka_gripper/StopAction.h>
#include <pick_and_place/SetTraj.h>

// Utils
#include <pick_and_place/check_realtime.h>
#include <ros/ros.h>

// Sun
#include <sun_math_toolbox/UnitQuaternion.h>

// Messages
#include <franka_msgs/FrankaState.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>

/* rosbag record /jointsIK /cartesian_trajectory_command
 /franka_state_controller/joint_states_desired
 /franka_state_controller/franka_states*/

using namespace trajectory;

// Posizioni [m]
TooN::Vector<3, double> High_center = TooN::makeVector(0.5, 0.0, 0.3);

TooN::Vector<3, double> BH1_S = TooN::makeVector(
    0.6872160179268519, -0.14076322774510802, 0.022745403230982633);

TooN::Vector<3, double> BH2_S = TooN::makeVector(
    0.617426846065691 , -0.139827712624430  , 0.022430527407691);

TooN::Vector<3, double> BH3_S = TooN::makeVector(
    0.547637674204531 , -0.138892197503752  , 0.022115651584400);

TooN::Vector<3, double> BH4_S = TooN::makeVector(
    0.47784850234337045, -0.13795668238307343, 0.02180077576110917);

TooN::Vector<3, double> BH1_G = TooN::makeVector(
    0.4486985001162725, 0.22299845949006336, 0.02163288252981005 + 0.002);



TooN::Vector<3, double> BH2_G = TooN::makeVector(
    0.5447036303651978, 0.1477191846817073, 0.022160634657607883 + 0.002);

TooN::Vector<3, double> BH3_G = TooN::makeVector(
    0.46847657797357173, 0.04301584096398785, 0.02072385873126041 + 0.002);

TooN::Vector<3, double> BH4_G = TooN::makeVector(
    0.669827234358595, 0.18942833873609624, 0.023672426228587645 + 0.002);    


TooN::Matrix<3, 3, double> goal_R = TooN::Data(1, 0, 0,
                                                 0, -1, 0,
                                                 0, 0, -1);


int main(int argc, char **argv) {

  ros::init(argc, argv, "trajectory_node");
  ros::NodeHandle nh;

  client_set_traj = nh.serviceClient<pick_and_place::SetTraj>("/set_traj");
  
  ros::Subscriber state_sub = nh.subscribe<franka_msgs::FrankaState>(
      "/franka_state_controller/franka_states", 1, stateCB);

  // Check e set realtime node
  if (!check_realtime())
    throw std::runtime_error("REALTIME NOT AVAILABLE");

  if (!set_realtime_SCHED_FIFO())
    throw std::runtime_error("ERROR IN set_realtime_SCHED_FIFO");

  

  // Accensione del controller
  if (!switch_controller("pick_and_place_controller", "")) {
    std::cout << "Lo switch del controller non è andato a buon fine!"
              << std::endl;
    return -1;
  }

  //Homing gripper
  // if (!gripper_homing())
    // return -1;
 

  
  double width = 0.07;
  double speed = 0.05;
  gripper_move(width, speed);

  // Pick della vite lato filettato
  if(!pick_vite(BH1_S, 10)){ // 10 [s] 
    std::cout << "Il robot non è riuscito a raccogliere la vite \n";
    return -1;
  }
    

  // Movimento del robot in alto al centro
  
  pose_goal.goal_position = High_center;
  pose_goal.goal_quaternion = sun::UnitQuaternion(goal_R);
  pose_goal.Tf = 5; // [s]
  set_goal_and_call_srv(pose_goal);


  // Place della vite
  if(!place_vite(BH1_G,10)){ // 15 [s]
    std::cout << "Il robot non è riuscito a poggiare la vite sul banco \n";
    return -1;
  }

  // Movimento del robot per tornare in alto
  pose_goal.goal_position = High_center;
  pose_goal.goal_quaternion = sun::UnitQuaternion(goal_R);
  pose_goal.Tf = 5; // [s]
  set_goal_and_call_srv(pose_goal);
  
  // Spegnimento del controller
  if (!switch_controller("", "pick_and_place_controller")) {
    std::cout << "Lo switch del controller non è andato a buon fine!"
              << std::endl;
    return -1;
  }

  return 0;
}
