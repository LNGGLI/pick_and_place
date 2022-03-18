
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include <pick_and_place/calibration_node.h>

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
#include <big_head/Point2DStamped.h>

/* rosbag record /jointsIK /cartesian_trajectory_command
 /franka_state_controller/joint_states_desired
 /franka_state_controller/franka_states*/

using namespace trajectory;

// Posizioni [m]
TooN::Vector<3, double> center = TooN::makeVector(0.5, 0.0, 0.195);


TooN::Matrix<3, 3, double> goal_R = TooN::Data(1, 0, 0,
                                                 0, -1, 0,
                                                 0, 0, -1);
// 
// TooN::Matrix<3, 3, double> goal_R = TooN::Data(0, 1, 0,
//                                                  0, 0, 1,
//                                                  1, 0, 0);



int main(int argc, char **argv) {

  ros::init(argc, argv, "calibration_node");
  ros::NodeHandle nh;

  client_set_traj = nh.serviceClient<pick_and_place::SetTraj>("/set_traj");
  
  ros::Subscriber state_sub = nh.subscribe<franka_msgs::FrankaState>(
      "/franka_state_controller/franka_states", 1, stateCB);

  ros::Subscriber contact_sub = nh.subscribe<big_head::Point2DStamped>(
      "/contact_point_F110", 1, contactCB);

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


  // Movimento del robot in modo che la base della vite tocchi al centro del tattile
  
  pose_goal.goal_position = center;
  pose_goal.goal_quaternion = sun::UnitQuaternion(goal_R);
  pose_goal.Tf = 7; // [s]
  set_goal_and_call_srv(pose_goal);

  press_y_gripper();
  x_calibration();

  pose_goal.goal_position = center;
  pose_goal.goal_quaternion = sun::UnitQuaternion(goal_R);
  pose_goal.Tf = 7; // [s]
  set_goal_and_call_srv(pose_goal);
  
  y_calibration();



  

  return 0;
}
