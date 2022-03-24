
#include <cmath>
#include <math.h>
#include <memory>
#include <stdexcept>
#include <string>

#include <pick_and_place/Panda.h>
#include <pick_and_place/training_node.h>

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
#include <big_head/Point2DStamped.h>
#include <franka_msgs/FrankaState.h>
#include <read_sensor/tactile_sensor_data.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>

/* rosbag record /jointsIK /cartesian_trajectory_command
 /franka_state_controller/joint_states_desired
 /franka_state_controller/franka_states*/

using namespace training;

TooN::Vector<3,double> High_center = TooN::makeVector(0.5609167289621934, 0.3418667768627687 , 0.3);

TooN::Matrix<3, 3, double> R_vite =
    TooN::Data(0.9992015325089199, 0.01580575242573673, 0.0364314160708723,
               0.016179945874301216, -0.9998094667944435, -0.00999941428324648,
               0.036266426409821945, 0.01058088841616233, -0.9992861270112097);

TooN::Vector<3, double> pos_vite = TooN::makeVector(
    0.5609167289621934, 0.3418667768627687, 0.02566054272474297);



int main(int argc, char **argv) {

  ros::init(argc, argv, "training_node");
  ros::NodeHandle nh;

  client_set_traj = nh.serviceClient<pick_and_place::SetTraj>("/set_traj");

  ros::Subscriber state_sub = nh.subscribe<franka_msgs::FrankaState>(
      "/franka_state_controller/franka_states", 1, stateCB);

  ros::Subscriber tactile_sub = nh.subscribe<read_sensor::tactile_sensor_data>(
      "/TactileData_dev_ttyUSB0_F110", 1, tactileCB);

  ros::Subscriber force_sub = nh.subscribe<big_head::Point2DStamped>(
      "/force_indicator_F110", 1, forceCB);

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



  // Movimento del robot in alto al centro
  pose_goal.goal_position = High_center;
  pose_goal.goal_quaternion = sun::UnitQuaternion(R_vite);
  pose_goal.Tf = 5; // [s]
  set_goal_and_call_srv(pose_goal);



  // pose_goal.goal_position = High_center;
  // pose_goal.goal_quaternion = sun::UnitQuaternion( R_vite * sun::roty(25*M_PI/180));
  // pose_goal.Tf = 4; // [s]
  // set_goal_and_call_srv(pose_goal);


  pose_goal.goal_position = pos_vite;
  pose_goal.goal_quaternion = sun::UnitQuaternion(R_vite);
  pose_goal.Tf = 10.0;
  set_goal_and_call_srv(pose_goal);

  build_dataset(pos_vite,R_vite);
  

  

  return 0;
}
