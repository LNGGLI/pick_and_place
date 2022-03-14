
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include <pick_and_place/trajectory_node.h>
#include <pick_and_place/Panda.h>

// Server , Actions
#include <pick_and_place/SetTraj.h>
#include <actionlib/client/simple_action_client.h>
#include <franka_gripper/GraspAction.h>
#include <franka_gripper/HomingAction.h>
#include <franka_gripper/MoveAction.h>
#include <franka_gripper/StopAction.h>

// Utils
#include <ros/ros.h>
#include <pick_and_place/check_realtime.h>



// Sun
#include <sun_math_toolbox/UnitQuaternion.h>

// Messages
#include <franka_msgs/FrankaState.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>


/* rosbag record /jointsIK /cartesian_trajectory_command
 /franka_state_controller/joint_states_desired 
 /franka_state_controller/franka_states*/


using namespace trajectory;

// Gripper action
using franka_gripper::GraspAction;
using GraspClient = actionlib::SimpleActionClient<GraspAction>;
using franka_gripper::HomingAction;
using HomingClient = actionlib::SimpleActionClient<HomingAction>;
using franka_gripper::MoveAction;
using MoveClient = actionlib::SimpleActionClient<MoveAction>;


// Posizioni
TooN::Vector<3,double> High_center = TooN::makeVector(0.4, 0.0, 0.4);

TooN::Vector<3,double> BH1_S = TooN::makeVector(0.6878383541320452, -0.14129504107227095, 0.022850597080517718);
TooN::Vector<3,double> BH1_G = TooN::makeVector(0.4492478934831226, 0.22222377189755013, 0.021709117575713865);


/*
Operazioni svolte dal nodo:
- Set realtime
- Esegue homing del gripper 
- Muove il gripper 
- Aziona il controller CartesianPose
- Comanda goal in cartesiano e aspetta che venga completato.
- Grasp
*/

int main(int argc, char **argv)
{

    ros::init(argc, argv, "trajectory_node");
    ros::NodeHandle nh;
    
    client_set_traj = nh.serviceClient<pick_and_place::SetTraj>("/set_traj");
    ros::Subscriber state_sub = nh.subscribe<franka_msgs::FrankaState>("/franka_state_controller/franka_states", 1, stateCB);
    
    
    HomingClient homing_client("/franka_gripper/homing");
    MoveClient move_client("/franka_gripper/move");
    GraspClient grasp_client("/franka_gripper/grasp");
    bool finished_before_timeout = false;

    // Check e set realtime node
    if (!check_realtime()) 
      throw std::runtime_error("REALTIME NOT AVAILABLE");

    if (!set_realtime_SCHED_FIFO()) 
        throw std::runtime_error("ERROR IN set_realtime_SCHED_FIFO");


    // Homing del gripper
    // if(!press_y_gripper()) // premere y per continuare
    //     return -1;
    // homing_client.sendGoal(franka_gripper::HomingGoal());
    // finished_before_timeout = homing_client.waitForResult(ros::Duration(10.0));
    // if (finished_before_timeout)
    // {
    //     actionlib::SimpleClientGoalState state = move_client.getState();
    //     ROS_INFO("Action finished: %s", state.toString().c_str());
    // }
    // else
    // {
    //     std::cout <<"Action did not finish before the time out. \n";
    // }

    // Move gripper ad una certa width
    // if(!press_y_gripper()) // premere y per continuare
    //     return -1;

    // franka_gripper::MoveGoal move_goal;
    // move_goal.width = 0.08;
    // move_goal.speed = 0.05;
    // move_client.sendGoal(move_goal);
    // finished_before_timeout = move_client.waitForResult(ros::Duration(10.0));
    // if (finished_before_timeout)
    // {
    //     actionlib::SimpleClientGoalState state = move_client.getState();
    //     ROS_INFO("Action finished: %s", state.toString().c_str());
    // }
    // else
    // {
    //     std::cout <<"Action did not finish before the time out. \n";
    // }

    // Accensione del controller
    bool success = switch_controller("cartesian_pose_controller", "");
    if (!success){
        std::cout << "Lo switch del controller non è andato a buon fine!" << std::endl;
        return -1;
    }
    
    // Costruzione e invio della posa desiderata
    TooN::Matrix<3,3,double> goal_R = TooN::Data(1,0,0,
                                                 0,-1,0,
                                                 0,0,-1);

    cartesian_goal.goal_position = BH1_S;                               
    cartesian_goal.goal_quaternion = sun::UnitQuaternion(goal_R);
    cartesian_goal.Tf = 20; // s 

    success = set_goal_and_call_srv(cartesian_goal);

    while(ros::ok() && traj_running && success){
        
        std::cout << "In attesa che il robot raggiunga la posa assegnata \n";
        ros::spinOnce();
        ros::Duration(5).sleep();

    }

    // Grasp action del gripper 
    if(!press_y_gripper()) // premere y per continuare
        return -1;

    // Parametri dell'azione di grasp
    franka_gripper::GraspGoal grasp_goal;
    grasp_goal.width = 0.005; // [m]
    grasp_goal.speed = 0.05;  // [m/s]
    grasp_goal.force = 40.0;  // [N]
    grasp_goal.epsilon.inner = 0.001;
    grasp_goal.epsilon.outer = 0.001;

    // Invio del comando
    grasp_client.sendGoal(grasp_goal);
    finished_before_timeout = grasp_client.waitForResult(ros::Duration(10.0));
    if (finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = grasp_client.getState();
        ROS_INFO("Action finished: %s", state.toString().c_str());
    }
    else
    {
        std::cout <<"Action did not finish before the time out. \n";
    }


    cartesian_goal.goal_position = High_center;
    cartesian_goal.goal_quaternion = sun::UnitQuaternion(goal_R);
    std::cout << cartesian_goal.goal_quaternion;
    cartesian_goal.Tf = 20; // s 

    success = set_goal_and_call_srv(cartesian_goal);

    while(ros::ok() && traj_running && success){
        
        std::cout << " In attesa che il robot raggiunga la posa assegnata \n";
        ros::spinOnce();
        ros::Duration(3).sleep();
    }

    cartesian_goal.goal_position = High_center;
    cartesian_goal.goal_quaternion = sun::UnitQuaternion(goal_R);
    std::cout << cartesian_goal.goal_quaternion;
    cartesian_goal.Tf = 20; // s 

    success = set_goal_and_call_srv(cartesian_goal);

    while(ros::ok() && traj_running && success){
        
        std::cout << " In attesa che il robot raggiunga la posa assegnata \n";
        ros::spinOnce();
        ros::Duration(3).sleep();
    }

    return 0;
}
