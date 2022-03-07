
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include <pick_and_place/trajectory_node.h>
#include <pick_and_place/Panda.h>

// Server , Actions
#include <pick_and_place/SetTraj.h>

// Utils
#include <ros/ros.h>
#include <franka_gripper/franka_gripper.h>
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

int main(int argc, char **argv)
{

    ros::init(argc, argv, "trajectory_node");
    ros::NodeHandle nh;

    client_set_traj = nh.serviceClient<pick_and_place::SetTraj>("/set_traj");
    ros::Subscriber state_sub = nh.subscribe<franka_msgs::FrankaState>("/franka_state_controller/franka_states", 1, stateCB);
    
    // Check e set realtime node
    // if (!check_realtime()) 
    //   throw std::runtime_error("REALTIME NOT AVAILABLE");

    // if (!set_realtime_SCHED_FIFO()) 
    //     throw std::runtime_error("ERROR IN set_realtime_SCHED_FIFO");


    // Accensione del controller
    bool ok = switch_controller("cartesian_pose_controller", "");


    if (ok){
        std::cout << "Lo switch del controller è stato effettuato!" << std::endl;
    }
    else{
        std::cout << "Lo switch del controller non è andato a buon fine " << std::endl;
    }

    // Costruzione e invio della posa desiderata 
    goal_position = TooN::makeVector(0.4 , 0.4 , 0.4);

    TooN::Matrix<3,3,double> goal_R = TooN::Data(1,0,0,
                                                0,0,1,
                                                0,-1,0);
                                                
    goal_quat = sun::UnitQuaternion(goal_R);
    double Tf = 10; // s 

    bool success = set_goal_and_call_srv(goal_position, goal_quat, Tf);

    while(ros::ok() && traj_running && success){
        
        std::cout << " In attesa che il robot raggiunga la posa assegnata \n";
        ros::spinOnce();
        ros::Duration(3).sleep();

    }
    
    std::cout << "Il robot ha raggiunto la posa assegnata \n";
        

    return 0;
}
