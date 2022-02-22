
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

// Server , Actions
#include <pick_and_place/SetTraj.h>

// Utils
#include <ros/ros.h>
#include <franka_gripper/franka_gripper.h>

#include <pick_and_place/trajectory_node.h>
#include <pick_and_place/check_realtime.h>

// Sun
#include <sun_math_toolbox/UnitQuaternion.h>

// Messages
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <franka_msgs/FrankaState.h>

/* rosbag record /jointsIK /cartesian_trajectory_command
 /franka_state_controller/joint_states_desired 
 /franka_state_controller/franka_states*/

using namespace trajectory;

int main(int argc, char **argv)
{

    ros::init(argc, argv, "trajectory_node");
    ros::NodeHandle nh;

    ros::ServiceClient client_set_traj = nh.serviceClient<pick_and_place::SetTraj>("/set_traj");
    // Accensione del controller
    // {
    // if (!check_realtime()) 
    //   throw std::runtime_error("REALTIME NOT AVAILABLE");

    // if (!set_realtime_SCHED_FIFO()) 
    //     throw std::runtime_error("ERROR IN set_realtime_SCHED_FIFO");

    // }



    bool ok = switch_controller("cartesian_pose_controller", "");

    if (ok)
        std::cout << "Lo switch del controller è stato effettuato!" << std::endl;
    else
        std::cout << "Lo switch del controller non è andato a buon fine " << std::endl;


    pick_and_place::SetTraj set_traj_msg;

    TooN::Vector<3,double> goal_position = TooN::makeVector(0.4, 0.4, 0.4);
    TooN::Matrix<3, 3> R_des = TooN::Data(1,0,0,0,0,1,0,-1,0); // orientamento desiderata
    sun::UnitQuaternion goal_quaternion(R_des);
    ros::Duration(4).sleep();
    set_traj_msg.request.goal_position.x = goal_position[0];
    set_traj_msg.request.goal_position.y = goal_position[1];
    set_traj_msg.request.goal_position.z = goal_position[2];

    set_traj_msg.request.goal_quaternion.w = goal_quaternion.getS();
    set_traj_msg.request.goal_quaternion.x = goal_quaternion.getV()[0];
    set_traj_msg.request.goal_quaternion.y = goal_quaternion.getV()[1];
    set_traj_msg.request.goal_quaternion.z = goal_quaternion.getV()[2];

    set_traj_msg.request.Tf = 15;

    if(client_set_traj.call(set_traj_msg)){
        if(set_traj_msg.response.success)
            std::cout << "Il setting della traiettoria è stato effettuato correttamente\n";
        else
            std::cout <<"Il setting della traiettoria non ha avuto successo \n";

    }
    else
        std::cout << "Errore nella chiamata del servizio \n";
    
    return 0;
}
