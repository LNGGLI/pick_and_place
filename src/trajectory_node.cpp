
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

// Server , Actions
#include <actionlib/client/simple_action_client.h>

// Utils
#include <ros/ros.h>
#include <franka_gripper/franka_gripper.h>

#include <pick_and_place/trajectory_node.h>
#include <pick_and_place/check_realtime.h>
#include <sun_traj_lib/Cartesian_Independent_Traj.h>
#include <sun_traj_lib/Quintic_Poly_Traj.h>
#include <sun_traj_lib/Line_Segment_Traj.h>
#include <sun_traj_lib/Rotation_Const_Axis_Traj.h>

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
    ros::Publisher command_pb = nh.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>(
        "/cartesian_trajectory_command", 10);

    ros::Subscriber pose_sub = nh.subscribe<franka_msgs::FrankaState>(
        "/franka_state_controller/franka_states", 1, stateCB);


    while (!initial_read)
    {
        ros::spinOnce();
        ros::Duration(0.2).sleep();
        std::cout << "In attesa di leggere la posa iniziale\n";
    }

    std::cout << "Configurazione iniziale (initial_transform) acquisita. \n";

    // initial_transform è una Toon::SE3

    TooN::Vector<3> pi = initial_transform.get_translation();                     // initial_position

    sun::UnitQuaternion init_quat(initial_transform.get_rotation().get_matrix()); // initial orientation

    std::cout << "Generazione della traiettoria in cartesiano \n";

    // Generazione della traiettoria su primitiva di percorso di tipo segmento:

    // TODO: scegliere punto finale
    TooN::Vector<3, double> pf({0.3, 0.3, 0.3});
    TooN::Vector<3, double> axis({0, 0, 1});
    
    sun::Quintic_Poly_Traj qp(Tf, 0, 1); // polinomio quintico utilizzato sia per line_traj che theta_traj
    
    sun::Line_Segment_Traj line_traj(pi, pf, qp);
    sun::Rotation_Const_Axis_Traj quat_traj(init_quat, axis, qp);

    sun::Cartesian_Independent_Traj cartesian_traj(line_traj, quat_traj);

    // Accensione del controller
    {
    if (!check_realtime()) 
      throw std::runtime_error("REALTIME NOT AVAILABLE");

    if (!set_realtime_SCHED_FIFO()) 
        throw std::runtime_error("ERROR IN set_realtime_SCHED_FIFO");

    }

    bool ok = switch_controller("cartesian_pose_controller", "");

    if (ok)
        std::cout << "Lo switch del controller è stato effettuato!" << std::endl;
    else
        std::cout << "Lo switch del controller non è andato a buon fine " << std::endl;

    

    return 0;
}
