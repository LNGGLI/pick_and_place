
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

// Server , Actions 
#include <actionlib/client/simple_action_client.h>

// Utils
#include <ros/ros.h>
#include <franka_gripper/franka_gripper.h>
#include <sun_traj_lib/Cartesian_Independent_Traj.h>
#include <pick_and_place/trajectory_node.h>
#include <sun_traj_lib/Quintic_Poly_Traj.h>
#include <tf/transform_listener.h>


// Messages
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <franka_msgs/FrankaState.h>





using namespace trajectory;

int main(int argc, char** argv) {

    ros::init(argc, argv, "trajectory_node");
    ros::NodeHandle nh;
    ros::Publisher command_pb = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>("cartesian_trajectory_command", 1);
    ros::Subscriber pose_sub = nh.subscribe<franka_msgs::FrankaState>("/franka_state_controller/franka_states",1,stateCB);

    while(!initial_read){
        ros::spinOnce();
        ros::Duration(0.1).sleep();
        std::cout << "In attesa di leggere la posa iniziale\n";
    }
    
    std::cout << "Configurazione iniziale (initial_transform) acquisita. \n";
    
    std::cout << "Generazione della traiettoria in cartesiano \n";

    // Generazione della traiettoria su primitiva di percorso di tipo segmento:

    tf::Vector3 pi = initial_transform.getOrigin();
    // TooN::Vector< 3 > pi = {initial_position.getX(),initial_position.getY(),initial_position.getZ()}; // initial_position
    // TooN::Vector< 3 > pf = {1,2,3};


    //sun::Quintic_Poly_Traj polinomio_quintico(Tf, 0 , 1);

    // Line_Segment_Traj line_traj(); (pi,pf,polinomio_quintico)



    // Cartesian_Independent_Traj cartesian_traj;

    std::string start_controller = "";
    std::string stop_controller = "";
    bool ok = switch_controller(start_controller,stop_controller);

    if(ok)
        std::cout << "Lo switch dei controller è stato effettuato!" << std::endl;
    else
        std::cout << "Lo switch dei controller non è andato a buon fine " << std::endl;


    double begin = ros::Time::now().toSec();
    double t;


    ros::Rate loop_rate(1000); // 1kHz

    while (ros::ok() && t < Tf)
    {
        
        t = ros::Time::now().toSec() - begin; // tempo trascorso
        
        trajectory_msgs::MultiDOFJointTrajectory msg;
        command_pb.publish(msg);

        loop_rate.sleep();
        
    }
    
    
    return 0;

}


