
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

#include <sun_traj_lib/Cartesian_Independent_Traj.h>
#include <sun_traj_lib/Quintic_Poly_Traj.h>
#include <sun_traj_lib/Line_Segment_Traj.h>
#include <sun_traj_lib/Rotation_Const_Axis_Traj.h>


// Messages
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <franka_msgs/FrankaState.h>





using namespace trajectory;

int main(int argc, char** argv) {

    ros::init(argc, argv, "trajectory_node");
    ros::NodeHandle nh;
    ros::Publisher command_pb = nh.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>("cartesian_trajectory_command", 1);
    ros::Subscriber pose_sub = nh.subscribe<franka_msgs::FrankaState>("/franka_state_controller/franka_states",1,stateCB);

    while(!initial_read){
        ros::spinOnce();
        ros::Duration(0.1).sleep();
        std::cout << "In attesa di leggere la posa iniziale\n";
    }
    
    std::cout << "Configurazione iniziale (initial_transform) acquisita. \n";
    
    TooN::Vector< 3 > pi({initial_transform.getOrigin().x(),
                          initial_transform.getOrigin().y(),
                          initial_transform.getOrigin().z()}); // initial_position

    TooN::Vector <4> vec_quat({initial_transform.getRotation()[0], initial_transform.getRotation()[1],
                                initial_transform.getRotation()[2], initial_transform.getRotation()[3]});

    sun::UnitQuaternion initial_quaternion(vec_quat);
    std::cout << "Generazione della traiettoria in cartesiano \n";

    // Generazione della traiettoria su primitiva di percorso di tipo segmento:
    

    TooN::Vector< 3 > pf ({0.5,0.5,0.5});
    TooN::Vector<3> axis({0,0,1});

    sun::Quintic_Poly_Traj qp(Tf, 0 , 1); // polinomio quintico.

    sun::Line_Segment_Traj line_traj(pi,pf,qp);
    sun::Rotation_Const_Axis_Traj quat_traj(initial_quaternion,axis,qp);

    sun::Cartesian_Independent_Traj cartesian_traj(line_traj,quat_traj);


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
        
        trajectory_msgs::MultiDOFJointTrajectoryPoint msg;

       
        // Comando in posizione   
        TooN::Vector<3> posizione = cartesian_traj.getPosition(t);
        msg.transforms[0].translation.x = posizione[0];
        msg.transforms[0].translation.y = posizione[1];
        msg.transforms[0].translation.z = posizione[2];
        
        // Comando in velocità lineare
        TooN::Vector<3> velocita_lineare = cartesian_traj.getLinearVelocity(t);
        msg.velocities[0].linear.x = velocita_lineare[0];
        msg.velocities[0].linear.y = velocita_lineare[1];
        msg.velocities[0].linear.z = velocita_lineare[2];
        
        // Comando in orientamento 
        sun::UnitQuaternion unit_quat = cartesian_traj.getQuaternion(t);
        msg.transforms[0].rotation.x = unit_quat.getS();
        TooN::Vector<3> vec_quat = unit_quat.getV();
        msg.transforms[0].rotation.y = vec_quat[0];
        msg.transforms[0].rotation.z= vec_quat[1];
        msg.transforms[0].rotation.w = vec_quat[2];

        // Comando in velocità angolare
        TooN::Vector<3> velocita_angolare = cartesian_traj.getAngularVelocity(t);
        msg.velocities[0].angular.x = velocita_angolare[0];
        msg.velocities[0].angular.x = velocita_angolare[1];
        msg.velocities[0].angular.x = velocita_angolare[2];
             
        command_pb.publish(msg);

        loop_rate.sleep();
        
    }
    



    
    return 0;

}


