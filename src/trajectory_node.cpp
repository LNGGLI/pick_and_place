
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
 /franka_statcontroller/joint_states_desired 
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

    
    
    // double begin = ros::Time::now().toSec();
    // double t;

    // ros::Rate loop_rate(1000); // 1kHz

    // std::cout << "Resta fermo per 2 secondi" << std::endl;
    // while (t < 2)  
    // {

    //     t = ros::Time::now().toSec() - begin; // tempo trascorso

    //     trajectory_msgs::MultiDOFJointTrajectoryPoint msg;
    //     // Solo dopo resize si puo' indicizzare transforms[0] e velocity[0] poichè sono
    //     // inizialmente vuoti.
    //     msg.transforms.resize(1);
    //     msg.velocities.resize(1);

    //     // Comando in posizione
    //     TooN::Vector<3, double> posizione = cartesian_traj.getPosition(0.0);
    //     msg.transforms[0].translation.x = posizione[0];
    //     msg.transforms[0].translation.y = posizione[1];
    //     msg.transforms[0].translation.z = posizione[2];

    //     // Comando in orientamento
    //     sun::UnitQuaternion unit_quat = cartesian_traj.getQuaternion(0.0);
    //     msg.transforms[0].rotation.x = unit_quat.getS();
    //     TooN::Vector<3, double> vec_quat = unit_quat.getV();
    //     msg.transforms[0].rotation.y = vec_quat[0];
    //     msg.transforms[0].rotation.z = vec_quat[1];
    //     msg.transforms[0].rotation.w = vec_quat[2];



    //     command_pb.publish(msg);

    //     loop_rate.sleep();
    // }

    // std:: cout << "Inizia moto " << std::endl;
    
    // begin = ros::Time::now().toSec();
    // bool start = true;
    // while (ros::ok() && !cartesian_traj.isCompleate(t))  
    // {

    //     t = ros::Time::now().toSec() - begin; // tempo trascorso

    //     trajectory_msgs::MultiDOFJointTrajectoryPoint msg;
    //     // Solo dopo resize si puo' indicizzare transforms[0] e velocity[0] poichè sono
    //     // inizialmente vuoti.
    //     msg.transforms.resize(1);
    //     msg.velocities.resize(1);

    //     // Comando in posizione
    //     TooN::Vector<3, double> posizione = cartesian_traj.getPosition(t);
    //     msg.transforms[0].translation.x = posizione[0];
    //     msg.transforms[0].translation.y = posizione[1];
    //     msg.transforms[0].translation.z = posizione[2];

    //     // std::cout << "Posizione traiettoria: \n"
    //     //           << "x:" <<  posizione[0] << "\n"
    //     //           << "y:" <<  posizione[1] << "\n"
    //     //           << "z:" <<  posizione[2] << "\n";

    //     // Comando in velocità lineare
    //     TooN::Vector<3, double> velocita_lineare = cartesian_traj.getLinearVelocity(t);
    //     msg.velocities[0].linear.x = velocita_lineare[0];
    //     msg.velocities[0].linear.y = velocita_lineare[1];
    //     msg.velocities[0].linear.z = velocita_lineare[2];

    //     // Comando in orientamento
    //     sun::UnitQuaternion unit_quat = cartesian_traj.getQuaternion(t);
    //     msg.transforms[0].rotation.x = unit_quat.getS();
    //     TooN::Vector<3, double> vec_quat = unit_quat.getV();
    //     msg.transforms[0].rotation.y = vec_quat[0];
    //     msg.transforms[0].rotation.z = vec_quat[1];
    //     msg.transforms[0].rotation.w = vec_quat[2];


    //     // if(start){
    //     //     start = false;
    //     //     TooN::Matrix<3> R = unit_quat.R();
    //     //     std::cout << "Matrice di rotazione inviata: \n";
    //     //     for(int i = 0; i < 3 ; i++ ){
    //     //         for(int j = 0 ; j<3 ; j++)
    //     //             std::cout << R[i][j] << " ";
    //     //         std::cout << "\n";
    //     //     }
    //     // }
       

    //     // Comando in velocità angolare
    //     TooN::Vector<3, double> velocita_angolare = cartesian_traj.getAngularVelocity(t);
    //     msg.velocities[0].angular.x = velocita_angolare[0];
    //     msg.velocities[0].angular.x = velocita_angolare[1];
    //     msg.velocities[0].angular.x = velocita_angolare[2];

    //     command_pb.publish(msg);

    //     loop_rate.sleep();
    // }

    return 0;
}
