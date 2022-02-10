
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include <pick_and_place/trajectory_node.h>
#include <pick_and_place/Panda.h>

// Server , Actions
#include <actionlib/client/simple_action_client.h>

// Utils
#include <ros/ros.h>
#include <franka_gripper/franka_gripper.h>
#include <pick_and_place/check_realtime.h>
#include <TooN/TooN.h>


// Sun
#include <sun_robot_lib/Robot.h>
#include <sun_traj_lib/Cartesian_Independent_Traj.h>
#include <sun_traj_lib/Quintic_Poly_Traj.h>
#include <sun_traj_lib/Line_Segment_Traj.h>
#include <sun_traj_lib/Rotation_Const_Axis_Traj.h>

// Messages
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <franka_msgs/FrankaState.h>

/* rosbag record /jointsIK /cartesian_trajectory_command
 /franka_statcontroller/joint_states_desired 
 /franka_state_controller/franka_states*/

using namespace trajectory;

int main(int argc, char **argv)
{

    ros::init(argc, argv, "trajectory_node");
    ros::NodeHandle nh;
    

    {
    if (!check_realtime()) 
      throw std::runtime_error("REALTIME NOT AVAILABLE");

    if (!set_realtime_SCHED_FIFO()) 
        throw std::runtime_error("ERROR IN set_realtime_SCHED_FIFO");
    }

    ros::Publisher command_pb = nh.advertise<trajectory_msgs::JointTrajectoryPoint>(
        "joint_commands", 1);

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
    TooN::Matrix<4,4,double> n_T_e (TooN::Data(0.7071, 0.7071,   0.0, 0.0,
                                                -0.7071,  0.7071,  0.0, 0.0,
                                                0.0,   0.0,    1.0, 0.1034,
                                                0.0,   0.0,    0.0, 1.0));

    sun::Panda panda(n_T_e,10.0,"panda");
    

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
    

    bool ok = switch_controller("cartesian_pose_controller", "");

    if (ok)
        std::cout << "Lo switch del controller è stato effettuato!" << std::endl;
    else
        std::cout << "Lo switch del controller non è andato a buon fine " << std::endl;

    
    double fs = 1000; // frequenza
    double Ts = 1/fs;

    ros::Rate loop_rate(fs); // Hz
    double begin = ros::Time::now().toSec();
    double t;
    

    std:: cout << "Inizia moto " << std::endl;
    double gain = 0.5*fs;
    TooN::Vector<> qdot = TooN::Zeros(7); // velocità di giunto ritorno
    TooN::Vector<6,int> mask = TooN::Ones; // maschera, se l'i-esimo elemento è zero allora l'i-esima componente cartesiana non verrà usata per il calcolo dell'errore
    TooN::Vector<3> xd = TooN::Zeros; // velocità in translazione desiderata
    TooN::Vector<3> w = TooN::Zeros; // velocità angolare desiderata
    TooN::Vector<6> error = TooN::Ones; // questo va "resettato" ogni volta prima del clik
    TooN::Vector<7> qDH_k;
    sun::UnitQuaternion oldQ = init_quat;


    begin = ros::Time::now().toSec();
    bool start = true;


    while (ros::ok() && !cartesian_traj.isCompleate(t))  
    {

        t = ros::Time::now().toSec() - begin; // tempo trascorso
        TooN::Vector<3,double> posizione_d = cartesian_traj.getPosition(t);
        sun::UnitQuaternion unit_quat_d = cartesian_traj.getQuaternion(0.0);

         qDH_k = panda.clik(qDH_k,
                            posizione_d,
                            unit_quat_d,
                            oldQ,
                            xd,
                            w,
                            mask,
                            gain,
                            Ts,
                            0.0,
                            TooN::Zeros(panda.getNumJoints()),
                            qdot,
                            error,
                            oldQ);

        
        trajectory_msgs::JointTrajectoryPoint command_msg;
        
        command_pb.publish(command_msg);

        loop_rate.sleep();
    }

    return 0;
}
