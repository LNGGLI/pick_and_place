
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
#include <geometry_msgs/Pose.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>

/*rosbag record /fkine /joint_commands /franka_state_controller/franka_states /franka_state_controller/joint_states_desired errors /cartesian_trajectory_command*/

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
        "/joint_commands", 1);
    
    ros::Publisher traj_pb = nh.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>(
        "/cartesian_trajectory_command", 1);
    
    ros::Publisher fkine_pb = nh.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>(
        "/fkine", 1);

    ros::Publisher error_pb = nh.advertise<geometry_msgs::Pose>("errors", 1);

    ros::Subscriber pose_sub = nh.subscribe<franka_msgs::FrankaState>(
        "/franka_state_controller/franka_states", 1, stateCB);


    
    while (!initial_read)
    {
        ros::spinOnce();
        ros::Duration(0.2).sleep();
        std::cout << "\nIn attesa di leggere la posa iniziale\n";
    }

    std::cout << "Configurazione iniziale (initial_transform e initial_config) acquisita. \n";

    // initial_transform è una Toon::SE3
    TooN::Matrix<4,4,double> n_T_e (TooN::Data(0.7071, 0.7071,   0.0, 0.0,
                                                -0.7071,  0.7071,  0.0, 0.0,
                                                0.0,   0.0,    1.0, 0.1034,
                                                0.0,   0.0,    0.0, 1.0));

    sun::Panda panda(n_T_e,10.0,"panda");
    panda.display();
    TooN::Matrix<4,4,double> initial_pose = panda.fkine(initial_conf);

    std::cout << "\n Fkine panda object: \n ";
    for(int i = 0; i < 4;i++){
        for(int j = 0; j < 4; j++){
            std::cout << initial_pose[i][j] << " ";
        }
        std::cout << std::endl;
    }
    
     
    TooN::Vector<3> pi = TooN::makeVector(initial_pose[0][3],initial_pose[1][3],initial_pose[2][3]);//initial_transform.get_translation();                     // initial_position

    sun::UnitQuaternion init_quat(initial_pose);// initial_transform.get_rotation().get_matrix()); // initial orientation

    std::cout << "Generazione della traiettoria in cartesiano \n";

    // Generazione della traiettoria su primitiva di percorso di tipo segmento:

    // TODO: scegliere punto finale
    TooN::Vector<3, double> pf({0.3, 0.3, 0.5});
    TooN::Vector<3, double> axis({0, 0, 1});
    
    sun::Quintic_Poly_Traj qp(Tf, 0, 1); // polinomio quintico utilizzato sia per line_traj che theta_traj
    
    sun::Line_Segment_Traj line_traj(pi, pf, qp);
    sun::Rotation_Const_Axis_Traj quat_traj(init_quat, axis, qp);

    sun::Cartesian_Independent_Traj cartesian_traj(line_traj, quat_traj);

    // Accensione del controller
    

    // bool ok = switch_controller("pick_and_place_controller", "");

    // if (ok)
    //     std::cout << "Lo switch del controller è stato effettuato!" << std::endl;
    // else
    //     std::cout << "Lo switch del controller non è andato a buon fine " << std::endl;

    
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
    TooN::Vector<7> qDH_k = initial_conf;
    sun::UnitQuaternion oldQ = init_quat;

    sun::UnitQuaternion unit_quat_d = init_quat;
    TooN::Vector<3,double> posizione_d = pi;

    begin = ros::Time::now().toSec();
    bool start = true;


    while (ros::ok())  // && !cartesian_traj.isCompleate(t)
    {

        t = ros::Time::now().toSec() - begin; // tempo trascorso
        posizione_d = cartesian_traj.getPosition(t);
        unit_quat_d = cartesian_traj.getQuaternion(0);
        xd = cartesian_traj.getLinearVelocity(t);
        w = cartesian_traj.getAngularVelocity(0);

        qDH_k = panda.clik(qDH_k, //<- qDH attuale
                            posizione_d, // <- posizione desiderata
                            unit_quat_d, // <- quaternione desiderato
                            oldQ, // <- quaternione al passo precedente (per garantire la continuità)
                            xd, // <- velocità in translazione desiderata
                            w, //<- velocità angolare desiderata
                            mask,// <- maschera, se l'i-esimo elemento è zero allora l'i-esima componente cartesiana non verrà usata per il calcolo dell'errore
                            gain,// <- guadagno del clik
                            Ts,// <- Ts, tempo di campionamento
                            0.0, // <- quadagno obj secondario
                            TooN::Zeros(panda.getNumJoints()),
                            
                            //Return Vars
                            qdot,// <- variabile di ritorno velocità di giunto
                            error, //<- variabile di ritorno errore
                            oldQ // <- variabile di ritorno: Quaternione attuale (N.B. qui uso oldQ in modo da aggiornare direttamente la variabile oldQ e averla già pronta per la prossima iterazione)
                        );
        
        bool limits_exceeded = panda.exceededHardJointLimits(panda.joints_DH2Robot(qDH_k));
        trajectory_msgs::JointTrajectoryPoint command_msg;
        geometry_msgs::Pose error_msg;
        trajectory_msgs::MultiDOFJointTrajectoryPoint traj_msg;
        trajectory_msgs::MultiDOFJointTrajectoryPoint fkine_msg;

        
        // Pubblicazione errore
        error_msg.position.x = error[0];
        error_msg.position.y = error[1];
        error_msg.position.z = error[2];

        error_msg.orientation.x = error[3];
        error_msg.orientation.y = error[4];
        error_msg.orientation.z = error[5];

        error_pb.publish(error_msg);
        
        // Pubblicazione comando in spazio giunti 
        if(!limits_exceeded){
        for(int i = 0; i< 7; i++)
            command_msg.positions.push_back(qDH_k[i]);
        }
        else 
            throw std::runtime_error("Limiti di giunto superati");

        command_pb.publish(command_msg);
        

        // Pubblicazione comando in cartesiano
        
        traj_msg.transforms.resize(1);
        traj_msg.velocities.resize(1);
        traj_msg.time_from_start = ros::Duration(t);
       
        
        traj_msg.transforms[0].translation.x = posizione_d[0];
        traj_msg.transforms[0].translation.y = posizione_d[1];
        traj_msg.transforms[0].translation.z = posizione_d[2];

        // Comando in orientamento
        traj_msg.transforms[0].rotation.x = unit_quat_d.getS();
        TooN::Vector<3, double> vec_quat = unit_quat_d.getV();
        traj_msg.transforms[0].rotation.y = vec_quat[0];
        traj_msg.transforms[0].rotation.z = vec_quat[1];
        traj_msg.transforms[0].rotation.w = vec_quat[2];

        traj_pb.publish(traj_msg);


        // Pubblicazione cinematica diretta ottenuta dalla q clik
        TooN::Matrix<4,4,double> fkine = panda.fkine(qDH_k);
        fkine_msg.transforms.resize(1);
        fkine_msg.transforms[0].translation.x = fkine[0][3];
        fkine_msg.transforms[0].translation.y = fkine[1][3];
        fkine_msg.transforms[0].translation.z = fkine[2][3];

        sun::UnitQuaternion quat_fkine(fkine); 
        TooN::Vector<3, double> fkine_vec_quat = quat_fkine.getV();
        fkine_msg.transforms[0].rotation.x = unit_quat_d.getS();
        fkine_msg.transforms[0].rotation.y = fkine_vec_quat[0];
        fkine_msg.transforms[0].rotation.z = fkine_vec_quat[1];
        fkine_msg.transforms[0].rotation.w = fkine_vec_quat[2];

          
        fkine_pb.publish(fkine_msg);
        

        loop_rate.sleep();

    }

    return 0;
}
