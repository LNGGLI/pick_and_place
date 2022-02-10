
#include <ros/ros.h>

#include <sun_robot_lib/Robot.h>
#include <sun_robot_lib/Robots/UR5e.h>
#include <sun_math_toolbox/UnitQuaternion.h>
#include <sun_math_toolbox/PortingFunctions.h>

#include <sun_traj_lib/Line_Segment_Traj.h>
#include <sun_traj_lib/Quintic_Poly_Traj.h>

#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>


/* Global variables */
double fs = 500.0;
sun::UR5e UR5e_robot;
TooN::Vector<6> joint_position_robot;
TooN::Vector<6> joint_position_DH;
TooN::Vector<6> jointPosition_Robot_cmd;
TooN::Vector<3> actual_pos;
sun::UnitQuaternion actual_Q;

ros::Publisher joints_traj_pub;
trajectory_msgs::JointTrajectoryPoint traj_point_msg;


/* Callback for Robot Joint States Data */
bool joints_acquired = false;
void jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  joint_position_robot[0] = msg->position[2];
  joint_position_robot[1] = msg->position[1];
  joint_position_robot[2] = msg->position[0];
  joint_position_robot[3] = msg->position[3];
  joint_position_robot[4] = msg->position[4];
  joint_position_robot[5] = msg->position[5];
  
  joint_position_DH = UR5e_robot.joints_Robot2DH(joint_position_robot);
  
  actual_pos = sun::transl( UR5e_robot.fkine(joint_position_DH) );
  actual_Q = sun::UnitQuaternion( UR5e_robot.fkine(joint_position_DH) );
  joints_acquired = true;
}


double linear_velocity = 0.02; // m/sec
double angular_velocity = 3.0 * M_PI / 180.0; // rad/sec
void goToPose(const TooN::Vector<3>& desired_pos, const sun::UnitQuaternion& desired_Q)
{
    ros::spinOnce(); // update robot's state
    
    TooN::Vector<6> qDH_k = joint_position_DH;
    sun::UnitQuaternion oldQ = actual_Q;

    /* Questi possono essere dichiarati una sola volta all'esterno della funzione */
    double Ts = 1.0/fs;
    double gain = 0.5*fs;
    TooN::Vector<> qpDH = TooN::Zeros(6);
    TooN::Vector<6,int> mask = TooN::Ones;
    TooN::Vector<3> xd = TooN::Zeros;
    TooN::Vector<3> w = TooN::Zeros;
    TooN::Vector<6> error = TooN::Ones; // questo va "resettato" ogni volta prima del clik

    /* Come durata della traiettoria scelgo il max tra la durata del movimento in traslazione e rotazione */
    double lin_duration = norm(actual_pos-desired_pos)/linear_velocity;
    double ang_duration = abs((inv(actual_Q)*desired_Q).toangvec().getAng())/angular_velocity;
    double duration = std::max( lin_duration,ang_duration);
    
    sun::Quintic_Poly_Traj scalar_traj(duration,0.0,1.0);
    sun::Line_Segment_Traj line_traj(actual_pos,desired_pos,scalar_traj);

    scalar_traj.changeInitialTime(ros::Time::now().toSec());
    line_traj.changeInitialTime(scalar_traj.getInitialTime());
    double actual_time = ros::Time::now().toSec();
    while (ros::ok() && !line_traj.isCompleate(actual_time))
    {
        qDH_k = UR5e_robot.clik(   
                        qDH_k, //<- qDH attuale
                        line_traj.getPosition(actual_time), // <- posizione desiderata
                        actual_Q.interp(desired_Q,scalar_traj.getPosition(actual_time)), // <- quaternione desiderato
                        oldQ,// <- quaternione al passo precedente (per garantire la continuità)
                        xd, // <- velocità in translazione desiderata
                        w, //<- velocità angolare desiderata
                        mask,// <- maschera, se l'i-esimo elemento è zero allora l'i-esima componente cartesiana non verrà usata per il calcolo dell'errore
                        gain,// <- guadagno del clik
                        Ts,// <- Ts, tempo di campionamento
                        0.0, // <- quadagno obj secondario
                        TooN::Zeros(UR5e_robot.getNumJoints()), // velocità di giunto dell'obj secondario (qui sono zero)              
                        //Return Vars
                        qpDH, // <- variabile di ritorno velocità di giunto
                        error, //<- variabile di ritorno errore
                        oldQ // <- variabile di ritorno: Quaternione attuale (N.B. qui uso oldQ in modo da aggiornare direttamente la variabile oldQ e averla già pronta per la prossima iterazione)
                        );
        jointPosition_Robot_cmd = UR5e_robot.joints_DH2Robot(qDH_k);

        traj_point_msg.time_from_start = ros::Duration(Ts);

        traj_point_msg.positions[0] = jointPosition_Robot_cmd[0]; /* shoulder_pan_joint */
        traj_point_msg.positions[1] = jointPosition_Robot_cmd[1]; /* shoulder_lift_joint */
        traj_point_msg.positions[2] = jointPosition_Robot_cmd[2]; /* elbow_joint */
        traj_point_msg.positions[3] = jointPosition_Robot_cmd[3]; /* wrist_1_joint */
        traj_point_msg.positions[4] = jointPosition_Robot_cmd[4]; /* wrist_2_joint */
        traj_point_msg.positions[5] = jointPosition_Robot_cmd[5]; /* wrist_3_joint */
        
        joints_traj_pub.publish(traj_point_msg);

        ros::Rate(fs).sleep();
        actual_time = ros::Time::now().toSec();
    }
    
}


int main(int argc, char **argv){

	ros::init(argc,argv,"test_motion_node");
	ros::NodeHandle nh;

    /* ROS subscriber declaration */
    ros::Subscriber jointStates_sub = nh.subscribe("/joint_states", 1, jointStatesCallback);
    
    traj_point_msg.positions.resize(6);

    ros::Duration(0.5).sleep();

    TooN::Vector<> tcp0 = TooN::makeVector(0.0,0.0,0.2245);
    UR5e_robot.setnTe( sun::rt2tr( tcp0, TooN::Identity(3) ) ); // set TCP at pad center
    TooN::Vector<3> p_des = TooN::makeVector(atof(argv[1]),atof(argv[2]),atof(argv[3]));
    std::cout << "p_des: " << p_des << std::endl;
    double r = atof(argv[4])*M_PI/180.0;
    std::cout << "r: " << r << std::endl;
    double p = atof(argv[5])*M_PI/180.0;
    std::cout << "p: " << p << std::endl;
    double y = atof(argv[6])*M_PI/180.0;
    std::cout << "y: " << y << std::endl;
    sun::UnitQuaternion Q_des = sun::UnitQuaternion::rpy(r,p,y);

    goToPose(p_des,Q_des);

    return 0;
}