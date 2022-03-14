
#pragma once

#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

// Server , Actions
#include <pick_and_place/SetTraj.h>
#include <controller_manager_msgs/ControllerState.h>
#include <controller_manager_msgs/SwitchController.h>
#include <actionlib/client/simple_action_client.h>

// Utils
#include <ros/ros.h>

// Sun
#include <sun_math_toolbox/UnitQuaternion.h>

// Messages
#include <franka_msgs/FrankaState.h>
#include <TooN/TooN.h>



using controller_manager_msgs::SwitchControllerRequest;
using controller_manager_msgs::SwitchControllerResponse;

namespace trajectory{


    
    struct CartesianGoal{
        TooN::Vector<3,double> goal_position;
        sun::UnitQuaternion goal_quaternion;
        double Tf;
    };

    CartesianGoal cartesian_goal; // struct per definire il goal in cartesiano
    ros::ServiceClient client_set_traj;
    bool traj_running = false;


    bool switch_controller(const std::string& start_controller, const std::string& stop_controller){

        SwitchControllerRequest switch_req;
        SwitchControllerResponse switch_res;
        
        switch_req.start_controllers.push_back(start_controller);
        switch_req.stop_controllers.push_back(stop_controller);
        switch_req.strictness = 1; // BEST_EFFORT 
        switch_req.start_asap = false;
        switch_req.timeout = 0.0;
        ros::service::call("/controller_manager/switch_controller",switch_req,switch_res);     
        if(switch_res.ok==true)
        
            if(start_controller != "" && stop_controller !="")
                ROS_INFO_STREAM("Attivato " << start_controller << " e fermato "<< stop_controller);

            else if (start_controller != "" && stop_controller =="")
                ROS_INFO_STREAM("Attivato " << start_controller);

            else if (start_controller == "" && stop_controller !="")
                ROS_INFO_STREAM("Fermato " << stop_controller);   
        else
            ROS_INFO("Operazione switch non riuscita");     

        return switch_res.ok;
    }
    
  
    bool set_goal_and_call_srv(const CartesianGoal& cartesian_goal){
        
        pick_and_place::SetTraj set_traj_msg;

        set_traj_msg.request.goal_position.x = cartesian_goal.goal_position[0];
        set_traj_msg.request.goal_position.y = cartesian_goal.goal_position[1];
        set_traj_msg.request.goal_position.z = cartesian_goal.goal_position[2];

        set_traj_msg.request.goal_quaternion.w = cartesian_goal.goal_quaternion.getS();
        set_traj_msg.request.goal_quaternion.x = cartesian_goal.goal_quaternion.getV()[0];
        set_traj_msg.request.goal_quaternion.y = cartesian_goal.goal_quaternion.getV()[1];
        set_traj_msg.request.goal_quaternion.z = cartesian_goal.goal_quaternion.getV()[2];

        set_traj_msg.request.Tf = cartesian_goal.Tf;

        if(client_set_traj.call(set_traj_msg)){
            if(set_traj_msg.response.success){
                std::cout << "Il setting della traiettoria è stato effettuato correttamente\n";
                traj_running = true;
                return true;
            }
            else{
                std::cout <<"Il setting della traiettoria non ha avuto successo \n";
                return false;
            }
        }
        else{
            std::cout << "Errore nella chiamata del servizio \n";
            return false;
        }
        
    }

    void stateCB(const franka_msgs::FrankaState::ConstPtr& msg){

        double transform[16];
        for(int i = 0; i < 16 ; i++)
            transform[i]= msg->O_T_EE[i]; // ATTENZIONE la O_T_EE è passata per colonne!

        
        double R_array[9] = {transform[0],transform[1],transform[2],  // Array matrice di rotazione (Column Major)
                            transform[4],transform[5],transform[6],
                            transform[8],transform[9],transform[10]};

        // Posizione attuale
        TooN::Vector<3,double> current_pos({transform[12],transform[13],transform[14]});

        // Costruzione matrice di rotazione e quaternione
        TooN::Matrix<3,3> R;
        for(int j = 0; j < 3;j++)
            for(int i = 0; i < 3; i++)
                R(i,j)=R_array[i+j*3];  

        sun::UnitQuaternion current_quat(R);

        // Calcolo errore
        double delta_p = TooN::norm(current_pos - cartesian_goal.goal_position);
        double delta_q_norm = TooN::norm( (cartesian_goal.goal_quaternion*inv(current_quat)).getV() );


        if(delta_p < 0.01 && delta_q_norm < 0.01) 
            traj_running = false; // La traiettoria può considerarsi terminata

    }

    bool press_y_gripper(){
      char carattere = 'n';

      while (ros::ok() && carattere != 'y') {
        std::cout << "Il gripper sta per muoversi, premere y per continuare o "
                     "n per abortire l'operazione e il programma \n";
        carattere = getchar();
        if (carattere == 'n')
          return false;
      };
    }

    

   
}