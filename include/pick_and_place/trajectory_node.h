
#pragma once

#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

// Server , Actions
#include <pick_and_place/SetTraj.h>
#include <controller_manager_msgs/ControllerState.h>
#include <controller_manager_msgs/SwitchController.h>


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


    ros::ServiceClient client_set_traj;

    TooN::Vector<3,double> goal_position;
    sun::UnitQuaternion goal_quat;

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
    
  
    bool set_goal_and_call_srv(TooN::Vector<3,double> goal_position, sun::UnitQuaternion goal_quaternion, double Tf){
        
        pick_and_place::SetTraj set_traj_msg;

        set_traj_msg.request.goal_position.x = goal_position[0];
        set_traj_msg.request.goal_position.y = goal_position[1];
        set_traj_msg.request.goal_position.z = goal_position[2];

        set_traj_msg.request.goal_quaternion.w = goal_quaternion.getS();
        set_traj_msg.request.goal_quaternion.x = goal_quaternion.getV()[0];
        set_traj_msg.request.goal_quaternion.y = goal_quaternion.getV()[1];
        set_traj_msg.request.goal_quaternion.z = goal_quaternion.getV()[2];

        set_traj_msg.request.Tf = Tf;

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
        double delta_p = TooN::norm(current_pos - goal_position);
        double delta_q_norm = TooN::norm( (goal_quat*current_quat).getV() );


        if(delta_p < 0.01 && delta_q_norm < 0.01) 
            traj_running = false; // La traiettoria può considerarsi terminata

    }


   
}