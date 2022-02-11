
#pragma once

#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>


#include <controller_manager_msgs/ControllerState.h>
#include <controller_manager_msgs/SwitchController.h>
#include <ros/ros.h>

#include <franka_msgs/FrankaState.h>
#include <TooN/TooN.h>
#include <TooN/se3.h>


using controller_manager_msgs::SwitchControllerRequest;
using controller_manager_msgs::SwitchControllerResponse;

namespace trajectory{

   
    TooN::SE3<double> initial_transform;
    TooN::Vector<7,double> initial_conf; 

    bool initial_read = false;
    double Tf = 10;

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
    
  


    void stateCB(const franka_msgs::FrankaState::ConstPtr& msg){

        double transform[16];
        for(int i = 0; i < 16 ; i++)
            transform[i]= msg->O_T_EE[i]; // ATTENZIONE la O_T_EE è passata per colonne!
       

        // Stampa matrice di transformazione
        std::cout << "Matrice di trasformazione iniziale: \n";
        TooN::Matrix<4,4,double> posa;
        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 4; j++)
            {
                posa[j][i] = transform[j+4*i];
            }
            std::cout << std::endl;
        }

         for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 4; j++)
            {
                std::cout << posa[i][j] << " ";
            }
            std::cout << std::endl;
        }

        for(int i = 0; i < 7; i++)
            initial_conf[i] = msg->q[i];
        
        std::cout << "Configurazione iniziale q0:  \n";
        for(int i = 0; i < 7; i++)
            std::cout << initial_conf[i] << " ";

        std::cout << "\n";
        
        initial_read = true;
    }



   
}