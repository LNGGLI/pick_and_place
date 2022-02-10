
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
    double Tf = 20;

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

        std::array<double,16> transform;
        for(int i = 0; i < 16 ; i++)
            transform[i]= msg->O_T_EE[i]; // ATTENZIONE la O_T_EE è passata per colonne!
        
        double R_array[9] = {transform[0],transform[1],transform[2],
                            transform[4],transform[5],transform[6],
                            transform[8],transform[9],transform[10]};

        // Posizione 
        TooN::Vector<3,double> p({transform[12],transform[13],transform[14]});

        // Matrice di rotazione CONTROLLARE
        TooN::Matrix<3,3> R;
        for(int j = 0; j < 3;j++)
            for(int i = 0; i < 3; i++)
                R(i,j)=R_array[i+j*3];  

        TooN::SO3<double> R_so3(R);
        
        // Costruzione matrice di trasformazione omogenea
        initial_transform = TooN::SE3<double>(R_so3,p);
        
        initial_read = true;

        // Stampa posizione e matrice di rotazione
        
        std::cout << "Posizione iniziale: \n"
                  << "x:" <<  initial_transform.get_translation()[0] << "\n"
                  << "y:" <<  initial_transform.get_translation()[1] << "\n"
                  << "z:" <<  initial_transform.get_translation()[2] << "\n";

        std::cout << "Matrice di rotazione iniziale: \n";
        for(int i = 0; i < 3;i++){
            for(int j = 0; j < 3; j++)
                std::cout << R(i,j) << " ";
            std::cout << std::endl;
        }
        std::cout << "\n";

        for(int i = 0; i < 7; i++)
            initial_conf[i] = msg->q[i];

    }



   
}