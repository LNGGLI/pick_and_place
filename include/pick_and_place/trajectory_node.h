
#pragma once

#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>


#include <controller_manager_msgs/ControllerState.h>
#include <controller_manager_msgs/SwitchController.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>

#include <franka_msgs/FrankaState.h>

using controller_manager_msgs::SwitchControllerRequest;
using controller_manager_msgs::SwitchControllerResponse;


namespace trajectory{

   
    tf::Transform initial_transform;

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
    
    
    tf::Transform convertArrayToTf(const std::array<double, 16>& transform) {

        tf::Matrix3x3 rotation(transform[0], transform[4], transform[8], transform[1], transform[5],
                            transform[9], transform[2], transform[6], transform[10]);
        
        tf::Vector3 translation(transform[12], transform[13], transform[14]);

        return tf::Transform(rotation, translation);
        
    }


    void stateCB(const franka_msgs::FrankaState::ConstPtr& msg){

        std::array<double,16> transform;
        for(int i = 0; i < 16 ; i++)
            transform[i]= msg->O_T_EE[i]; // ATTENZIONE, CONTROLLARE SE VIENE COSTRUITA CORRETTAMENTE
                                         // la O_T_EE è passata per colonne!
                
        initial_transform = convertArrayToTf(transform); 

        initial_read = true;
    }



   
}