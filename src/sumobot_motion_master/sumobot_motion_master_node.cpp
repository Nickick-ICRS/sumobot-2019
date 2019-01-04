#include <iostream>
#include <ros/ros.h>
#include <chrono>

#include "xbox_listener.h"
#include "motor_publisher.h"
#include "motion_controller.h"
#include "config.h"
#include "motion_master_config.h"

using namespace std::chrono;

int main(int argc, char **argv) {
    // Create config managers
    ConfigManager xbox_cfg_mgr(std::string(getenv("HOME")) + 
                             "/.config/sumobot/sumobot_xbox_controller/", 
                             "config.json");
    ConfigManager motor_cfg_mgr(std::string(getenv("HOME")) + 
                             "/.config/sumobot/sumobot_drive/", 
                             "config.json");

    MotionMasterConfigManager cfg_mgr;

    Json::Value *xbox_config_data = xbox_cfg_mgr.get_config_data();
    Json::Value *motor_config_data = motor_cfg_mgr.get_config_data();
    Json::Value *config_data = cfg_mgr.get_config_data();
    
    // Initialise ROS
    ros::init(argc, argv, (*config_data)["node_name"].asString());
    // Create the node handle
    ros::NodeHandle node_handle;


    // Create publisher to send motor power data
    MotorPublisher publisher(&node_handle,
                             (*motor_config_data)["topic_name"].asString());
    // Create motion controller to handle data processing
    MotionController motion_controller(
        &publisher,
        (*config_data)["left_deadband"].asFloat(),
        (*config_data)["right_deadband"].asFloat(),
        (*config_data)["max_acceleration"].asFloat(),
        (*config_data)["left_multiplier"].asFloat(),
        (*config_data)["right_multiplier"].asFloat(),
        (*config_data)["motor_zero_power"].asInt(),
        (*config_data)["timeoue_s"].asFloat()
    );
    // Create listener for xbox controller data
    XboxListener controller(&node_handle, 
                            &motion_controller, 
                            (*xbox_config_data)["topic_name"].asString());

    // Limit the update rate of the motion_controller to 100 Hz
    ros::Rate sleeper(100);

    // Time keeping
    high_resolution_clock clock;
    auto current_time = clock.now();

    while(ros::ok()) {
        auto dt_us = duration_cast<microseconds>(
            clock.now() - current_time
        );
        current_time = current_time + dt_us;

        // Update the motion controller with the elapsed time
        if(!motion_controller.update(dt_us.count()/1000000.0f)) {
            std::cout << "Lost signal with xbox controller. Closing node\n";
            return 1;
        }
        
        // Check for ros updates (e.g. messages)
        ros::spinOnce();

        // Maintain 100 Hz
        sleeper.sleep();
    }
    return 0;
}
