#include <iostream>
#include <wiringPi.h>
#include <ros/ros.h>

#include "listener.h"
#include "pwm_controller.h"
#include "config.h"
#include "json/json.h"

int main(int argc, char **argv) {
    // Load config files
    ConfigManager cfg_mgr;
    // Store config data
    Json::Value *config_data = cfg_mgr.get_config_data();
    
    // Initialise ROS
    ros::init(argc, argv, (*config_data)["node_name"].asString());
    // Create the node handle
    ros::NodeHandle node_handle;
    // Setup wiringPi for GPIO stuff
    // Note that this will run for any arm processor
    // The point is that this can still be safely compiled on a laptop
    #ifdef __arm__
    std::cout << "We're running on a RPi! Setting up GPIO!\n";
    wiringPiSetup();
    #else
    std::cout << "This is not a RPi. Most features won't work. :(\n";
    #endif // __arm__    
    
    // Create PWMControllers
    PWMController left_motor((*config_data)["pwm_frequency"].asUInt(),
                             (*config_data)["left_motor_pwm_pin"].asUInt(),
                             (*config_data)["left_motor_direction_pin"].asUInt());
    PWMController right_motor((*config_data)["pwm_frequency"].asUInt(),
                              (*config_data)["right_motor_pwm_pin"].asUInt(),
                              (*config_data)["right_motor_direction_pin"].asUInt());

    // Create Listener
    Listener command_listener(&node_handle, 
                              (*config_data)["topic_name"].asString());
    
    while(ros::ok()) {
        // Update motor powers and directions
        left_motor.set_motor_power(
            command_listener.get_left_motor_target() & POWER_MASK
        );
        left_motor.set_motor_direction(
            command_listener.get_left_motor_target() & DIRECTION_MASK
        );
        right_motor.set_motor_power(
            command_listener.get_right_motor_target() & POWER_MASK
        );
        right_motor.set_motor_direction(
            command_listener.get_right_motor_target() & DIRECTION_MASK
        );
        // Check for ros updates (e.g. topic messages)
        ros::spinOnce();
    }

    return 0;
}
