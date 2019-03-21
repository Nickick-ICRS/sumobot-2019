/********************************************************************
 ********************************************************************
 * Copyright 2019 Nick Hafner                                       *
                                                                    *
 * Permission is hereby granted, free of charge, to any person      *
 * obtaining a copy of this software and associated documentation   *
 * files (the "Software"), to deal in the Software without          *
 * restriction, including without limitation the rights to use,     *
 * copy, modify, merge, publish, distribute, sublicense, and/or     *
 * sell copies of the Software, and to permit persons to whom the   *
 * Software is furnished to do so, subject to the following         *
 * conditions:                                                      *
                                                                    *
 * The above copyright notice and this permission notice shall be   *
 * included in all copies or substantial portions of the Software.  *
                                                                    *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,  *
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES  *
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND         *
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT      *
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,     *
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING     *
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR    *
 * OTHER DEALINGS IN THE SOFTWARE.                                  *
 ********************************************************************
 ********************************************************************/

#include <iostream>
#include <pigpio.h>
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
    // Setup pigpio for GPIO stuff
    // Note that this will run for any arm processor
    // The point is that this can still be safely compiled on a laptop
    #ifdef __arm__
    std::cout << "We're running on a RPi! Setting up GPIO!\n";
    if(gpioInitialise() < 0) {
        std::cout << "Failed to init\n";
        gpioTerminate();
        return -1;
    }
    #else
    std::cout << "This is not a RPi. Most features won't work. :(\n";
    #endif // __arm__    
    
    // Create PWMControllers
    PWMController left_motor((*config_data)["kp"].asFloat(),
                             (*config_data)["ki"].asFloat(),
                             (*config_data)["kd"].asFloat(),
                             (*config_data)["left_motor_pwm_pin"].asUInt(),
                             (*config_data)["left_motor_direction_pin"].asUInt());
    PWMController right_motor((*config_data)["kp"].asFloat(),
                              (*config_data)["ki"].asFloat(),
                              (*config_data)["kd"].asFloat(),
                              (*config_data)["right_motor_pwm_pin"].asUInt(),
                              (*config_data)["right_motor_direction_pin"].asUInt());
    

    // Create Listener
    Listener command_listener(&node_handle, 
                              (*config_data)["topic_name"].asString());

    ros::Rate sleeper(100);

    while(ros::ok()) {
        // Update motor powers and directions
        
        if(command_listener.get_left_motor_target() & DIRECTION_MASK) {
            left_motor.set_motor_power(
                command_listener.get_left_motor_target() & POWER_MASK
            );
        }
        else {
            left_motor.set_motor_power(
                -command_listener.get_left_motor_target() & POWER_MASK
            );

        }

        if(command_listener.get_right_motor_target() & DIRECTION_MASK) {
            right_motor.set_motor_power(
                command_listener.get_right_motor_target() & POWER_MASK
            );
        }
        else {
            right_motor.set_motor_power(
                -command_listener.get_right_motor_target() & POWER_MASK
            );
        }
        
        // Check for ros updates (e.g. topic messages)
        ros::spinOnce();
        sleeper.sleep();
    }
    
    // Free memory
    gpioTerminate();
    return 0;
}
