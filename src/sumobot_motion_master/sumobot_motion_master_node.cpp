#include <iostream>
#include <ros/ros.h>
#include <chrono>

#include "xbox_listener.h"
#include "motor_publisher.h"
#include "motion_controller.h"

using namespace std::chrono;

int main(int argc, char **argv) {
    // Load config files

    // Initialise ROS
    ros::init(argc, argv, "node_name");
    // Create the node handle
    ros::NodeHandle node_handle;

    // Create listener for xbox controller data
    XboxListener controller(&node_handle, "topic_name");
    // Create publisher to send motor power data
    MotorPublisher publisher(&node_handle, "topic_name");
    // Create motion controller to handle data processing
    MotionController motion_controller;

    // Limit the update rate of the motion_controller to 100 Hz
    ros::Rate sleeper(100);

    high_resolution_clock clock;

    auto current_time = clock.now();

    while(ros::ok()) {
        auto dt_us = duration_cast<microseconds>(
            clock.now() - current_time
        );
        current_time = current_time + dt_us;

        // Update the motion controller with the elapsed time
        motion_controller.update(dt_us.count()/1000000.0f);
        
        std::cout << dt_us.count() / 1000000.0f  << std::endl;

        // Check for ros updates (e.g. messages)
        ros::spinOnce();

        // Maintain 100 Hz
        sleeper.sleep();
    }
}
