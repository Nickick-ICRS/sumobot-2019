#include <iostream>
#include <ros/ros.h>

#include "xbox_listener.h"
#include "motor_publisher.h"

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

}
