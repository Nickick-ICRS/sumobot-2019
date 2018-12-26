#ifndef __LISTENER_H
#define __LISTENER_H

#include <ros/ros.h>
#include <string>
#include <sumobot_msgs/MotorPowers.h>

/*
 * A class to listen on a ROS topic for motor instructions
 */

// Masks to easily get the power and direction from the motors
#define DIRECTION_MASK 0b10000000
#define POWER_MASK     0b01111111

class Listener {
public:
    Listener(ros::NodeHandle *n, std::string topic_name);
    ~Listener();

    char get_left_motor_target() { return m_left_motor_target; };
    char get_right_motor_target() { return m_right_motor_target; };

private:
    // Target motor powers received from the topic
    // The 1st bit is the direction
    char m_left_motor_target;
    char m_right_motor_target;
    // The subscriber which listens to the topic
    ros::Subscriber m_subscriber;
    // Pointer to this node's handle
    ros::NodeHandle *node_handle;
    // Callback to process information from the topic
    void receive_motor_instructions(const sumobot_msgs::MotorPowers::ConstPtr& msg); 
};

#endif // __LISTENER_H
