#ifndef __XBOX_LISTENER_H
#define __XBOX_LISTENER_H

#include <ros/ros.h>
#include <string>
#include <sensor_msgs/Joy.h>

#include "motion_controller.h"

class XboxListener {
public:
    XboxListener(ros::NodeHandle *n, 
                 MotionController* m, 
                 std::string topic_name);
    ~XboxListener();

private:
    // The subscriber which listens to the topic
    ros::Subscriber m_subscriber;
    // Pointer to the node's motion controller
    MotionController *motion_controller;
    // Callback to process information
    void receive_xbox_data(const sensor_msgs::Joy::ConstPtr& msg);
};

#endif // __XBOX_LISTENER_H<F3>
