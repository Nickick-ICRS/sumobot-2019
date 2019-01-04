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
