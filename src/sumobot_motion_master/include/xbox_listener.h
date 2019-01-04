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
