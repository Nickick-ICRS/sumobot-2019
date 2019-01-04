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

#include "listener.h"

Listener::Listener(ros::NodeHandle *n, std::string topic_name) {
    node_handle = n;
    // Subscribe to the topic and register the callback
    m_subscriber = 
        node_handle->subscribe(topic_name, 
                              100, 
                              &Listener::receive_motor_instructions, 
                              this);
    m_left_motor_target = 0;
    m_right_motor_target = 0;
}

Listener::~Listener() {
    // dtor
}

void Listener::receive_motor_instructions(const sumobot_msgs::MotorPowers::ConstPtr& msg) {
    // Set motor powers based on received values
    if(msg->left_power > 100)
        m_left_motor_target = 100;
    else
        m_left_motor_target = msg->left_power;
    if(msg->left_direction)
        m_left_motor_target |= DIRECTION_MASK;
    
    if(msg->right_power > 100)
        m_right_motor_target = 100;
    else
        m_right_motor_target = msg->right_power;
    if(msg->right_direction)
        m_right_motor_target |= DIRECTION_MASK;
}
