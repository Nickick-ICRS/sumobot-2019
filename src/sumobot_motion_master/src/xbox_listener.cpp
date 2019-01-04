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

#include "xbox_listener.h"

XboxListener::XboxListener(ros::NodeHandle *n,
                           MotionController *m,
                           std::string topic_name) {
    m_subscriber = n->subscribe(topic_name,
                                100,
                                &XboxListener::receive_xbox_data,
                                this);
    motion_controller = m;
}

XboxListener::~XboxListener() {
    // dtor
}

void XboxListener::receive_xbox_data(const sensor_msgs::Joy::ConstPtr& msg) {
    float left_axes[2];
    float right_axes[2];
    // lx
    left_axes[0] = msg->axes[0];
    // ly
    left_axes[1] = msg->axes[1];
    // rx
    right_axes[0] = msg->axes[2];
    // ry
    right_axes[1] = msg->axes[3];

    motion_controller->process_axes_data(left_axes, right_axes);

    // a
    bool a = msg->buttons[0];
    // b
    bool b = msg->buttons[1];
    // x
    bool x = msg->buttons[2];
    // y
    bool y = msg->buttons[3];

    if(a || b || x || y) {
        motion_controller->process_button_data(a, b, x, y);
    }
}
