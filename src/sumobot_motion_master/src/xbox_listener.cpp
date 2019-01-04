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
