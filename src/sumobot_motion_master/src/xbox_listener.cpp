#include "xbox_listener.h"

XboxListener::XboxListener(ros::nodeHandle *n, std::string topic_name) {
    node_handle = n;
    m_subscriber = n->subscribe(topic_name,
                                100,
                                &XboxListener::receive_xbox_data,
                                this);
}

XboxListener::~XboxListener() {
    // dtor
}

void XboxListener::receive_xbox_data(const sensor_msgs::Joy::ConstPtr& msg) {
    // lx
    msg->axes[0];
    // ly
    msg->axes[1];
    // rx
    msg->axes[2];
    // ry
    msg->axes[3];

    // a
    msg->buttons[0];
    // b
    msg->buttons[1];
    // x
    msg->buttons[2];
    // y
    msg->buttons[3];
}
