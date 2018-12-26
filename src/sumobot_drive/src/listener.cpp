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
