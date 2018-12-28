#include <motor_publisher.h>

MotorPublisher::MotorPublisher(ros::NodeHandle *n, std::string topic_name) {
    m_publisher = n->advertise<sumobot_msgs::MotorPowers>(topic_name, 100);
}

MotorPublisher::~MotorPublisher() {
    // dtor
}

void MotorPublisher::send_message(MotorPowers msg) {
    // Publish the data
    m_publisher.publish(msg);
}
