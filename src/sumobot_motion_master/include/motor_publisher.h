#include <ros/ros.h>
#include <string>
#include <sumobot_msgs/MotorPowers.h>

class MotorPublisher {
public:
    MotorPublisher(ros::NodeHandle *n, std::string topic_name);
    ~MotorPublisher();

    // Send a message to the motor drivers
    void send_message(MotorPowers msg);
private:
    // The publisher that sends the messages
    ros::Publisher m_publisher;
};