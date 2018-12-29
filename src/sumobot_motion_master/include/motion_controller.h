#ifndef __MOTION_CONTROLLER_H
#define __MOTION_CONTROLLER_H

#include <sensor_msgs/Joy.h>
#include <sumobot_msgs/MotorPowers.h>

#include "motor_publisher.h"

// Helper struct to allow floating point acceleration of 
// the integer motor powers
struct MotorPowerFloats {
    float left_power;
    float right_power;
    bool left_direction;
    bool right_direction;
};

bool operator!=(const MotorPowerFloats& lhs, const MotorPowerFloats& rhs);
bool operator==(const MotorPowerFloats& lhs, const MotorPowerFloats& rhs);

class MotionController {
public:
    MotionController(MotorPublisher *p);
    ~MotionController();

    void process_axes_data(float *left, float *right);
    void process_button_data(bool a, bool b, bool x, bool y);
    bool update(float dt_s);
private:
    // Pointer to the publisher of motor power data
    MotorPublisher *publisher;

    // Target for the motor powers to reach
    MotorPowerFloats m_target_powers;
    // Current values of the motor powers
    MotorPowerFloats m_motor_powers;
    // If this rises past a certain threshhold 
    // the robot needs to stop for safety reasons
    float m_time_since_last_xbox_command;
    // Acceleration amount (in % per second)
    float m_acceleration;
    // Whether the powers have been changed
    bool m_powers_changed;

    /* CONFIG Variables */

    // Deadband width for the left and right joysticks
    float m_left_deadband;
    float m_right_deadband;

    // Maximum acceleration (in % per second) of the motors
    float m_max_acceleration;
    
    // Motor constants to account for uneven motors
    // This robot has no feedback so these are just set multipliers
    float m_left_multiplier;
    float m_right_multiplier;

    // The % of PWM at which the torque is too low to move the robot
    char m_motor_zero_power;

    // The maximum amount of time the robot will wait for an xbox command
    // before timing out
    float m_timeout_s;

    /* Functions */

    // Send the motor powers
    void send_data();
    // Update motor powers
    void update_motor_powers(float dt_s);

    // Run if the respective button is pressed
    void button_a_callback();
    void button_b_callback();
    void button_x_callback();
    void button_y_callback();
};

#endif // __MOTION_CONTROLLER_H
