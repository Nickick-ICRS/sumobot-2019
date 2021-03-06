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
    MotionController(MotorPublisher *p,
                     float left_deadband,
                     float right_deadband,
                     float max_acceleration,
                     float left_multiplier,
                     float right_multiplier,
                     int motor_zero_power,
                     float timeoue_s);
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
