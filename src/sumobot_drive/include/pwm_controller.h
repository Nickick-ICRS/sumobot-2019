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

#ifndef __PWM_CONTROLLER_H
#define __PWM_CONTROLLER_H

#ifdef __arm__
#include <pigpio.h>
#endif
#include <thread>
#include <chrono>
#include <cmath>

/*
 * PWM Control for the motors used in The Shoveller
 * When compiling ensure to include -lwiringPi for wiringPi
 */

class PWMController {
public:
    PWMController(float kp, 
                  float ki, 
                  float kd,
                  char motor_pwm_pin, 
                  char motor_direction_pin);
    ~PWMController();

    // Set the power of the motor (+/- 100%)
    void set_motor_power(signed char percent);
private:
    // Constants and variables for PID
    float m_kp, m_ki, m_kd;
    float m_i_err;
    float m_pr_diff;
    
    // Target for motor power
    volatile float m_target_power;
    // Pins
    char m_pwm_pin;
    char m_direction_pin;
    
    // Worker thread to regularly update the PWM pin
    std::thread *pwm_thread;
    // Flag to signal when the thread should end
    bool m_running;
    
    // Function to update the PWM state of the motor - run by above thread
    void update_PWM();
    // Function to check the current power of the motors
    float get_motor_power();
};

#endif // MOTOR_PWM_H
