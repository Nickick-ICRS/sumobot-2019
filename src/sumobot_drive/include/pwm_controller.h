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

#include <wiringPi.h>
#include <thread>
#include <chrono>

#include "mbed_serial.h"

class PWMController {
public:
    PWMController(int pwm_frequency_hz, 
                  char motor_pwm_pin, 
                  char motor_direction_pin);
    ~PWMController();

    // Set the duty cycle on period as a percentage of the total duty cycle
    void set_motor_power(char percent);
    // Change the direction of the motor
    void set_motor_direction(bool direction);
private:
    // Length of the duty cycle
    float m_duty_cycle_length_us;
    // Duty cycle on and off period lengths in us
    volatile float m_duty_cycle_on_us;
    volatile float m_duty_cycle_off_us;
    // Current state of output - on or off
    bool m_current_state;
    // Current direction / state of direction pin
    volatile bool m_direction;
    // Pins
    char m_pwm_pin;
    char m_direction_pin;
    
    // Worker thread to regularly update the PWM pin
    std::thread *pwm_thread;
    // Flag to signal when the thread should end
    bool m_running;
    // Function to update the PWM state of the motor - run by above thread
    void update_PWM();

    MbedSerial *m_serial;
};

#endif // MOTOR_PWM_H
