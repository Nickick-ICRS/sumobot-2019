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

#include "pwm_controller.h"

// Include for debugging on non-Pis
#ifndef __arm__
#include <iostream>
#endif // __arm__

PWMController::PWMController(int pwm_frequency_hz,
                             char motor_pwm_pin,
                             char motor_direction_pin):
                             m_pwm_pin(motor_pwm_pin),
                             m_direction_pin(motor_direction_pin),
                             m_duty_cycle_on_us(0),
                             m_current_state(0),
                             m_direction(0),
                             m_running(true) {
    m_duty_cycle_length_us = 1000.0f / (float)pwm_frequency_hz;
    #ifdef __arm__
    pinMode(m_direction_pin, OUTPUT);
    pinMode(m_pwm_pin, OUTPUT);
    #endif // __arm__
    // Initialise thread to update the PWM cycle
    pwm_thread = new std::thread(&PWMController::update_PWM, this);
    pwm_thread->detach();
}

PWMController::~PWMController() {
    m_running = false;
    pwm_thread->join();
    delete pwm_thread;
}

void PWMController::set_motor_power(char percent) {
    // Prevent spam below
    #ifndef __arm__
    float m_old_duty_cycle_on_us = m_duty_cycle_on_us;
    #endif // __arm__
    // If the direction pin is off then the duty cycle is on for full speed
    if(!m_direction) {
        m_duty_cycle_on_us = percent * m_duty_cycle_length_us / 100.0f;
    }
    // Otherwise the duty cycle is off for full speed
    else {
        m_duty_cycle_on_us = (100 - percent) * m_duty_cycle_length_us / 100.0f;
    }
    m_duty_cycle_off_us = m_duty_cycle_length_us - m_duty_cycle_on_us;
    // Terminal output when running on non-Pi
    #ifndef __arm__
    if(m_old_duty_cycle_on_us != m_duty_cycle_on_us) {
        std::cout << "PWM Period: " << m_duty_cycle_on_us << std::endl;
    }
    #endif // __arm__
}

void PWMController::set_motor_direction(bool direction) {
    if(m_direction != direction) {
        m_direction = direction;
        #ifdef __arm__
        digitalWrite(m_direction_pin, m_direction);
        // Terminal output when running on non-Pi
        #else
        std::cout << "Direction Changed!\n";
        #endif // __arm__
    }
}

void PWMController::update_PWM() {
    using namespace std::chrono;

    auto current_time = high_resolution_clock::now();

    while(m_running) {
        auto dt = duration_cast<microseconds>(
            high_resolution_clock::now() - current_time
        );
        // Update pin output depending on state and time since last change
        if((m_current_state && dt.count() >=  m_duty_cycle_on_us) ||
           (!m_current_state && dt.count() >= m_duty_cycle_off_us)) {
            current_time += dt;
            m_current_state = !m_current_state;
            #ifdef __arm__
            digitalWrite(m_pwm_pin, m_current_state);
            #endif // __arm__
        }
    }
}
