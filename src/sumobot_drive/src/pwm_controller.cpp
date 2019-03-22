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

PWMController::PWMController(float kp, 
                             float ki, 
                             float kd,
                             char motor_pwm_pin,
                             char motor_direction_pin):
                             m_pwm_pin(motor_pwm_pin),
                             m_direction_pin(motor_direction_pin),
                             m_kp(kp),
                             m_ki(ki),
                             m_kd(kd),
                             m_i_err(0),
                             m_pr_diff(0),
                             m_running(true) {
    #ifdef __arm__
    gpioSetMode(m_direction_pin, PI_OUTPUT);
    gpioSetMode(m_pwm_pin, PI_OUTPUT);
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

void PWMController::set_motor_power(signed char percent) {
    // Set target power in W
    m_target_power = 0.015 * percent;
    percent = 2.55f * (float)percent;
#ifdef __arm__
    if(percent < 0) {
        percent = -percent;
        gpioWrite(m_direction_pin, 1);
    }
    else
        gpioWrite(m_direction_pin, 0);
    gpioPWM(m_pwm_pin, percent);
#endif 
}

void PWMController::update_PWM() {
    return;
    using namespace std::chrono;

    char dir;
    float pwm = 0;
    float err;
    float diff;

    auto current_time = high_resolution_clock::now();

    while(m_running) {
        auto dt = duration_cast<microseconds>(
            high_resolution_clock::now() - current_time
        );

        current_time += dt;

        float dt_s = dt.count() / 1000000;

        if(dt_s > 0.01)
            dt_s = 0.01;
        
        // Calculate stuf for pid
        err = m_target_power - get_motor_power();
        diff = 0.7*(err / dt_s) + 0.3 * m_pr_diff;
        m_pr_diff = diff;

        // Calculate pid
        float pid = m_kp * err + m_ki * m_i_err + m_kd * diff;

        pwm += pid;

        // Don't increase i_err if we're at max pwm
        if(fabs(pwm) < 255) 
            m_i_err += err * dt_s;
        #ifdef __arm__
        else if(pwm < 0) {
            pwm = -255;
            // Write to the pins
            gpioPWM(m_pwm_pin, -pwm);
            gpioWrite(m_direction_pin, 1);
        }
        else {
            pwm = 255;
            // Write to the pins
            gpioPWM(m_pwm_pin, -pwm);
            gpioWrite(m_direction_pin, 0);
        }
        #endif
        
        // Let CPU rest
        std::this_thread::sleep_for(milliseconds(1));
    }
}

float PWMController::get_motor_power() {
    return 0;
}
