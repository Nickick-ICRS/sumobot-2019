#include "motor_pwm.h"

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
    pinMode(m_direction_pin, OUTPUT);
    pinMode(m_pwm_pin, OUTPUT);
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
    // If the direction pin is off then the duty cycle is on for full speed
    if(!m_direction) {
        m_duty_cycle_on_us = percent * m_duty_cycle_length_us / 100.0f;
    }
    // Otherwise the duty cycle is off for full speed
    else {
        m_duty_cycle_on_us = (100 - percent) * m_duty_cycle_length_us / 100.0f;
    }
    m_duty_cycle_off_us = m_duty_cycle_length_us - m_duty_cycle_on_us;
}

void PWMController::set_motor_direction(bool direction) {
    m_direction = direction;
    digitalWrite(m_direction_pin, m_direction);
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
            digitalWrite(m_pwm_pin, m_current_state);
        }
    }
}
