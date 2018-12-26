#ifndef __PWM_CONTROLLER_H
#define __PWM_CONTROLLER_H

#include <wiringPi.h>
#include <thread>
#include <chrono>

/*
 * PWM Control for the motors used in The Shoveller
 * When compiling ensure to include -lwiringPi for wiringPi
 */

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
};

#endif // MOTOR_PWM_H
