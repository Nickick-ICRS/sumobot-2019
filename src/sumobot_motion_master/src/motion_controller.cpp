#include "motion_controller.h"

bool operator!=(const MotorPowerFloats& lhs, const MotorPowerFloats& rhs) {
    return !(lhs == rhs);
}

bool operator==(const MotorPowerFloats& lhs, const MotorPowerFloats& rhs) {
    if(lhs.left_power == rhs.left_power &&
       lhs.right_power == rhs.right_power &&
       lhs.left_direction == rhs.left_direction &&
       lhs.right_direction == rhs.right_direction)
        return true;
    return false;
}

MotionController::MotionController() {
    // Config variables
    m_left_deadband = 0.1;
    m_right_deadband = 0.1;
    m_max_acceleration = 100;
    m_left_multiplier = 1.0;
    m_right_multiplier = 1.0;
    m_motor_zero_power = 10;
    m_timeout_s = 30;

    // Changing variables
    m_target_powers.left_power = 0;
    m_target_powers.right_power = 0;
    m_target_powers.left_direction = 1;
    m_target_powers.right_direction = 1;
    m_motor_powers.left_power = 0;
    m_motor_powers.right_power = 0;
    m_motor_powers.left_direction = 1;
    m_motor_powers.right_direction = 1;
    m_time_since_last_xbox_command = 0;
    m_acceleration = m_max_acceleration / 2;
    m_powers_changed = false;
}

MotionController::~MotionController() {
    // dtor
}

void MotionController::process_axes_data(float *left, float *right) {
    // Reset timeout counter
    m_time_since_last_xbox_command = 0;
    // Turn on flag
    m_powers_changed = true;
    
    // lx - not used
    // left[0];
    // ly
    float forwards_vel = left[1];
    // rx
    float turning_vel = right[0];
    // ry - not used
    // right[1];
    
    // Set both directions as if not turning
    if(forwards_vel < 0) {
        m_target_powers.left_direction = 0;
        m_target_powers.right_direction = 0;
        // Set forwards_vel to positive
        forwards_vel = -forwards_vel;
    }
    else {
        m_target_powers.left_direction = 1;
        m_target_powers.right_direction = 1;
    }


    // Below the respective deadband width the joysticks 
    // positions are ignored, so re-normalise
    if(forwards_vel < m_left_deadband) {
        forwards_vel = 0;
    }
    else {
        forwards_vel = (forwards_vel - m_left_deadband)
                     / (1.0f - m_left_deadband);
    }
    if(abs(turning_vel) < m_right_deadband) 
        turning_vel = 0;
    else {
        turning_vel = (turning_vel - m_right_deadband)
                    / (1.0f - m_right_deadband);
    }

    // Set right and left velocities to maximum
    m_target_powers.left_power = forwards_vel * 100;
    m_target_powers.right_power = forwards_vel * 100;
    
    // If turning_vel > 0 then turn right
    if(turning_vel > 0) {
        // Now varies between -0.5 and 0.5
        turning_vel -= 0.5f;
        // When turning_vel is 0 then right_vel is 0
        // When turning_vel < 0 then right_vel should be positive
        // When turning_vel > 0 then right_vel should be negative
        // forwards_vel is normalised though so account for this at the end
        float right_vel = (0.5 - abs(turning_vel)) * forwards_vel / 0.5f;
        
        m_target_powers.right_power = right_vel * 100;

        if(turning_vel > 0) {
            m_target_powers.right_direction = 
                !m_target_powers.right_direction;
        }
    }
    // Else turn left
    else {
        // Now varies between -0.5 and 0.5
        turning_vel += 0.5f;
        // When turning_vel is 0 then left_vel is 0
        // When turning_vel > 0 then left_vel should be positive
        // When turning_vel < 0 then left_vel should be negative
        // forwards_vel is normalised though so account for this at the end
        float left_vel = (0.5 - abs(turning_vel)) * forwards_vel / 0.5f;
        
        m_target_powers.left_power = left_vel * 100;

        if(turning_vel < 0) {
            m_target_powers.left_direction = 
                !m_target_powers.left_direction;
        }
    }

    // Counteract for a stringer left or right motor
    m_target_powers.left_power *= m_left_multiplier;
    m_target_powers.right_power *= m_right_multiplier;
}

void MotionController::process_button_data(bool a,
                                            bool b,
                                            bool x,
                                            bool y) {
    // Reset timeout counter
    m_time_since_last_xbox_command = 0;
    // Turn on flag
    m_powers_changed = true;

    if(a)
        button_a_callback();
    if(b)
        button_b_callback();
    if(x)
        button_x_callback();
    if(y)
        button_y_callback();
}

void MotionController::update(float dt_s) {
    // Increment timeout counter
    m_time_since_last_xbox_command += dt_s;
    // If we've timedout then we need to stop moving
    if(m_time_since_last_xbox_command > m_timeout_s) {
        m_target_powers.left_power = 0;
        m_target_powers.right_power = 0;
    }
    // No point updating if nothing has changed - the PWM controller
    // doesn't care how often it receives data
    if(m_powers_changed) {
        update_motor_powers(dt_s);
        send_data();
        m_powers_changed = false;
    }
}

void MotionController::send_data() {
    sumobot_msgs::MotorPowers msg;
    msg.left_power = m_motor_powers.left_power;
    msg.right_power = m_motor_powers.right_power;
    msg.left_direction = m_motor_powers.left_direction;
    msg.right_direction = m_motor_powers.right_direction;

    // Send data
}

void MotionController::update_motor_powers(float dt_s) {
    // If there's a direction change we need to decelerate the motor
    if(m_target_powers.left_direction != m_motor_powers.left_direction) {
        // Decelerate until direction can swap
        if(m_motor_powers.left_power < m_motor_zero_power)
            m_motor_powers.left_direction = m_target_powers.left_direction;
        else
            m_motor_powers.left_direction -= m_acceleration * dt_s;
        // Prevent negative PWM
        if(m_motor_powers.left_direction < 0)
            m_motor_powers.left_direction = 0;
    }
    // No direction change so just move towards goals
    else {
        // Move left motor powers towards target
        // If within the movement step then set them equal
        if(abs(m_target_powers.left_power - m_motor_powers.left_power) 
                < m_acceleration * dt_s)
            m_motor_powers.left_power = m_target_powers.left_power;
        // If target is higher then increase power
        else if(m_target_powers.left_power > m_motor_powers.left_power)
            m_motor_powers.left_power += m_acceleration * dt_s;
        // If target is lower then decrease power
        else 
            m_motor_powers.left_power -= m_acceleration * dt_s;
    }

    // Repeat for right motor
    if(m_target_powers.right_direction != m_motor_powers.right_direction) {
        // Decelerate until direction can swap
        if(m_motor_powers.right_power < m_motor_zero_power)
            m_motor_powers.right_direction = m_target_powers.right_direction;
        else
            m_motor_powers.right_direction -= m_acceleration * dt_s;
        // Prevent negative PWM
        if(m_motor_powers.right_direction < 0)
            m_motor_powers.right_direction = 0;
    }
    else {
        // Move right motor powers towards target
        // If within the movement step then set them equal
        if(abs(m_target_powers.right_power - m_motor_powers.right_power) 
                < m_acceleration * dt_s)
            m_motor_powers.right_power = m_target_powers.right_power;
        // If target is higher then increase power
        else if(m_target_powers.right_power > m_motor_powers.right_power)
            m_motor_powers.right_power += m_acceleration * dt_s;
        // If target is lower then decrease power
        else 
            m_motor_powers.right_power -= m_acceleration * dt_s;
    }
}

void MotionController::button_a_callback() {
    std::cout << "Button A pressed!\n";
}

void MotionController::button_b_callback() {
    std::cout << "Button B pressed!\n";
}

void MotionController::button_x_callback() {
    std::cout << "Button X pressed\n";
}

void MotionController::button_y_callback() {
    std::cout << "Button Y pressed\n";
}
