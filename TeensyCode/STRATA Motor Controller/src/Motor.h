#ifndef __MOTOR_H__
#define __MOTOR_H__

#include <Arduino.h>

#define PWM_BITS 14
#define MAX_PWM ((1 << PWM_BITS) - 1)


class Motor{
    public:
    Motor(const uint8_t pwm_pin, const uint8_t phase_pin, const uint8_t enca_pin, const uint8_t encb_pin, void (*enca_func)(), void (*encb_func)());
    
    void EncA_ISR();
    void EncB_ISR();

    void update_PID();

    void set_target_speed(double new_speed);

    double get_speed();
    double get_position();

    void is_reversed(bool rev);

    private:
    uint8_t _enca_pin, _encb_pin, _pwm_pin, _phase_pin;
    double _target_speed = 0;
    double _current_speed = 0;
    double err_i = 0;

    double _position = 0;

    double last_position = 0;
    double last_err = 0;

    double last_time = 0;

    bool reversed = false;

};

#endif