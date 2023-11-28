#include "Motor.h"
#include <Arduino.h>
#include <functional>

#define PWM_FREQ (10.0)

#define ENC_PPR (6*2*30.0)
#define MAX_RPM (11000/30.0)

#define Kp 100.0
#define Ki 40.0
#define Kd 0.0


Motor::Motor(const uint8_t pwm_pin, const uint8_t phase_pin, const uint8_t enca_pin, const uint8_t encb_pin, void (*enca_func)(), void (*encb_func)()){
    pinMode(pwm_pin, OUTPUT);
    pinMode(phase_pin, OUTPUT);

    pinMode(enca_pin, INPUT_PULLUP);
    pinMode(encb_pin, INPUT_PULLUP);

    _enca_pin = enca_pin;
    _encb_pin = encb_pin;
    _pwm_pin = pwm_pin;
    _phase_pin = phase_pin;

    attachInterrupt(digitalPinToInterrupt(enca_pin), enca_func, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encb_pin), encb_func, CHANGE);
}

void Motor::is_reversed(bool rev){
    reversed = rev;
}

void Motor::EncA_ISR(){
    if(digitalRead(_enca_pin) == digitalRead(_encb_pin)) _position += 1/ENC_PPR;
    else _position -= 1/ENC_PPR;
}

void Motor::EncB_ISR(){
    if(digitalRead(_enca_pin) == digitalRead(_encb_pin)) _position -= 1/ENC_PPR;
    else _position += 1/ENC_PPR;
}

void Motor::set_target_speed(double speed){
    if(speed > MAX_RPM) return;
    else if(speed < -MAX_RPM) return;

    _target_speed = reversed ? -speed : speed;
}

double Motor::get_position(){
    return _position;
}

double Motor::get_speed(){
    return _current_speed;
}

void Motor::update_PID(){
    uint32_t this_time = micros();

    uint32_t dt = this_time - last_time;
    if(dt < 1000000/PWM_FREQ) return;
    last_time = this_time;

    _current_speed = (_position - last_position)*PWM_FREQ*60.0;
    last_position = _position;
    
    double err_p = (_target_speed - _current_speed);
    err_i += err_p/PWM_FREQ;
    double err_d = (err_p - last_err)*PWM_FREQ;

    last_err = err_p;

    double error = Kp*err_p + Ki*err_i + Kd*err_d;

    uint8_t dir = error < 0;
    error = abs(error);

    if(error > MAX_PWM){
        error = MAX_PWM;
    }

    if(_current_speed == 0 && _target_speed == 0) error = 0;

    uint32_t last = analogWriteResolution(PWM_BITS);
    analogWrite(_pwm_pin, (uint32_t)error);
    digitalWrite(_phase_pin, dir);
    analogWriteResolution(last);
}