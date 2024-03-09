#ifndef __COMMANDS_H__
#define __COMMANDS_H__

typedef enum{
    NOP = 0x00,
    SET_WHEEL_SPEED = 0x01, // 4 double array of speeds (rpm)
    STOP_WHEELS = 0x02, // No args
    GET_WHEEL_SPEED = 0x03, // Returns 4 double array of speeds (rpm)
    GET_WHEEL_POSITION = 0x04, // Returns 4 double array of position (revolutions)
    SET_TURN_ANGLE = 0x07, // int32 turn angle in degrees
    GET_ACCELERATION = 0x08, // Returns 3 float array of acceleration values in XYZ G's
    GET_ANGLE = 0x09 // Returns 3 float array of angles in XYZ degrees
} command_t;

typedef enum{
    RETURN_WHEEL_SPEED = 0x05,
    RETURN_WHEEL_POSITION = 0x06,
    RETURN_ACCELERATION = 0x0a,
    RETURN_ANGLE = 0x0b,
    IMPULSE_DETECTED = 0x0c,
    SEND_STRING = 0x0d
} return_command_t;

#endif