#ifndef __IMU_H__
#define __IMU_H__
#include <Arduino.h>

typedef struct{
    int16_t accel_data[3];
    int16_t gyro_data[3];
    int16_t mag_data[3];
} Packet;

typedef struct{
    double x;
    double y;
    double z;
} Vector3;

// Pins: 
// CS - 10
// MOSI - 11
// MISO - 12
// SCK - 13
// INT - 22

class MPU9250{
    public:
    MPU9250();

    void update();
    void ISR();

    Vector3 get_rotation();
    Vector3 get_accel();

    private:
    bool data_ready = false;

    Vector3 rotation;
    Vector3 acceleration;

    bool impulse_detected = false;

    void write_reg(uint8_t address, uint8_t val);
    uint8_t read_reg(uint8_t address);
    void burst_read(uint8_t address, uint8_t* buffer, uint8_t length);

    void process_data(Packet data);

    void filter_accel(Vector3 new_accel);
    void filter_rotation(Vector3 new_rot);

    void check_for_impulse();
};


extern MPU9250 MPU;

#endif