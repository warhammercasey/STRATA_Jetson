#include <Arduino.h>
#include "IMU.h"
#include <SPI.h>
#include <math.h>

#define INT_PIN 22
#define CS_PIN 10

#define GYRO_FS 32000.0
#define GYRO_FULL_SCALE 250 // dps

#define ACCEL_FULL_SCALE 4 // +/-4g

#define MAGNETOMETER_ADDRESS 0x0C
#define MAG_FULL_SCALE 4912 // uT

#define ACCEL_WEIGHT 0.2 // Lower values mean more filtering
#define MAX_ACCEL_WEIGHT 5
#define MAG_WEIGHT 0.2

#define SIZE(a) (sizeof(a)/sizeof(a[0]))

MPU9250 MPU = MPU9250();

const double accel_coeffs[] = {-9.77915780971477e-19,-5.84581064237915e-17,-1.71551099381875e-15,-3.29368828694934e-14,-4.65204212672488e-13,-5.15306553948023e-12,-4.66030628522594e-11,-3.53699347110423e-10,-2.29803479902555e-09,-1.29734948371873e-08,-6.43747829173923e-08,-2.83281749207533e-07,-1.11328542020765e-06,-3.92854168932828e-06,-1.24980755240092e-05,-3.59434569675052e-05,-9.35695920380671e-05,-0.000220412572798612,-0.000468572817886037,-0.000893367704697882,-0.00150784452487967,-0.00219117975799770,-0.00255507956731086,-0.00181136820928633,0.00126859947412332,0.00818931703857967,0.0202914008018466,0.0380941759204974,0.0606486591083423,0.0852424025474306,0.107744812913818,0.123630601351059,0.129370544569634,0.123630601351059,0.107744812913818,0.0852424025474306,0.0606486591083422,0.0380941759204974,0.0202914008018466,0.00818931703857964,0.00126859947412331,-0.00181136820928634,-0.00255507956731086,-0.00219117975799770,-0.00150784452487967,-0.000893367704697883,-0.000468572817886037,-0.000220412572798612,-9.35695920380672e-05,-3.59434569675052e-05,-1.24980755240092e-05,-3.92854168932828e-06,-1.11328542020765e-06,-2.83281749207533e-07,-6.43747829173923e-08,-1.29734948371873e-08,-2.29803479902556e-09,-3.53699347110423e-10,-4.66030628522594e-11,-5.15306553948023e-12,-4.65204212672488e-13,-3.29368828694934e-14,-1.71551099381875e-15,-5.84581064237916e-17,-9.77915780971478e-19};
const double rot_coeffs[] = {2.15186188203270e-10,6.95653584430703e-09,1.08849684002894e-07,1.09802484597226e-06,8.02464128430732e-06,4.52675190421164e-05,0.000205057765334070,0.000766193068821825,0.00240707521300378,0.00644880105334868,0.0148919105012547,0.0298847977508294,0.0524429764081370,0.0808545042876122,0.109898819099703,0.131999331338964,0.140292054612825,0.131999331338964,0.109898819099703,0.0808545042876122,0.0524429764081370,0.0298847977508294,0.0148919105012547,0.00644880105334868,0.00240707521300378,0.000766193068821825,0.000205057765334070,4.52675190421164e-05,8.02464128430732e-06,1.09802484597226e-06,1.08849684002894e-07,6.95653584430703e-09,2.15186188203270e-10};

SPISettings settings(1000000, MSBFIRST, SPI_MODE0);

void mpu_isr();

MPU9250::MPU9250(){
    return;
    pinMode(INT_PIN, INPUT_PULLDOWN);
    //pinMode(CS_PIN, OUTPUT);
    //digitalWrite(CS_PIN, HIGH);

    Serial.println("Starting IMU...");
    
    SPI.begin();

    return;

    write_reg(26, 0); // Config register
    write_reg(27, 0b10); // Gyro config. Set FCHOICE
    write_reg(28, 1 << 3); // Accel config. Set full scale to 4g
    write_reg(29, 0); // Accel config 2. Set DLPF
    write_reg(35, (0b1111 << 3) | 1); // Enable gyro, accel, and mag in fifo
    write_reg(55, (1 << 4)); // INT pin config. Set any read to clear int
    write_reg(56, 1); // Enable interrupt
    

    // Magnetometer registers
    // Set to continuous measurement mode and 16-bit output
    write_reg(49, MAGNETOMETER_ADDRESS);
    write_reg(50, 0x0A);
    write_reg(51, (0b0010) | (1 << 4));
    write_reg(51, 1 << 7);
    
    // Enable magnetometer
    write_reg(37, (1 << 7) | MAGNETOMETER_ADDRESS);
    write_reg(38, 0x03);
    write_reg(39, (1 << 7) | 6);

    attachInterrupt(digitalPinToInterrupt(INT_PIN), mpu_isr, RISING);


}

void MPU9250::update(){
    if(!data_ready) return;
    data_ready = false;

    SPI.beginTransaction(settings);
    digitalWrite(CS_PIN, LOW);

    SPI.transfer((1 << 7) | 114);
    uint16_t count = SPI.transfer16(0);

    digitalWrite(CS_PIN, HIGH);
    SPI.endTransaction();

    if(count < sizeof(Packet)){
        uint8_t data_buff[count] = {0};
        burst_read(116, data_buff, count);
        return;
    }

    uint8_t packets = (count/sizeof(Packet))*sizeof(Packet);    

    Packet data_buff[packets];
    burst_read(116, (uint8_t*)data_buff, packets/sizeof(Packet));
    
    for(uint8_t i = 0; i < packets; i++){
        process_data(data_buff[i]);
    }
}

Vector3 MPU9250::get_rotation(){
    return rotation;
}

Vector3 MPU9250::get_accel(){
    return acceleration;
}

void MPU9250::process_data(Packet data){
    double gyro[3];
    double accel[3];
    double mag[3];

    for(uint8_t i = 0; i < 3; i++){
        gyro[i] = ((double)data.gyro_data[i])/0x7FFF*GYRO_FULL_SCALE;
        accel[i] = ((double)data.accel_data[i])/0x7FFF*ACCEL_FULL_SCALE;
        mag[i] = ((double)data.mag_data[i])/0x7FFF*MAG_FULL_SCALE;
    }

    Vector3 new_accel = {accel[0], accel[1], accel[2]};
    Vector3 new_gyro = {gyro[0], gyro[1], gyro[2]};
    Vector3 new_mag = {mag[0], mag[1], mag[2]};

    filter_accel(new_accel);



    double accel_rot_x = -atan2(new_accel.y, new_accel.z)*180/M_PI;
    double accel_rot_y = atan2(new_accel.x, new_accel.z)*180/M_PI;
    double mag_rot_z = atan2(new_mag.y, new_mag.x)*180/M_PI;

    double accel_magnitude = sqrt(new_accel.x*new_accel.x + new_accel.y*new_accel.y + new_accel.z*new_accel.z);

    double accel_weight = (accel_magnitude < 1) ? (MAX_ACCEL_WEIGHT*accel_magnitude) : (max((2 - accel_magnitude)*MAX_ACCEL_WEIGHT, 0));

    Vector3 new_rotation;
    new_rotation.x = (accel_weight*accel_rot_x + rotation.x + new_gyro.x/GYRO_FS)/(accel_weight + 1);
    new_rotation.y = (accel_weight*accel_rot_y + rotation.y + new_gyro.y/GYRO_FS)/(accel_weight + 1);
    new_rotation.z = (MAG_WEIGHT*mag_rot_z + rotation.z + new_gyro.z/GYRO_FS)/(MAG_WEIGHT + 1);

    filter_rotation(new_rotation);
}

void MPU9250::check_for_impulse(){
    
}

void MPU9250::filter_rotation(Vector3 new_rot){
    static Vector3 rot_vals[SIZE(rot_coeffs)] = {0};
    static uint16_t idx = 0;

    rot_vals[idx] = new_rot;

    Vector3 filt_sum = {0, 0, 0};
    for(uint16_t i = 0; i < SIZE(rot_coeffs); i++){
        filt_sum.x += rot_coeffs[i]*rot_vals[(i + idx)%SIZE(rot_coeffs)].x;
        filt_sum.y += rot_coeffs[i]*rot_vals[(i + idx)%SIZE(rot_coeffs)].y;
        filt_sum.z += rot_coeffs[i]*rot_vals[(i + idx)%SIZE(rot_coeffs)].z;
    }

    if(idx == 0) idx = SIZE(rot_coeffs);
    else idx--;

    rotation = filt_sum;
}

void MPU9250::filter_accel(Vector3 new_accel){
    static Vector3 accel_vals[SIZE(accel_coeffs)] = {0};
    static uint16_t idx = 0;

    accel_vals[idx] = new_accel;

    Vector3 filt_sum = {0, 0, 0};
    for(uint16_t i = 0; i < SIZE(accel_coeffs); i++){
        filt_sum.x += accel_coeffs[i]*accel_vals[(i + idx)%SIZE(accel_coeffs)].x;
        filt_sum.y += accel_coeffs[i]*accel_vals[(i + idx)%SIZE(accel_coeffs)].y;
        filt_sum.z += accel_coeffs[i]*accel_vals[(i + idx)%SIZE(accel_coeffs)].z;
    }

    if(idx == 0) idx = SIZE(accel_coeffs) - 1;
    else idx--;

    acceleration = filt_sum;
}

void MPU9250::burst_read(uint8_t address, uint8_t* buffer, uint8_t length){
    SPI.beginTransaction(settings);
    digitalWrite(CS_PIN, LOW);

    SPI.transfer((1 << 7) | address);
    SPI.transfer(buffer, length);

    digitalWrite(CS_PIN, HIGH);
    SPI.endTransaction();
}

void MPU9250::write_reg(uint8_t address, uint8_t val){
    SPI.beginTransaction(settings);
    digitalWrite(CS_PIN, LOW);

    SPI.transfer(address);
    SPI.transfer(val);

    digitalWrite(CS_PIN, HIGH);
    SPI.endTransaction();
}

uint8_t MPU9250::read_reg(uint8_t address){
    SPI.beginTransaction(settings);
    digitalWrite(CS_PIN, LOW);

    SPI.transfer((1 << 7) | address);
    uint8_t out = SPI.transfer(0x00);

    digitalWrite(CS_PIN, HIGH);
    SPI.endTransaction();

    return out;
}

void MPU9250::ISR(){
    data_ready = true;
}

void mpu_isr(){
    MPU.ISR();
}