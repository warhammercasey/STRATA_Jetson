#include <Arduino.h>
#include "Motor.h"
#include "Commands.h"
#include "PWMServo.h"
#include "MPU9250.h"
#include <vector>

//#include "IMU.h"

#define MAX_SERVO_ANGLE 60

#define IMPULSE_TIME 0.2f
#define IMPULSE_POWER 0.06f

const uint8_t servo_pins[] = {10, 11, 22, 23};
const int32_t servo_offsets[] = {-1, 5, -2, 1}; // FrontRight, FrontLeft, BackLeft, BackRight
const bool reverse_servo[] = {false, false, true, true};


typedef struct{
  uint32_t timestamp;
  float accel;
} impulse_point_t;

void ENCA(uint8_t num);
void ENCB(uint8_t num);

bool parse_cmd(uint8_t cmd, uint8_t* data);
void handle_cmd();

void send_string(const char* str);

// PWM, PHASE, ENCA, ENCB
Motor motors[4] = {
  Motor(0, 1, 2, 3, [](){ENCA(0);}, [](){ENCB(0);}),
  Motor(4, 5, 6, 7, [](){ENCA(1);}, [](){ENCB(1);}),
  Motor(8, 9, 12, 13, [](){ENCA(2);}, [](){ENCB(2);}),
  Motor(14, 15, 16, 17, [](){ENCA(3);}, [](){ENCB(3);}) 
};

PWMServo servos[4];

MPU9250 mpu;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial5.begin(115200);

  Wire.begin();

  analogWriteResolution(PWM_BITS);
  
  delay(1000);

  for(uint8_t i = 0; i < sizeof(servo_pins)/sizeof(servo_pins[0]); i++){
    servos[i].attach(servo_pins[i]);
    servos[i].write(90);
  }

  MPU9250Setting setting;
  setting.accel_fs_sel = ACCEL_FS_SEL::A4G;
  setting.gyro_fs_sel = GYRO_FS_SEL::G500DPS;
  setting.mag_output_bits = MAG_OUTPUT_BITS::M16BITS;
  setting.fifo_sample_rate = FIFO_SAMPLE_RATE::SMPL_250HZ;
  setting.gyro_fchoice = 0x03;
  setting.gyro_dlpf_cfg = GYRO_DLPF_CFG::DLPF_250HZ;
  setting.accel_fchoice = 0x01;
  setting.accel_dlpf_cfg = ACCEL_DLPF_CFG::DLPF_99HZ;

  if (!mpu.setup(0x68, setting)) {  // change to your own address
    while (1) {
      Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
      delay(5000);
    }
  }

  motors[0].is_reversed(true);
  motors[1].is_reversed(false);
  motors[2].is_reversed(true);
  motors[3].is_reversed(false);

  //mpu.calibrateAccelGyro();

  Serial.println("Starting...");
  send_string("Starting");

  /*motors[0].set_target_speed(50);
  motors[1].set_target_speed(50);

  motors[2].set_target_speed(50);
  motors[3].set_target_speed(50);*/

  //delay(5000);
}

void loop() {
  // put your main code here, to run repeatedly:
  static std::vector<impulse_point_t> accel_vals;
  static bool in_impulse = false;

  if(mpu.update()){
    uint32_t current_time = micros();

    float accelX = mpu.getAccX();
    float accelY = mpu.getAccY();
    float accelZ = mpu.getAccZ();

    impulse_point_t new_point;
    new_point.accel = abs(sqrtf(accelX*accelX + accelY*accelY + accelZ*accelZ)-1);
    new_point.timestamp = current_time;

    accel_vals.push_back(new_point);

    while(accel_vals.size() > 0 && (current_time - accel_vals[0].timestamp)/1.0e6 > IMPULSE_TIME){
      accel_vals.erase(accel_vals.begin());
    }

    float impulse = 0;
    for(uint16_t i = 0; i < accel_vals.size() - 1; i++){
      float dt = (accel_vals[i + 1].timestamp - accel_vals[i].timestamp)/1.0e6;

      float mag = (accel_vals[i + 1].accel + accel_vals[i].accel)/2;

      impulse += mag*dt;
    }

    //Serial.printf("Buffer length: %i\tImpulse: %f\r\n", accel_vals.size(), impulse);

    if(impulse >= IMPULSE_POWER){
      if(!in_impulse){
        in_impulse = true;
        //Serial.println("Impulse detected");
        Serial5.write((uint8_t)IMPULSE_DETECTED);
      }
    }else{
      in_impulse = false;
    }
  }

  for(uint8_t i = 0; i < 4; i++){
    motors[i].update_PID();
  }

  /*static uint32_t last_time = 0;
  if(millis() - last_time > 1000){
    last_time = millis();
    //Serial5.write(1);
    //Serial.printf("Speed: %lf\tPosition: %lf\r\n", motors[0].get_speed(), motors[0].get_position());

    //Vector3 rot = MPU.get_rotation();
    //Serial.printf("Rotx: %f\tRoty: %f\tRotz: %f\r\n", mpu.getAccX(), mpu.getAccY(), mpu.getAccZ());
  }*/

  

  static uint8_t val = 0;
  if(Serial.available()){
    
    Serial.read();
    for(uint8_t i = 0; i < 4; i++){
      motors[i].set_target_speed(val*200);
    }

    if(val) Serial.println("Starting Motors");
    else Serial.println("Stopping Motors");

    val = !val;
  }

  handle_cmd();
}


void send_string(const char* str){
  Serial5.write((uint8_t)SEND_STRING);
  Serial5.printf("%s\n", str);
}

void handle_cmd(){
  static uint8_t cmd = 0;
  static uint16_t cmd_index = 0;
  static uint8_t rx_buffer[sizeof(double)*4];
  

  uint16_t count = Serial5.available();
  if(!count) return;

  while((!cmd || cmd > 0x0b) && count){
    cmd = Serial5.read();
    cmd_index = 0;
    if(!cmd || cmd > 0x0b) count--;
  }

  if(!cmd || cmd > 0x0b){
    cmd = 0;
    return;
  }

  while(count--){
    //Serial.printf("Cmd: %u\r\n", cmd);
    switch((command_t)cmd){
      case NOP:
      break;

      case SET_WHEEL_SPEED:
      rx_buffer[cmd_index] = Serial5.read();
      if(cmd_index >= sizeof(int32_t)*4){
        Serial.println("Set wheel speed");
        send_string("Set wheel speed");
        if(parse_cmd(cmd, rx_buffer)){
          Serial5.clear();
        }
        cmd = 0;
        cmd_index = 0;
        return;
      }
      break;

      case SET_TURN_ANGLE:
      rx_buffer[cmd_index] = Serial5.read();
      if(cmd_index >= sizeof(int32_t)){
        Serial.println("Set turn angle");
        send_string("Set turn angle");
        if(parse_cmd(cmd, rx_buffer)){
          Serial5.clear();
        }
        cmd = 0;
        cmd_index = 0;
        return;
      }
      break;

      case GET_WHEEL_SPEED:
      case GET_WHEEL_POSITION:
      case GET_ACCELERATION:
      case GET_ANGLE:
      case STOP_WHEELS:
      if(parse_cmd(cmd, rx_buffer)){
        Serial5.clear();
      }
      cmd = 0;
      cmd_index = 0;
      return;
    }
    
    cmd_index++;
  }
}

bool parse_cmd(uint8_t cmd, uint8_t* data){
  const uint8_t TX_SIZE = 1 + sizeof(double)*4;
  static uint8_t tx_buff[1 + sizeof(double)*4];
  double data_buff[4] = {0};
  float imu_data_buff[3] = {0};
  int32_t servo_angle;

  bool failed = false;

  switch(cmd){
  case SET_WHEEL_SPEED:
    for(uint8_t i = 0; i < 4; i++){
      motors[i].set_target_speed(((int32_t*)data)[i]);
      if(((int32_t*)data)[i] > 1000 || ((int32_t*)data)[i] < -1000) failed = true;
    }
    break;

  case GET_WHEEL_SPEED:
    for(uint8_t i = 0; i < 4; i++){
      data_buff[i] = motors[i].get_speed();
    }
    memcpy(tx_buff + 1, data_buff, sizeof(data_buff[0])*4);
    tx_buff[0] = RETURN_WHEEL_SPEED;
    Serial5.write(tx_buff, TX_SIZE);
    break;

  case SET_TURN_ANGLE:
    servo_angle = *(int32_t*)data;
    if(abs(servo_angle) > MAX_SERVO_ANGLE) {
      failed = true;
      break;
    }

    servo_angle = (servo_angle*90.0/MAX_SERVO_ANGLE) + 90;

    for(uint8_t i = 0; i < sizeof(servo_pins)/sizeof(servo_pins[0]); i++){
      int32_t offset_angle = servo_angle + servo_offsets[i];
      servos[i].write(reverse_servo[i] ? (180 - offset_angle) : offset_angle);
    }
    break;

  case GET_WHEEL_POSITION:
    for(uint8_t i = 0; i < 4; i++){
      data_buff[i] = motors[i].get_position();
    }
    memcpy(tx_buff + 1, data_buff, sizeof(data_buff[0])*4);
    tx_buff[0] = RETURN_WHEEL_POSITION;
    Serial5.write(tx_buff, TX_SIZE);
    break;

  case STOP_WHEELS:
    for(uint8_t i = 0; i < 4; i++){
      motors[i].set_target_speed(0);
    }
    break;

  case GET_ACCELERATION:
    imu_data_buff[0] = mpu.getEulerX();
    imu_data_buff[1] = mpu.getEulerY();
    imu_data_buff[2] = mpu.getEulerZ();
    memcpy(tx_buff + 1, imu_data_buff, sizeof(imu_data_buff));
    tx_buff[0] = RETURN_ANGLE;
    Serial5.write(tx_buff, sizeof(imu_data_buff) + 1);
    break;

  case GET_ANGLE:
    imu_data_buff[0] = mpu.getAccX();
    imu_data_buff[1] = mpu.getAccY();
    imu_data_buff[2] = mpu.getAccZ();
    memcpy(tx_buff + 1, imu_data_buff, sizeof(imu_data_buff));
    tx_buff[0] = RETURN_ACCELERATION;
    Serial5.write(tx_buff, sizeof(imu_data_buff) + 1);
    break;
  }

  return failed;
}

void ENCA(uint8_t num){
  motors[num].EncA_ISR();
}

void ENCB(uint8_t num){
  motors[num].EncB_ISR();
}