/* Copyright (c) 2021  Paulo Costa
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:

   * Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
   * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in
     the documentation and/or other materials provided with the
     distribution.
   * Neither the name of the copyright holders nor the names of
     contributors may be used to endorse or promote products derived
     from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE. */

#ifndef ROBOT_H
#define ROBOT_H

#include <Arduino.h>
#include <math.h>
#include "PID.h"
#include <Adafruit_VL53L0X.h>
//#include <MPU6050.h>¨
#include <VectorXf.h>
#include "MPU6500_Raw.h"
#include <MadgwickAHRS.h>


typedef enum { 
  cm_pwm,
  cm_pid
} control_mode_t;

typedef struct{
  Adafruit_VL53L0X lox;
  VL53L0X_RangingMeasurementData_t measure;
  bool outOfRange;
  uint16_t distance;
}laser_ranging_sensor_t;

typedef struct{
  Vec3f w;
  Vec3f a;
  uint32_t cycle_time, last_cycle_time;
  uint32_t dt; // IMU cycle tracking
  float roll, pitch, yaw;  // Euler angles from Madgwick filter
}imu_t;

typedef struct{
  uint16_t ir_raw[5]; // IR sensorů
  uint8_t ir_digital[5]; // Digitální hodnoty IR senzorů (přes prahovou hodnotu)
  const int ir_treshold = 500; // Záleží na odstínu čáry, NASTAVIT !!!
  uint16_t ir_ref_black[5]; // Referenční hodnoty pro kalibraci
  uint16_t ir_ref_white[5]; // Referenční hodnoty pro kalibraci
  int weights[5] = { 0, -1, 0, 1, 0}; // Váhy pro výpočet polohy čáry
  uint8_t pins[5];
  float ir_signal[5];
  float weighted_sum;
  float sum;
  float line_position;
  bool line_detected;
} infrared_sensor_t;

class robot_t {
  public:
  int enc1, enc2;
  int Senc1, Senc2;
  float w1e, w2e;
  float v1e, v2e;
  float ve, we;
  float ds, dtheta;
  float rel_s, rel_theta;
  float xe, ye, thetae;
  
  float dt;
  float v, w;
  float v_req, w_req;
  float dv_max, dw_max;
  
  float wheel_radius, wheel_dist;
  
  float v1ref, v2ref;
  float w1ref, w2ref;
  float u1, u2;
  int PWM_1, PWM_2;
  int PWM_1_req, PWM_2_req;
  control_mode_t control_mode;
  
  PID_t PID1, PID2;
  float battery_voltage;
  
  robot_t();

  void odometry(void);
  void setRobotVW(float Vnom, float Wnom);

  void IMURead(MPU6500 mpu, imu_t &imu);

  void LaserRangingSensorRead(laser_ranging_sensor_t &laser_ranging_sensor);

  void InfraredSensorsRead(infrared_sensor_t &infrared_sensors);
  void InfraredSensorsReference(infrared_sensor_t &infrared_sensors);

  void accelerationLimit(void);
  void VWToMotorsVoltage(void);
};


#endif // ROBOT_H
