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

#include <Arduino.h>
#include "robot.h"

robot_t::robot_t()
{
  wheel_dist = 0.105;
  wheel_radius = 0.0689 / 2;
  dv_max = 5;
  dw_max = 10;
  //dv_max = 0.1f;
  //dw_max = 0.1f;
  dt = 0.04;
}

void robot_t::odometry(void)
{
  // Estimate wheels speed using the encoders
  w1e = enc1 * TWO_PI / (2.0 * 1920.0 * dt);
  w2e = enc2 * TWO_PI / (2.0 * 1920.0 * dt);

  v1e = w1e * wheel_radius;
  v2e = w2e * wheel_radius;

  // Estimate robot speed
  ve = (v1e + v2e) / 2.0;
  we = (v1e - v2e) / wheel_dist;
  
  // Estimate the distance and the turn angle
  ds = ve * dt;
  dtheta = we * dt;

  // Estimate pose
  xe += ds * cos(thetae + dtheta/2);
  ye += ds * sin(thetae + dtheta/2);
  thetae = thetae + dtheta;

  // Relative displacement
  rel_s += ds;
  rel_theta += dtheta;
}


void robot_t::setRobotVW(float Vnom, float Wnom)
{
  v_req = Vnom;
  w_req = Wnom;
}


void robot_t::accelerationLimit(void)
{
  float dv = v_req - v;
  dv = constrain(dv, -dv_max, dv_max);
  v += dv;

  float dw = w_req - w;
  dw = constrain(dw, -dw_max, dw_max);
  w += dw;
}


void robot_t::VWToMotorsVoltage(void)
{
  v1ref = v + w * wheel_dist / 2;
  v2ref = v - w * wheel_dist / 2; 
  
  w1ref = v1ref * wheel_radius;
  w2ref = v2ref * wheel_radius;

  if (control_mode == cm_pwm) {
    PWM_1 = PWM_1_req;  
    PWM_2 = PWM_2_req;  

  } else if (control_mode == cm_pid) {
    u1 = 0;
    u2 = 0;      

    if (v1ref != 0) u1 = PID1.calc(v1ref, v1e);
    else PID1.Se = 0;

    if (v2ref != 0) u2 = PID2.calc(v2ref, v2e);
    else PID2.Se = 0;

    PWM_1 = u1 / battery_voltage * 255;
    PWM_2 = u2 / battery_voltage * 255;
  }
}

void robot_t::IMURead(MPU6500 mpu, imu_t &imu)
{
  if(mpu.update())
  {
    unsigned long calib_start = millis();
    imu.dt = (calib_start - imu.cycle_time) * 1e-6; // seconds
    imu.cycle_time = calib_start;

    imu.w.x = mpu.getGyroX();
    imu.w.y = mpu.getGyroY();
    imu.w.z = mpu.getGyroZ();

    imu.a.x = mpu.getAccX();
    imu.a.y = mpu.getAccY();
    imu.a.z = mpu.getAccZ();
  }
}

void robot_t::LaserRangingSensorRead(laser_ranging_sensor_t &laser_ranging_sensor)
{
  laser_ranging_sensor.lox.rangingTest(&laser_ranging_sensor.measure, false);
  if (laser_ranging_sensor.measure.RangeStatus != 4) { // if completely out of range / bad signal
        //Serial.print("Distance: ");
        //Serial.print(laser_ranging_sensor.measure.RangeMilliMeter);
        //Serial.println(" mm");
        // možná později přidat proměnou pro ukládání posledních hodnot
    } else {
        Serial.println("Out of range");
    }

}

void robot_t::InfraredSensorsRead(infrared_sensor_t &infrared_sensors)
{
  // Read raw values from IR sensors
  //infrared_sensors.ir_raw[0] = analogRead(infrared_sensors.pins[0]);
  infrared_sensors.ir_digital[0] = !digitalRead(infrared_sensors.pins[0]);
  infrared_sensors.ir_raw[1] = analogRead(infrared_sensors.pins[1]);
  infrared_sensors.ir_raw[2] = analogRead(infrared_sensors.pins[2]);
  infrared_sensors.ir_raw[3] = analogRead(infrared_sensors.pins[3]);
  //infrared_sensors.ir_raw[4] = analogRead(infrared_sensors.pins[4]);
  infrared_sensors.ir_digital[4] = !digitalRead(infrared_sensors.pins[4]);

  // Convert analog to digital values based on threshold
  for (int i = 1; i < 4; i++) {
      infrared_sensors.ir_digital[i] = (infrared_sensors.ir_raw[i] < infrared_sensors.ir_treshold) ? 1 : 0;
  }
  
  // Print analog values on one line
  //Serial.print("IR analog: ");
  for (int i = 1; i < 4; i++) {
    Serial.print(infrared_sensors.ir_raw[i]);
    if (i < 3) Serial.print(" ");
  }
  Serial.println();

  // Print digital values on one line (indices: 0,1,2,3,5)
  //Serial.print("IR digital: ");
  for (int k = 0; k < 5; k++) {
    Serial.print(infrared_sensors.ir_digital[k]);
    if (k < 4) Serial.print(" ");
  }
  Serial.println();


  for (int i = 1; i <= 3; i++) { // IR2, IR3, IR4
      // Normalized signal: 0 = white, 1 = black
      float signal = float(infrared_sensors.ir_ref_white[i] - infrared_sensors.ir_raw[i]) /
                      float(infrared_sensors.ir_ref_white[i] - infrared_sensors.ir_ref_black[i]);
      if (signal < 0) signal = 0;
      if (signal > 1) signal = 1;

      infrared_sensors.ir_signal[i] = signal;
  }

  infrared_sensors.ir_signal[0] = infrared_sensors.ir_digital[0] ? 1.0f : 0.0f;
  infrared_sensors.ir_signal[4] = infrared_sensors.ir_digital[4] ? 1.0f : 0.0f;

  bool line_in_center = infrared_sensors.ir_signal[2] > 0.1f; // threshold can be tuned
  if (!line_in_center) {
      Serial.println("Line in center lost!");
      //return; // or handle line-lost recovery
  }

  for (int i = 1; i <= 3; i++) {
      infrared_sensors.weighted_sum += infrared_sensors.weights[i] * infrared_sensors.ir_signal[i];
      infrared_sensors.sum += infrared_sensors.ir_signal[i];
  }

  if (infrared_sensors.ir_signal[0] > 0.5f) { // left edge
      infrared_sensors.weighted_sum = -2.0f;
      infrared_sensors.sum = 1.0f;
  } else if (infrared_sensors.ir_signal[4] > 0.5f) { // right edge
      infrared_sensors.weighted_sum = 2.0f;
      infrared_sensors.sum = 1.0f;
  }

  infrared_sensors.line_position = (infrared_sensors.sum > 0.0f) ? (infrared_sensors.weighted_sum / infrared_sensors.sum) : 0.0f;

  Serial.print("Line position: ");
  Serial.println(infrared_sensors.line_position);
}



void robot_t::InfraredSensorsReference(infrared_sensor_t &infrared_sensors)
{
  constexpr int calibration_samples = 1000;
  uint32_t sum[5] = {0, 0, 0, 0, 0};

  for(int i = 0; i < calibration_samples; i++){
    for(int j = 1; j < 4; j++)
      sum[j] += analogRead(infrared_sensors.pins[j]);

    delayMicroseconds(200);
  }

  for(int k = 1; k < 4; k++)
    infrared_sensors.ir_ref_white[k] = sum[k] / calibration_samples;

  Serial.println("IR sensors calibrated. Reference values:");
  for(int m = 1; m < 4; m++){ 
    Serial.print("IR");
    Serial.print(m+1);
    Serial.print(": ");
    Serial.println(infrared_sensors.ir_ref_white[m]);
  }

}