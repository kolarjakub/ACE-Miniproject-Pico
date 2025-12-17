// 2023 Paulo Costa
// Periodic Interrupts for Quadrature Encoder reading and serial commands example
// It also has the PWM generation code to drive an H-bridge like the DRV8212PDSGR

#include <Arduino.h>
#include <RPi_Pico_TimerInterrupt.h>
#include <Wire.h>
#include "PID.h"

// Select the timer you're using, from ITimer0(0)-ITimer3(3)
// Init RPI_PICO_Timer
RPI_PICO_Timer ITimer1(1);

#define ENC1_A 11
#define ENC1_B 10

#define ENC2_A 9
#define ENC2_B 8

#define IR1_pin 22
#define IR2_pin A0  //GPIO26
#define IR3_pin A1  //GPIO27
#define IR4_pin A2  //GPIO28
#define IR5_pin 18

// Laser ranging sensor I2C0 pins:
//#define LASER_RANGING_SCL_pin 15
//#define LASER_RANGING_SDA_pin 14

// IMU I2C pins: I2C0
#define SDA_pin 20
#define SCL_pin 21

#define TEST_PIN 2

#define VELOCITY_BASE 0.05  // m/s


volatile int encoder1_pos = 0;
volatile int encoder2_pos = 0;

int enc1, enc2;

int encoder1_state, encoder2_state;
// This table implements the counting event for every transition of the encoder state machine
int encoder_table[16] = {0, 1, -1, 0, -1, 0, 0, 1, 1, 0, 0, -1, 0, -1, 1, 0}; 

volatile int count;
int act_count;

#define digitalWriteFast(pin, val)  (val ? sio_hw->gpio_set = (1 << pin) : sio_hw->gpio_clr = (1 << pin))
#define digitalReadFast(pin)        (((1 << pin) & sio_hw->gpio_in) >> pin)

#define pinIsHigh(pin, pins)        (((1 << pin) & pins) >> pin)


bool timer_handler(struct repeating_timer *t)
{
  int next_state, table_input, pins;
  digitalWriteFast(TEST_PIN, 1);

  pins = sio_hw->gpio_in;  // Just one read to get all pins

  next_state = pinIsHigh(ENC1_A, pins) << 1;
  next_state |= pinIsHigh(ENC1_B, pins);

  table_input = (encoder1_state << 2) | next_state;
  encoder1_pos += encoder_table[table_input];
  encoder1_state = next_state;

  next_state = pinIsHigh(ENC2_A, pins) << 1;
  next_state |= pinIsHigh(ENC2_B, pins);
  
  table_input = (encoder2_state << 2) | next_state;
  encoder2_pos -= encoder_table[table_input];
  encoder2_state = next_state;

  count++;
  digitalWriteFast(TEST_PIN, 0);
  return true;
}

/*
// Less optimized version
bool timer_handler(struct repeating_timer *t)
{
  int next_state, table_input;
  digitalWriteFast(TEST_PIN, 1);

  next_state = digitalReadFast(ENC1_A) << 1;
  next_state |= digitalReadFast(ENC1_B);

  table_input = (encoder1_state << 2) | next_state;
  encoder1_pos += encoder_table[table_input];
  encoder1_state = next_state;

  next_state = digitalReadFast(ENC2_A) << 1;
  next_state |= digitalReadFast(ENC2_B);

  table_input = (encoder2_state << 2) | next_state;
  encoder2_pos -= encoder_table[table_input];
  encoder2_state = next_state;

  count++;
  digitalWriteFast(TEST_PIN, 0);
  return true;
}
*/

void read_encoders(void)
{
  noInterrupts();  // Must be done with the interrupts disabled
  
  enc1 = encoder1_pos;
  enc2 = encoder2_pos;
  act_count = count;

  encoder1_pos = 0;
  encoder2_pos = 0;
  count = 0;

  interrupts();
}

// PWM stuff

#define MOTOR1A_PIN 7
#define MOTOR1B_PIN 6

#define MOTOR2A_PIN 5
#define MOTOR2B_PIN 4



void setMotorPWM(int new_PWM, int pin_a, int pin_b)
{
  int PWM_max = 200;
  if (new_PWM >  PWM_max) new_PWM =  PWM_max;
  if (new_PWM < -PWM_max) new_PWM = -PWM_max;
  
  if (new_PWM == 0) {  // Both outputs 0 -> A = H, B = H
    analogWrite(pin_a, 255);
    analogWrite(pin_b, 255);
  } else if (new_PWM > 0) {
    analogWrite(pin_a, 255 - new_PWM);
    analogWrite(pin_b, 255);
  } else {
    analogWrite(pin_a, 255);
    analogWrite(pin_b, 255 + new_PWM);
  }
}


#include "robot.h"
#include "fsm.h"

MPU6500 mpu;
imu_t imu;
laser_ranging_sensor_t laser_ranging_sensor;
infrared_sensor_t infrared_sensors = {
    .ir_ref_black = {0, 157, 154, 145, 0}, // black reference values
    .ir_ref_white = {0, 970, 969, 968, 0}, // white reference values
    .pins = {IR1_pin, IR2_pin, IR3_pin, IR4_pin, IR5_pin}
};
robot_t robot;
fsm FSM;

// Remote commands

unsigned long interval, last_cycle;
unsigned long loop_micros;

#include "commands.h"

commands_t serial_commands;

void process_command(frame_data_t frame)
{
  if (frame.command_is("m1")) {  // The 'm1' command sets the PWM value for motor 1
    robot.PWM_1_req = frame.value;

  } else if (frame.command_is("m2")) {  // The 'm2' command sets the PWM value for motor 1
    robot.PWM_2_req = frame.value;

  } else if (frame.command_is("mo")) { // The 'mo'de command ...
    robot.control_mode = (control_mode_t) frame.value;

  } else if (frame.command_is("v")) { 
    robot.v_req = frame.value;    

  } else if (frame.command_is("w")) { 
    robot.w_req = frame.value;    

  } else if (frame.command_is("kf")) { 
    robot.PID1.Kf = frame.value;    
    robot.PID2.Kf = frame.value;    

  } else if (frame.command_is("kp")) { 
    robot.PID1.Kp = frame.value;    
    robot.PID2.Kp = frame.value;    

  } else if (frame.command_is("ki")) { 
    robot.PID1.Ki = frame.value;    
    robot.PID2.Ki = frame.value;
 
  } // Put here more commands...
}


void setup() 
{
  Serial.begin(115200);
  int start = millis();
  while (millis() - start < 4000) {
    // wait for serial port to connect. Needed for native USB
  }

  Serial.println("FSM Init: Starting setup...");
  // Set the pins as input or output as needed
  pinMode(ENC1_A, INPUT_PULLUP);
  pinMode(ENC1_B, INPUT_PULLUP);
  pinMode(ENC2_A, INPUT_PULLUP);
  pinMode(ENC2_B, INPUT_PULLUP);

  pinMode(TEST_PIN, OUTPUT);

  pinMode(MOTOR1A_PIN, OUTPUT);
  pinMode(MOTOR1B_PIN, OUTPUT);
 
  pinMode(MOTOR2A_PIN, OUTPUT);
  pinMode(MOTOR2B_PIN, OUTPUT);

  delay(100);
  Serial.println("FSM Init: Starting GPIO setup...");
  //Serial.flush();
  // Initialize peripherals pins
  // Line sensors
  pinMode(IR1_pin, INPUT);
  pinMode(IR2_pin, INPUT);
  pinMode(IR3_pin, INPUT);
  pinMode(IR4_pin, INPUT);
  pinMode(IR5_pin, INPUT);

  Serial.println("FSM Init: Motor PWM pins configured");
  // IMU MPU6500
  Serial.println("FSM Init: Starting IMU initialization...");
  pinMode(SDA_pin, INPUT_PULLUP);
  pinMode(SCL_pin, INPUT_PULLUP);
  Wire.setSDA(SDA_pin);
  Wire.setSCL(SCL_pin);
  Wire.begin();

  Serial.println("Scanning I2C bus...");
  for (byte i = 1; i < 127; i++) {
    Wire.beginTransmission(i);
    if (Wire.endTransmission() == 0) {
      Serial.print("Found I2C device at 0x");
      Serial.println(i, HEX);
    }
  }

  MPU6500Setting setting;
  setting.accel_fs_sel = ACCEL_FS_SEL::A16G;
  setting.gyro_fs_sel = GYRO_FS_SEL::G2000DPS;
  setting.fifo_sample_rate = FIFO_SAMPLE_RATE::SMPL_200HZ;
  setting.gyro_fchoice = 0x03;
  setting.gyro_dlpf_cfg = GYRO_DLPF_CFG::DLPF_41HZ;
  setting.accel_fchoice = 0x01;
  setting.accel_dlpf_cfg = ACCEL_DLPF_CFG::DLPF_45HZ;
  
  if(!mpu.setup(0x68, setting)) {
      Serial.println("MPU connection failed - continuing anyway");
  } else {
      Serial.println("MPU initialized successfully");
  }

    // Laser Ranging Sensor
  Serial.println("FSM Init: Starting VL53L0X initialization...");

  // Initialize the laser ranging sensor
  if (!laser_ranging_sensor.lox.begin()) {
      Serial.println("Failed to initialize VL53L0X - continuing anyway");
  } else {
       Serial.println("VL53L0X initialized successfully");
       laser_ranging_sensor.lox.setMeasurementTimingBudgetMicroSeconds(50000);
  }

  serial_commands.init(process_command);

  if (ITimer1.attachInterrupt(40000, timer_handler))
    Serial.println("Starting ITimer OK, millis() = " + String(millis()));
  else
    Serial.println("Can't set ITimer. Select another freq. or timer");

  interval = 40;             // In miliseconds
  robot.dt = 1e-3 * interval; // In seconds
  robot.PID1.dt = robot.dt;
  robot.PID2.dt = robot.dt;
  // Set default control mode to PID so VW commands produce motor outputs
  robot.control_mode = cm_pid;
  // Clear PID integrators
  robot.PID1.Se = 0;
  robot.PID2.Se = 0;
  robot.PID1.e = 0;
  robot.PID2.e = 0;

  robot.battery_voltage = 7.4; // it really shoud be measured...

  robot.v_req=VELOCITY_BASE;
  //robot.w_req=infrared_sensors.line_position*-0.0001f;
  robot.w_req=0.0;

}

void loop() 
{
  uint8_t b;
  if (Serial.available()) {  // Only do this if there is serial data to be read
    b = Serial.read();    
    serial_commands.process_char(b);
  }  

  // To measure the time between loop() calls
  //unsigned long last_loop_micros = loop_micros; 
  
  // Do this only every "interval" miliseconds 
  // It helps to clear the switches bounce effect
  unsigned long now = millis();
  if (now - last_cycle > interval) {
    loop_micros = micros();
    //last_cycle = now;
    last_cycle += interval;

    // Read and process sensors
    read_encoders();
    robot.enc1 = enc1;
    robot.enc2 = enc2;
    robot.odometry();

    // Calc outputs
    robot.setRobotVW(robot.v_req, robot.w_req);
    robot.accelerationLimit();

    robot.v = robot.v_req;
    robot.w = robot.w_req;
    robot.VWToMotorsVoltage();

    setMotorPWM(robot.PWM_1, MOTOR1A_PIN, MOTOR1B_PIN);
    setMotorPWM(robot.PWM_2, MOTOR2A_PIN, MOTOR2B_PIN);


    robot.IMURead(mpu, imu);
    robot.InfraredSensorsRead(infrared_sensors);
    robot.LaserRangingSensorRead(laser_ranging_sensor);



    // Control the robot here by choosing:
    //   v_req and w_req          when robot.control_mode = cm_pid
    //   PWM_1_req and PWM_1_req  when robot.control_mode = cm_pwm
    // ...


    // ================= FSM handling ===================== //
    
    switch (FSM.currentState)
    {
      case State::IDLE:
        FSM.newState = State::CALIBRATION_IMU;
        //robot.InfraredSensorsReference(infrared_sensors);
        break;

      case State::CALIBRATION_IMU:
        //mpu.calibrateAccelGyro();
        FSM.newState = State::LINE_FOLLOW;
        break;

      case State::LINE_FOLLOW:
        break;

      case State::ROTATE:
        break;

    }
    
    // ================= End of FSM handling ===================== //

    FSM.updateTisTes();
    FSM.setState(FSM.newState);




    // Debug information
    Serial.print(" currentState: ");
    Serial.println(FSM.getStateName());

    
    Serial.print(" M1: ");
    Serial.print(robot.PWM_1);
    Serial.print(" M2: ");
    Serial.print(robot.PWM_2);

    Serial.print(" IMU_gyroscope X: ");
    Serial.print(imu.w.x);
    Serial.print(" Y: ");
    Serial.print(imu.w.y);
    Serial.print(" Z: ");
    Serial.print(imu.w.z);
    Serial.print(" IMU_accelerrometer X: ");
    Serial.print(imu.a.x);
    Serial.print(" Y: ");
    Serial.print(imu.a.y);
    Serial.print(" Z: ");
    Serial.print(imu.a.z);
          
    // Serial.print(" cnt: ");
    // Serial.print(act_count);
    // Serial.print(" e1: ");
    // Serial.print(enc1);
    // Serial.print(" e2: ");
    // Serial.print(enc2);
    Serial.print(" v1e: ");
    Serial.print(robot.v1e);
    Serial.print(" v2e: ");
    Serial.print(robot.v2e);
    Serial.print(" v1ref: ");
    Serial.print(robot.v1ref);
    Serial.print(" v2ref: ");
    Serial.print(robot.v2ref);
    Serial.print(" v_req: ");
    Serial.print(robot.v_req);
    
    Serial.print(" mode: ");
    Serial.print(robot.control_mode);
    Serial.print(" cmd: ");
    Serial.print(serial_commands.frame.command);
    Serial.print(" loop: ");
    Serial.println(micros() - loop_micros);

  }
    
}

