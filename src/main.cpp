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

#define MOTOR1A_PIN 5
#define MOTOR1B_PIN 4

#define MOTOR2A_PIN 6
#define MOTOR2B_PIN 7


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

#define VELOCITY_BASE -0.05f


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
PID_t lineFollowerPID;
float angular_correction;


void setMotorPWM(int new_PWM, int pin_a, int pin_b)
{
  int PWM_max = 50;
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

// Rotate robot to the nearest pi/4 angle relative to current yaw
void rotateToNearestPiOver4(float relative_angle_rad)
{
    // Calculate target absolute angle
    float target_angle = imu.yaw + relative_angle_rad;

    // Round to nearest multiple of pi/4
    float nearest_pi4 = round(target_angle / (PI/4.0f)) * (PI/4.0f);

    imu.yaw_target = nearest_pi4;
}

// Remote commands
unsigned long interval, last_cycle;
unsigned long loop_micros;

#include "commands.h"

commands_t serial_commands;
unsigned int cycle_count = 0;

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
  robot.dv_max = 5.0f; // m/s every cycle
  robot.dw_max = 5.0f; // rad/s every cycle


  // Configure line-following PID (PD mode)
  lineFollowerPID.Kp = 1.1f;  // proportional gain
  lineFollowerPID.Ki = 0.0f;  // disable integral
  lineFollowerPID.Kd = 0.10f;  // derivative gain
  lineFollowerPID.Kf = 0.0f;  // no feedforward
  lineFollowerPID.dt = robot.dt;
  
  // Set default control mode to PID so VW commands produce motor outputs
  robot.control_mode = cm_pid;

  // Clear PID integrators
  robot.PID1.reset();
  robot.PID2.reset();
  lineFollowerPID.reset();


  robot.battery_voltage = 7.4; // it really shoud be measured...

  robot.setRobotVW(0.0, 0.0);
}
void loop() 
{
    uint8_t b;
    if (Serial.available()) {  // Only do this if there is serial data to be read
        b = Serial.read();    
        serial_commands.process_char(b);
    }  

    unsigned long now = millis();
    if (now - last_cycle > interval) {
        loop_micros = micros();
        last_cycle += interval;

        // ------------------- SENSOR UPDATES -------------------
        read_encoders();
        robot.enc1 = enc1;
        robot.enc2 = enc2;
        robot.odometry();

        robot.setRobotVW(robot.v_req, robot.w_req);
        robot.accelerationLimit();

        robot.IMURead(mpu, imu);
        robot.InfraredSensorsRead(infrared_sensors);
        robot.LaserRangingSensorRead(laser_ranging_sensor);

        // ================= FSM handling ===================== //
        switch (FSM.currentState)
        {
            case State::IDLE:
                FSM.newState = State::CALIBRATION_IMU;
                break;

            case State::CALIBRATION_IMU:
                mpu.calibrateAccelGyro();

                robot.PID1.reset();
                robot.PID2.reset();
                lineFollowerPID.reset();

                imu.yaw = 0.0f;        
                imu.yaw_ref = 0.0f;    
                imu.yaw_target = 0.0f; 

                FSM.newState = State::LINE_FOLLOW;
                break;

            case State::LINE_FOLLOW:
            {
                if(infrared_sensors.turn_left || infrared_sensors.turn_right)
                {
                    robot.turn_direction = infrared_sensors.turn_left ? 1 : -1;
                    robot.rel_s = 0.0f;
                    robot.turn_distance_remaining = -0.07f; // 7 cm forward before rotation

                    // Clear detection flags
                    infrared_sensors.turn_left = 0;
                    infrared_sensors.turn_right = 0;

                    FSM.newState = State::PRE_TURN_FORWARD;
                }
                else
                {
                    // Normal line following
                    angular_correction = lineFollowerPID.calc(0.0f, infrared_sensors.line_position);
                    robot.setRobotVW(VELOCITY_BASE, angular_correction);
                    robot.accelerationLimit();
                }
                break;
            }

            case State::PRE_TURN_FORWARD:
            {
                // Move forward until relative distance reaches target
                if(robot.rel_s > robot.turn_distance_remaining)
                {
                    robot.setRobotVW(-0.05f, 0.0f);
                    robot.accelerationLimit();
                }
                else
                {
                    // Distance complete, start rotation
                    FSM.newState = State::ROTATE;
                }
                break;
            }

            case State::ROTATE:
            {
                // Rotate with constant speed until line_detected is true
                float rotation_speed = 0.8f; // rad/s
                float w_rotate = robot.turn_direction * rotation_speed;

                if (infrared_sensors.line_detected && (fabsf(infrared_sensors.line_position) < 0.30f)) {
                    w_rotate = 0.0f;
                    robot.PID1.reset();
                    robot.PID2.reset();
                    lineFollowerPID.reset();
                    robot.turn_direction = 0;
                    FSM.newState = State::LINE_FOLLOW;
                }

                robot.setRobotVW(0.0f, w_rotate);
                robot.accelerationLimit();

                Serial.print(" ROTATE: w_rotate: ");
                Serial.print(w_rotate);
                Serial.print(" line_detected: ");
                Serial.println(infrared_sensors.line_detected);
                break;
            }
        }

        // Send commands to motors
        robot.v = robot.v_req;
        robot.w = robot.w_req;
        robot.VWToMotorsVoltage();
        setMotorPWM(robot.PWM_1, MOTOR1A_PIN, MOTOR1B_PIN);
        setMotorPWM(robot.PWM_2, MOTOR2A_PIN, MOTOR2B_PIN);

        FSM.updateTisTes();
        FSM.setState(FSM.newState);

        Serial.print("LP: ");
        Serial.print(infrared_sensors.line_position);

        Serial.print(" | L_seen: ");
        Serial.print(infrared_sensors.intersection_left_seen);

        Serial.print(" R_seen: ");
        Serial.print(infrared_sensors.intersection_right_seen);

        Serial.print(" | TL: ");
        Serial.print(infrared_sensors.turn_left);

        Serial.print(" TR: ");
        Serial.print(infrared_sensors.turn_right);

        Serial.print(" | ALL: ");
        Serial.println(infrared_sensors.all_sensors_on_line);


        // Debug prints every 25 cycles
        cycle_count++;
        if(cycle_count >= 25) {
            cycle_count = 0;

            Serial.print("FSM state: ");
            Serial.println(FSM.getStateName());
            Serial.print("Line pos: ");
            Serial.println(infrared_sensors.line_position);
            Serial.print("Angular correction: ");
            Serial.println(angular_correction);

            Serial.print("PWM M1: ");
            Serial.print(robot.PWM_1);
            Serial.print(" M2: ");
            Serial.println(robot.PWM_2);

            Serial.print("IMU Gyro Z: ");
            Serial.println(imu.w.z);
            Serial.print("Line detected: ");
            Serial.println(infrared_sensors.line_detected);

            Serial.print("values of lines sensors: ");
            Serial.print(infrared_sensors.ir_signal[0]);
            Serial.print(" ");
            Serial.print(infrared_sensors.ir_signal[1]);
            Serial.print(" ");
            Serial.print(infrared_sensors.ir_signal[2]);
            Serial.print(" ");
            Serial.print(infrared_sensors.ir_signal[3]);
            Serial.print(" ");
            Serial.print(infrared_sensors.ir_signal[4]);

            Serial.print("digital values of lines sensors: ");
            Serial.print(infrared_sensors.ir_digital[0]);
            Serial.print(" ");
            Serial.print(infrared_sensors.ir_digital[1]);
            Serial.print(" ");
            Serial.print(infrared_sensors.ir_digital[2]);
            Serial.print(" ");
            Serial.print(infrared_sensors.ir_digital[3]);
            Serial.print(" ");
            Serial.print(infrared_sensors.ir_digital[4]);
 
        }
    }
}


