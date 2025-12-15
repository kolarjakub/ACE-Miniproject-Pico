// 2023 Paulo Costa
// Periodic Interrupts for Quadrature Encoder reading and serial commands example
// It also has the PWM generation code to drive an H-bridge like the DRV8212PDSGR

#include <Arduino.h>
#include <RPi_Pico_TimerInterrupt.h>

// Select the timer you're using, from ITimer0(0)-ITimer3(3)
// Init RPI_PICO_Timer
RPI_PICO_Timer ITimer1(1);

#define ENC1_A 11
#define ENC1_B 10

#define ENC2_A 9
#define ENC2_B 8


#define TEST_PIN 2

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

robot_t robot;

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

  serial_commands.init(process_command);

  // Start the serial port with 115200 baudrate
  Serial.begin(115200);

  if (ITimer1.attachInterrupt(40000, timer_handler))
    Serial.println("Starting ITimer OK, millis() = " + String(millis()));
  else
    Serial.println("Can't set ITimer. Select another freq. or timer");

  interval = 40;             // In miliseconds
  robot.dt = 1e-3 * interval; // In seconds
  robot.PID1.dt = robot.dt;
  robot.PID2.dt = robot.dt;
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
      robot.battery_voltage = 7.4; // it really shoud be measured...

      // Control the robot here by choosing:
      //   v_req and w_req          when robot.control_mode = cm_pid
      //   PWM_1_req and PWM_1_req  when robot.control_mode = cm_pwm
      // ...

      // Calc outputs
      robot.setRobotVW(robot.v_req, robot.w_req);
      //robot.accelerationLimit();
      robot.v = robot.v_req;
      robot.w = robot.w_req;
      robot.VWToMotorsVoltage();

      setMotorPWM(robot.PWM_1, MOTOR1A_PIN, MOTOR1B_PIN);
      setMotorPWM(robot.PWM_2, MOTOR2A_PIN, MOTOR2B_PIN);

      // Debug information
      Serial.print(" M1: ");
      Serial.print(robot.PWM_1);

      Serial.print(" M2: ");
      Serial.print(robot.PWM_2);
            
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

