// Rover Motor Drive
// Code by Paulo Costa, May 2014
// This is not the original driver, this it is based on TimerOne libray
// It has suport for 3 motors with quadrature encoders
// and serial comunication for remote control
#include <Arduino.h>
#include <TimerOne.h> //http://www.pjrc.com/teensy/td_libs_TimerOne.html
#include <TimerThree.h> //http://www.pjrc.com/teensy/td_libs_TimerOne.html
#include <TimerFour.h> //http://www.pjrc.com/teensy/td_libs_TimerOne.html
//#include <digitalWriteFast.h> //http://code.google.com/p/digitalwritefast/

// Metal Gearmotor 12VDC with 64 CPR Encoder 350rpm
// Color  - Function

// Black  - motor power
// Red    - motor power
// Blue   - Hall sensor Vcc (3.5 ? 20 V)
// Green  - Hall sensor GND
// Yellow - Hall sensor A output
// White  - Hall sensor B output

// constants won't change. Used here to 
// set pin numbers:
const int ledPin =  13;      // the number of the LED pin
const int BUF_SIZE = 64;
//const int debugPin = 11;

//const int nD2_pin = 4;     // Tri-state disables both outputs of both motor channels when LOW;
//                           // toggling resets latched driver fault condition
const int M1_DIR_pin = 4;  // Motor 1 dir input
const int M1_PWM_pin = 5;  // Motor 1 PWM input
const int M1_CUR_pin = A15; // Motor 1 current sense output (approx. 1000 mV/A)

const int M2_DIR_pin = 6;  // Motor 2 dir input
const int M2_PWM_pin = 2;  // Motor 2 PWM input
const int M2_CUR_pin = A14; // Motor 2 current sense output (approx. 1000 mV/A)

const int M3_DIR_pin = 7;  // Motor 3 dir input
const int M3_PWM_pin = 3;  // Motor 3 PWM input
const int M3_CUR_pin = A13; // Motor 3 current sense output (approx. 1000 mV/A)


const int M1_encoderA_pin = 22;  // Motor 1 encoder A input
const int M1_encoderB_pin = 23;  // Motor 1 encoder B input

const int M2_encoderA_pin = 24;  // Motor 2 encoder A input
const int M2_encoderB_pin = 25;  // Motor 2 encoder B input

const int M3_encoderA_pin = 26;  // Motor 3 encoder A input
const int M3_encoderB_pin = 27;  // Motor 3 encoder B input


// Variables
byte ledState = LOW;            // ledState used to set the LED
long previousMillis = 0;        // will store last time LED was updated

long interval = 40;             // Sensor update period (ms)
byte motors_timeout, motors_timeout_count;

int M1_PWM_value = 0;
int M2_PWM_value = 0;
int M3_PWM_value = 0;

int M1_voltage = 0, M1_old_voltage = 0;
int M2_voltage = 0, M2_old_voltage = 0;
int M3_voltage = 0, M3_old_voltage = 0;


byte encoder1_state, encoder2_state, encoder3_state;
volatile uint16_t encoder1_pos, encoder2_pos, encoder3_pos;
/*
uint16_t prepareOutFrame(char channel)
{
  if (channel == 'i') {         
    return (1);  
  } else if (channel == 'q')  {
    return 2;  
  }
}
*/

// g
// h
// i - M1 Current
// j - M2 Current
// k - M3 Current
// l
// m - M1 Voltage (PWM: 0 - 1024)
// n - M2 Voltage (PWM: 0 - 1024)
// o - M3 Voltage (PWM: 0 - 1024)
// p - MetaPacket delimiter
// q - 
// r - M1 Odometry
// s - M2 Odometry
// t - M3 Odometry
// u
// v
// w
// x
// y
// z


void sendOut(char channel)
{
  if (channel == 'i') {         // i: sends A0 and A1 as j
    sendCurrent(); 
  } else if (channel == 'o')  { 
    sendOdometry(); 
  }
}

void sendPWM(void)
{
  sendChannel('m', M1_PWM_value); 
  sendChannel('n', M2_PWM_value); 
  sendChannel('o', M3_PWM_value); 
}


void sendCurrent(void)
{
  sendChannel('i', analogRead(M1_CUR_pin)); 
  sendChannel('j', analogRead(M2_CUR_pin)); 
  sendChannel('k', analogRead(M3_CUR_pin)); 
}


void sendOdometry(void)
{
  uint16_t o1, o2, o3;
  // Read odometry atomicaly
  cli();
  o1 = encoder1_pos;
  o2 = encoder2_pos;
  o3 = encoder3_pos;
  sei();
  // Send it
  sendChannel('r', o1); 
  sendChannel('s', o2); 
  sendChannel('t', o3); 
}

// G - S1 ON Pulse (us)
// H - S2 ON Pulse (us)
// I
// J
// K
// L
// M - M1 Voltage (PWM: 0 - 1024)
// N - M2 Voltage (PWM: 0 - 1024)
// O - M3 Voltage (PWM: 0 - 1024)
// P - Sensor update period (ms)
// Q - MetaPacket delimiter
// R - M1 Voltage (PWM: 0 - 1024) SYNC
// S - M2 Voltage (PWM: 0 - 1024) SYNC
// T - M3 Voltage (PWM: 0 - 1024) SYNC
// U
// V
// W
// X
// Y
// Z

void processInFrame(unsigned char channel, uint16_t value)
{
  byte i;
  if (channel == 'M' || channel == 'R') {
    M1_voltage = (value & 0x0FFF);
    if ((value & 0xF000) > 0)
      M1_voltage = - M1_voltage;
    if (channel == 'M')  
      set_M1_voltage(M1_voltage);
    motors_timeout_count = 0;

  } else if (channel == 'N' || channel == 'S')  {
    M2_voltage = (value & 0x0FFF);
    if ((value & 0xF000) > 0)
      M2_voltage = - M2_voltage;
    if (channel == 'N')  
      set_M2_voltage(M2_voltage);
    motors_timeout_count = 0;

  } else if (channel == 'O' || channel == 'T')  {
    M3_voltage = (value & 0x0FFF);
    if ((value & 0xF000) > 0)
      M3_voltage = - M3_voltage;
    if (channel == 'O')  
      set_M3_voltage(M3_voltage);
    motors_timeout_count = 0;

  } else if (channel == 'P')  {
    interval = value;

  } else if (channel == 'G')  {
    OCR1A = value; // Set Timer1 PWM duty cycle directly (ON time: value in us)

  } else if (channel == 'H')  {
    OCR1B = value; // Set Timer1 PWM duty cycle directly (ON time: value in us)
  }
}

static void set_M1_voltage(int new_voltage)
{
  M1_old_voltage = new_voltage;
  if (new_voltage >= 0) {
    M1_PWM_value = new_voltage;
    digitalWrite(M1_DIR_pin, 0);
  } else {
    M1_PWM_value = -new_voltage;
    digitalWrite(M1_DIR_pin, 1);
  }
  Timer3.setPwmDuty(TIMER3_A_PIN, M1_PWM_value);
}


static void set_M2_voltage(int new_voltage)
{
  M2_old_voltage = new_voltage;
  if (new_voltage >= 0) {
    M2_PWM_value = new_voltage;
    digitalWrite(M2_DIR_pin, 0);
  } else {
    M2_PWM_value = -new_voltage;
    digitalWrite(M2_DIR_pin, 1);
  }
  Timer3.setPwmDuty(TIMER3_B_PIN, M2_PWM_value);
}


static void set_M3_voltage(int new_voltage)
{
  M3_old_voltage = new_voltage;
  if (new_voltage >= 0) {
    M3_PWM_value = new_voltage;
    digitalWrite(M3_DIR_pin, 0);
  } else {
    M3_PWM_value = -new_voltage;
    digitalWrite(M3_DIR_pin, 1);
  }
  Timer3.setPwmDuty(TIMER3_C_PIN, M3_PWM_value);
}


void timer_interrupt(void)
{
  byte b, new_state;
  static int8_t encoder_table[16] = {0, 1, -1, 0,  -1, 0, 0, 1,  1, 0, 0, -1,  0, -1, 1, 0};
  
  //digitalWriteFast(debugPin, 1);
  b = PINA;

  new_state = (b >> 0) & 0x03; // Put encoder channels in the lowest bits
  encoder1_pos += encoder_table[encoder1_state | new_state];
  encoder1_state = new_state << 2;

  new_state = (b >> 2) & 0x03; // Put encoder channels in the lowest bits
  encoder2_pos += encoder_table[encoder2_state | new_state];
  encoder2_state = new_state << 2;

  new_state = (b >> 4) & 0x03; // Again, Put encoder channels in the lowest bits 
  encoder3_pos += encoder_table[encoder3_state | new_state];
  encoder3_state = new_state << 2;

  //digitalWriteFast(debugPin, 0);
}




void setup()
{
  byte i;
  // set the digital pin as output:
  pinMode(ledPin, OUTPUT);      
  //pinMode(debugPin, OUTPUT);      
 
  pinMode(M1_DIR_pin, OUTPUT);      
  pinMode(M1_PWM_pin, OUTPUT);      

  pinMode(M2_DIR_pin, OUTPUT);      
  pinMode(M2_PWM_pin, OUTPUT);      

  pinMode(M3_DIR_pin, OUTPUT);      
  pinMode(M3_PWM_pin, OUTPUT);      

  pinMode(M1_encoderA_pin, INPUT_PULLUP);
  pinMode(M1_encoderB_pin, INPUT_PULLUP);

  pinMode(M2_encoderA_pin, INPUT_PULLUP);
  pinMode(M2_encoderB_pin, INPUT_PULLUP);

  pinMode(M3_encoderA_pin, INPUT_PULLUP);
  pinMode(M3_encoderB_pin, INPUT_PULLUP);
  
  motors_timeout = 100; 
  motors_timeout_count = 0;  
 
  //Timer3.attachInterrupt(timer_interrupt); 
  Timer3.initialize(200); //uS
  Timer3.pwm(TIMER3_A_PIN, M1_PWM_value); 
  Timer3.pwm(TIMER3_B_PIN, M2_PWM_value); 
  Timer3.pwm(TIMER3_C_PIN, M3_PWM_value); 

  Timer4.attachInterrupt(timer_interrupt); 
  Timer4.initialize(50); //uS

  Timer1.initialize(20000); //uS
  Timer1.pwm(TIMER1_A_PIN, 50); 
  Timer1.pwm(TIMER1_B_PIN, 60); 

  initChannelsStateMachine();
  Serial.begin(115200);
}


void loop()
{
  
  if (Serial.available() > 0) {
    byte serialByte = Serial.read();
    channelsStateMachine(serialByte);
    //Serial.write(serialByte);
  } 
  
  unsigned long currentMillis = micros();
 
  if(currentMillis - previousMillis > interval * 1000) {
    // save the last time you blinked the LED 
    previousMillis = currentMillis;   

    // if the LED is off turn it on and vice-versa:
    if (ledState == LOW) {
      ledState = HIGH;
    } else {
      ledState = LOW;
    }
    // set the LED with the ledState of the variable:
    digitalWrite(ledPin, ledState);

    if (motors_timeout > 0){
      if (motors_timeout_count < motors_timeout){
        motors_timeout_count++;
        if (M1_voltage != M1_old_voltage) {
          set_M1_voltage(M1_voltage);
        }
        if (M2_voltage != M2_old_voltage) {
          set_M2_voltage(M2_voltage);
        }
        if (M3_voltage != M3_old_voltage) {
          set_M3_voltage(M3_voltage);
        }
      } else {
        set_M1_voltage(0);
        set_M2_voltage(0);
        set_M3_voltage(0);
      }
    }
    
    sendChannel('p', 9); // Start MetaPacket
    sendPWM();
    sendOdometry();
    sendCurrent();
    sendChannel('p', 0); // End MetaPacket
    Serial.println();
    
/*  if (motors_timeout > 0){
      if (motors_timeout_count >= motors_timeout){
        set_M1_voltage(0);
        set_M2_voltage(0);
        set_M3_voltage(0);
      } else {
        motors_timeout_count++;
      }
    }
  */
  }
}

