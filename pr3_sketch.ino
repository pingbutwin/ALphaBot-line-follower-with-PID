#include "TRSensors.h"

#define PIN_LEFT_MOTOR_SPEED 5
#define PIN_LEFT_MOTOR_FORWARD A0            
#define PIN_LEFT_MOTOR_REVERSE A1
#define PIN_LEFT_ENCODER 2

#define INTERRUPT_PIN 1
   
#define PIN_RIGHT_MOTOR_SPEED 6
#define PIN_RIGHT_MOTOR_FORWARD A2            
#define PIN_RIGHT_MOTOR_REVERSE A3
#define PIN_RIGHT_ENCODER 3

// #define TRIGER_PIN 11
// #define ECHO_PIN 12
#define ANALOG_READ_IR_LEFT A4
#define ANALOG_READ_IR_RIGHT A5

#define NUM_SENSORS 5
TRSensors trs =   TRSensors();
unsigned int sensorValues[NUM_SENSORS];
float last_proportional = 0;
float integral = 0;
unsigned long myTime;
const int limit = 100;
float frequency = 100.0;
bool going = false;

float kp = 0.03;
float ki = 0.0;
float kd = 1.0;

union FloatToBytes {
  float f_value;
  byte b_array[4];
};

struct CommandMessage {
  byte commandChar;
  FloatToBytes magnitude;  
  int checkSum;
};

enum class Side {
  RIGHT, LEFT
};

int MESSAGE_SIZE = 8;
const char END_MARKER = '\f';
const int BAUDRATE = 9600;

volatile int left_encoder_count=0;
volatile int right_encoder_count=0;
volatile bool stopConditionMet = false;

int velocity = 80;

auto stopRobot() -> void {
  if(stopConditionMet) {
    digitalWrite(PIN_LEFT_MOTOR_FORWARD, LOW);
    digitalWrite(PIN_LEFT_MOTOR_REVERSE, LOW);
    analogWrite(PIN_LEFT_MOTOR_SPEED, 0);

    digitalWrite(PIN_RIGHT_MOTOR_FORWARD, LOW);
    digitalWrite(PIN_RIGHT_MOTOR_REVERSE, LOW);
    analogWrite(PIN_RIGHT_MOTOR_SPEED, 0);

    stopConditionMet = false;
  }
}

void left_encoder(){
  left_encoder_count++;  
}

void right_encoder(){
  right_encoder_count++;  
}

void setup() {
  Serial.begin(BAUDRATE);

  for (int i = 0; i < 400; i++){                  // make the calibration take about 10 seconds
    trs.calibrate();                             // reads all sensors 10 times
  }

  Serial.print("Minimum values: ");  
  for (int i = 0; i < NUM_SENSORS; i++){
    Serial.print(trs.calibratedMin[i]);
    Serial.print(' ');
  }
  Serial.print("\t|||\t Maximum values:");
    for (int i = 0; i < NUM_SENSORS; i++){
    Serial.print(trs.calibratedMax[i]);
    Serial.print(' ');
  }
  delay(1000);

  // pinMode(TRIGER_PIN, OUTPUT);
  // pinMode(ECHO_PIN, INPUT);

  pinMode(PIN_LEFT_MOTOR_SPEED, OUTPUT);
  analogWrite(PIN_LEFT_MOTOR_SPEED, 0);
  pinMode(PIN_LEFT_MOTOR_FORWARD, OUTPUT);
  pinMode(PIN_LEFT_MOTOR_REVERSE, OUTPUT);

  pinMode(PIN_RIGHT_MOTOR_SPEED, OUTPUT);
  analogWrite(PIN_RIGHT_MOTOR_SPEED, 0);
  pinMode(PIN_RIGHT_MOTOR_FORWARD, OUTPUT);
  pinMode(PIN_RIGHT_MOTOR_REVERSE, OUTPUT);

  pinMode(INTERRUPT_PIN, INPUT_PULLUP);

  pinMode(ANALOG_READ_IR_LEFT, INPUT);
  pinMode(ANALOG_READ_IR_RIGHT, INPUT);

  attachInterrupt(digitalPinToInterrupt(PIN_LEFT_ENCODER), left_encoder, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_RIGHT_ENCODER), right_encoder, RISING);
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), stopRobot, FALLING);

  Serial.println("Connected to arduino successfully!");
}

auto readMessage(CommandMessage& receivedMessage) -> bool {
  byte msgBytes[MESSAGE_SIZE];
  for(int i = 0; i < MESSAGE_SIZE; i++) {
    msgBytes[i] = Serial.read();
  }
  if(((char) msgBytes[sizeof(msgBytes) - 1]) != END_MARKER) {
    Serial.print("ERROR: wrong end marker, expected: ");
    Serial.print(END_MARKER);
    Serial.print(", received: ");
    Serial.println((char) msgBytes[sizeof(msgBytes) - 1]);
    return false;
  }
  receivedMessage.commandChar = msgBytes[0];
  
  for(int i = 0; i < 4; i ++) {
    receivedMessage.magnitude.b_array[i] = msgBytes[i + 1];
  }
  receivedMessage.checkSum = (msgBytes[6] << 8) | msgBytes[5];

  return true;
}

auto checkSum(const CommandMessage& cm) -> bool {
  int calculatedCheckSum = (int)cm.commandChar;
  for(int i = 0; i < sizeof(cm.magnitude.b_array); i++) {
    calculatedCheckSum ^= cm.magnitude.b_array[i];
  }

  if(calculatedCheckSum == (int)(cm.checkSum & 0xFF)) {
    return true;
  } else {
    Serial.print("ERROR: wrong check sum, expected: ");
    Serial.print(calculatedCheckSum, HEX);
    Serial.print(", received: ");
    Serial.println((cm.checkSum & 0xFF), HEX);
    return false;
  }
}

auto PID(int position) -> int {
  float proportional = position - 2000;
  if(integral <= 1000 && integral >= -1000)
    integral += proportional*0.1;
  float derivative = (proportional - last_proportional)/(frequency / 1000.0);
  last_proportional = proportional;
  
  Serial.print(proportional);
  Serial.print(" | ");
  Serial.print(integral);
  Serial.print(" | ");
  Serial.print(derivative);
  Serial.print(" | ");
  Serial.print(left_encoder_count);
  Serial.print(" : ");
  Serial.print(right_encoder_count);

  return (int) (kp * proportional + ki * integral + kd * derivative);
}

auto restart_counters() -> void {
    right_encoder_count = 0;
    left_encoder_count = 0;
}

auto move(
    const Side& side, const int& LOW_HIGH_0, const int& LOW_HIGH_1, int velocity
) {
    if(side == Side::LEFT) {
      digitalWrite(PIN_LEFT_MOTOR_FORWARD, LOW_HIGH_0);
      digitalWrite(PIN_LEFT_MOTOR_REVERSE, LOW_HIGH_1);
      analogWrite(PIN_LEFT_MOTOR_SPEED, velocity);
    } else {
      digitalWrite(PIN_RIGHT_MOTOR_FORWARD, LOW_HIGH_0);
      digitalWrite(PIN_RIGHT_MOTOR_REVERSE, LOW_HIGH_1);
      analogWrite(PIN_RIGHT_MOTOR_SPEED, velocity);
  }
}

auto go() -> void {
  if((millis() - myTime) >= frequency){
    unsigned int position = trs.readLine(sensorValues);

    int u = PID(position);
    int pwm_l = velocity - u;
    int pwm_r = velocity + u;
    if(pwm_l > limit)
      pwm_l = limit;
    if(pwm_r > limit)
      pwm_r = limit;

    if (pwm_l < 0) 
      pwm_l = 0;
    if (pwm_r < 0) 
      pwm_r = 0;

    Serial.print("Correcting value: ");
    Serial.print(String(u));
    Serial.print(" | L: ");
    Serial.print(String(pwm_l));
    Serial.print(" | R: ");
    Serial.print(String(pwm_r));

    myTime = millis();

    move(Side::LEFT, HIGH, LOW, pwm_l);
    move(Side::RIGHT, LOW, HIGH, pwm_r);
    restart_counters();   
  }   
}

auto processCommand(const char& command, float& magnitude) -> void {
  Serial.print("ACK: Received command: ");
  Serial.print(command);
  Serial.print(" ");
  Serial.print(magnitude, 2);

  if(command == 'm') {
    Serial.println(", Ready to move!");
    magnitude = (int)magnitude;
    if(magnitude > 0) {

      move(Side::LEFT, LOW, HIGH, velocity);
      move(Side::RIGHT, HIGH, LOW, velocity);
    } else if (magnitude < 0) {

      move(Side::LEFT, HIGH, LOW, velocity);
      move(Side::RIGHT, LOW, HIGH, velocity);
      magnitude = -magnitude;
    }

    while(left_encoder_count < magnitude && right_encoder_count < magnitude) {
      Serial.print("");
    }

    move(Side::LEFT, LOW, LOW, 0);
    move(Side::RIGHT, LOW, LOW, 0);
    restart_counters();

  } else if(command == 'r') {
    Serial.println(", Ready to turn!");
    magnitude = (int)magnitude;

    if(magnitude > 0)
    {
      move(Side::LEFT, LOW, HIGH, velocity);
      while(left_encoder_count < magnitude) {
        Serial.print("");
      } 
      move(Side::LEFT, LOW, LOW, 0);
      restart_counters();

    } else if (magnitude < 0) {
      move(Side::RIGHT, HIGH, LOW, velocity);
      while(right_encoder_count < magnitude) {
        Serial.print("");
      } 
      move(Side::RIGHT, LOW, LOW, 0);
      restart_counters();
    } 
  } else if(command == 'v') {
    magnitude = (int)magnitude;

    velocity = magnitude;
    Serial.println(", elocity has been set");
  // } else if(command == 'b') {
  //   digitalWrite(TRIGER_PIN, LOW);
  //   delayMicroseconds(2);

  //   digitalWrite(TRIGER_PIN, HIGH);
  //   delayMicroseconds(10);
  //   digitalWrite(TRIGER_PIN, LOW);

  //   long duration = pulseIn(ECHO_PIN, HIGH);
  //   int distance = duration * 0.034 / 2;
  //   Serial.print(", Distance: ");
  //   Serial.print(distance);
  //   Serial.println(" cm");
  } else if(command == 'i') {

    Serial.print(", IR value left: ");
    Serial.print(analogRead(ANALOG_READ_IR_LEFT));
    Serial.print(" right: ");
    Serial.println(analogRead(ANALOG_READ_IR_RIGHT));
  } else if(command == 's') {
    stopConditionMet = true;
    Serial.println(", Stopping the robot");
  } else if(command == 'P') {
    kp = magnitude;
    Serial.println(", kp has been set");
  } else if(command == 'I') {
    ki = magnitude;
    Serial.println(", ki has been set");
  } else if(command == 'D') {
    kd = magnitude;
    Serial.println(", kd has been set");
  } else if(command == 'g') {
    going = true;
    Serial.println(", going!");
    myTime = millis();
  } else if(command == 'S') {
    going = false;
    Serial.println(", stopped");
  }
}



void loop() {
  if(Serial.available() >= MESSAGE_SIZE) {
    CommandMessage receivedMessage;
    if(readMessage(receivedMessage)) {
      if(checkSum(receivedMessage)) {
        processCommand((char)receivedMessage.commandChar, receivedMessage.magnitude.f_value);
      }
    }
  }
  if(going) {
    go();
  }
}

