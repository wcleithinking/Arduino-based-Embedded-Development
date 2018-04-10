#include <Servo.h>
Servo motor1, motor2, motor3, motor4;

#define channel1Pin 2
#define channel2Pin 3
#define channel3Pin 4
#define channel4Pin 5

#define Pulse_Min_O 1016
#define Pulse_Max_O 2016
#define Pulse_Min 1000
#define Pulse_Max 2000

// for pulses
volatile uint32_t pulse1, pulse2, pulse3, pulse4;

void setup() {
  motor1.attach(8);  // pin 3
  motor2.attach(9);  // pin 9
  motor3.attach(10); // pin 10
  motor4.attach(11); // pin 11
  Serial.begin(115200);
  // attach the interrupts used to read the pulses
  attachInterrupt(channel1Pin, updatepulse1, CHANGE);
  attachInterrupt(channel2Pin, updatepulse2, CHANGE);
  attachInterrupt(channel3Pin, updatepulse3, CHANGE);
  attachInterrupt(channel4Pin, updatepulse4, CHANGE);
}

void loop() {
  motor1.writeMicroseconds(1000);
  motor2.writeMicroseconds(1000);
  motor3.writeMicroseconds(1000);
  motor4.writeMicroseconds(1000);
  Serial.print("pulse1:");
  Serial.print(pulse1);
  Serial.print("\t");
  Serial.print("pulse2:");
  Serial.print(pulse2);
  Serial.print("\t");
  Serial.print("pulse3:");
  Serial.print(pulse3);
  Serial.print("\t");
  Serial.print("pulse4:");
  Serial.print(pulse4);
  Serial.println("\t");
}

void updatepulse1() {
  static uint32_t rising1;
  if (digitalRead(channel1Pin)) rising1 = micros();
  else {
    pulse1 = (uint32_t)(micros() - rising1);
    pulse1 = constrain(pulse1, Pulse_Min_O, Pulse_Max_O);
    pulse1 = map(pulse1, Pulse_Min_O, Pulse_Max_O, Pulse_Min, Pulse_Max);
  }
}

void updatepulse2() {
  static uint32_t rising2;
  if (digitalRead(channel2Pin)) rising2 = micros();
  else {
    pulse2 = (uint32_t)(micros() - rising2);
    pulse2 = constrain(pulse2, Pulse_Min_O, Pulse_Max_O);
    pulse2 = map(pulse2, Pulse_Min_O, Pulse_Max_O, Pulse_Min, Pulse_Max);
  }
}

void updatepulse3() {
  static uint32_t rising3;
  if (digitalRead(channel3Pin)) rising3 = micros();
  else {
    pulse3 = (uint32_t)(micros() - rising3);
    pulse3 = constrain(pulse3, Pulse_Min_O, Pulse_Max_O);
    pulse3 = map(pulse3, Pulse_Min_O, Pulse_Max_O, Pulse_Min, Pulse_Max);
  }
}

void updatepulse4() {
  static uint32_t rising4;
  if (digitalRead(channel4Pin)) rising4 = micros();
  else {
    pulse4 = (uint32_t)(micros() - rising4);
    pulse4 = constrain(pulse4, Pulse_Min_O, Pulse_Max_O);
    pulse4 = map(pulse4, Pulse_Min_O, Pulse_Max_O, Pulse_Min, Pulse_Max);
  }
}
