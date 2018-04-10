#include <Servo.h>
Servo motor1, motor2, motor3, motor4;

// Pin
#define channel1Pin   2
#define channel2Pin   3
#define channel3Pin   4
#define channel4Pin   5
#define motor1Pin     6
#define motor2Pin     7
#define motor3Pin     8
#define motor4Pin     9

// PWM
#define Pulse_Min_O   1016
#define Pulse_Max_O   2016
#define Pulse_Min     1000
#define Pulse_Max     2000
#define PWM_MIN       1000
#define PWM_MAX       2000


volatile int pulse1, pulse2, pulse3, pulse4;
int PWM_out[4] = {PWM_MIN, PWM_MIN, PWM_MIN, PWM_MIN};

void setup() {
  attachInterrupt(channel1Pin, updatepulse1, CHANGE);
  attachInterrupt(channel2Pin, updatepulse2, CHANGE);
  attachInterrupt(channel3Pin, updatepulse3, CHANGE);
  attachInterrupt(channel4Pin, updatepulse4, CHANGE);
  motor1.attach(motor1Pin);
  motor2.attach(motor2Pin);
  motor3.attach(motor3Pin);
  motor4.attach(motor4Pin);
  Serial.begin(115200);
}

void loop() {
  for (int i = 0; i < 4; i++) PWM_out[i] = pulse3;
  motor1.writeMicroseconds(PWM_out[0]);
  motor2.writeMicroseconds(PWM_out[1]);
  motor3.writeMicroseconds(PWM_out[2]);
  motor4.writeMicroseconds(PWM_out[3]);
  Serial.println(pulse3);
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
