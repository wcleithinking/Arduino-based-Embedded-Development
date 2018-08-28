// Pin
#define motor1Pin     3
#define motor2Pin     9
#define motor3Pin     10
#define motor4Pin     11

// PWM
#define Pulse_Min_O   1016
#define Pulse_Max_O   2016
#define Pulse_Min     1000
#define Pulse_Max     2000
#define PWM_MIN       1000
#define PWM_MAX       2000

/*
   Interrupt Varibales
*/
volatile int pulse1, pulse2, pulse3, pulse4;
volatile bool prestate1 = 0, prestate2 = 0, prestate3 = 0, prestate4 = 0;
volatile unsigned long  current_time, rising1, rising2, rising3, rising4;

/*
   Electronic Speed Controller
*/
int PWM_out[4] = {PWM_MIN, PWM_MIN, PWM_MIN, PWM_MIN};
int MOTOR[4] = {127, 127, 127, 127};

void setup() {
  Receiver();
  Serial.begin(115200);
}

void loop() {
  motor_update();
  Serial.println(pulse3);
}

void motor_update() {
  for (int i = 0; i < 4; i++) {
    // need modification for more accurate PWM input
    MOTOR[i] = map(pulse3, PWM_MIN, PWM_MAX, 127, (int)(127 + 0.128 * (PWM_MAX - PWM_MIN)));
  }
  analogWrite(motor1Pin, MOTOR[0]);
  analogWrite(motor2Pin, MOTOR[1]);
  analogWrite(motor3Pin, MOTOR[2]);
  analogWrite(motor4Pin, MOTOR[3]);
}

void Receiver() {
  PCICR  |= (1 << PCIE2);     // pin change interrupt enable 0: PCINT[23:16], which are enabled by PCMSK2
  PCMSK2 |= (1 << PCINT20);   // pin 4
  PCMSK2 |= (1 << PCINT21);   // pin 5
  PCMSK2 |= (1 << PCINT22);   // pin 6
  PCMSK2 |= (1 << PCINT23);   // pin 7
}

ISR(PCINT2_vect) {
  current_time = micros();
  // channel 1: digital pin 4 : roll signal
  if (PIND & 0b00010000) {
    if (prestate1 == 0) {
      prestate1 = 1;
      rising1 = current_time;
    }
  }
  else if (prestate1 == 1) {
    prestate1 = 0;
    pulse1 = current_time - rising1;
    pulse1 = constrain(pulse1, Pulse_Min_O, Pulse_Max_O);
    pulse1 = map(pulse1, Pulse_Min_O, Pulse_Max_O, Pulse_Min, Pulse_Max);
  }
  // channel 2: digital pin 5 : pitch signal
  if (PIND & 0b00100000) {
    if (prestate2 == 0) {
      prestate2 = 1;
      rising2 = current_time;
    }
  }
  else if (prestate2 == 1) {
    prestate2 = 0;
    pulse2 = current_time - rising2;
    pulse2 = constrain(pulse2, Pulse_Min_O, Pulse_Max_O);
    pulse2 = map(pulse2, Pulse_Min_O, Pulse_Max_O, Pulse_Min, Pulse_Max);
  }
  // channel 3: digital pin 6 : thrust signal
  if (PIND & 0b01000000) {
    if (prestate3 == 0) {
      prestate3 = 1;
      rising3 = current_time;
    }
  }
  else if (prestate3 == 1) {
    prestate3 = 0;
    pulse3 = current_time - rising3;
    pulse3 = constrain(pulse3, Pulse_Min_O, Pulse_Max_O);
    pulse3 = map(pulse3, Pulse_Min_O, Pulse_Max_O, Pulse_Min, Pulse_Max);
  }
  // channel 4: digital pin 7 : yaw signal
  if (PIND & 0b10000000) {
    if (prestate4 == 0) {
      prestate4 = 1;
      rising4 = current_time;
    }
  }
  else if (prestate4 == 1) {
    prestate4 = 0;
    pulse4 = current_time - rising4;
    pulse4 = constrain(pulse4, Pulse_Min_O, Pulse_Max_O);
    pulse4 = map(pulse4, Pulse_Min_O, Pulse_Max_O, Pulse_Min, Pulse_Max);
  }
}
