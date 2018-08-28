#include <Servo.h>

Servo motor1, motor2, motor3, motor4;

// for ISR
volatile int pulse1, pulse2, pulse3, pulse4;
volatile boolean prestate1 = 0, prestate2 = 0, prestate3 = 0, prestate4 = 0;
volatile unsigned long  current_time, rising1, rising2, rising3, rising4;

void setup() {
  // set pin change interrupt control register
  PCICR |= (1 << PCIE2);  // pin change interrupt enable 0: PCINT[23:16], which are enabled by PCMSK2
  // set pin change mask register 2
  PCMSK2 |= (1 << PCINT20);  // pin 4
  PCMSK2 |= (1 << PCINT21);  // pin 5
  PCMSK2 |= (1 << PCINT22);  // pin 6
  PCMSK2 |= (1 << PCINT23);  // pin 7
  // choose PWM pins
  motor1.attach(3);  // pin 3
  motor2.attach(9);  // pin 9
  motor3.attach(10); // pin 10
  motor4.attach(11); // pin 11
  Serial.begin(115200);
}

void loop() {
	motor1.writeMicroseconds(1000);
	motor2.writeMicroseconds(1000);
	motor3.writeMicroseconds(1000);
	motor4.writeMicroseconds(1000);
//  Serial.print("PPM1=");
//  Serial.print(pulse1);
//  Serial.print("\t");
//  Serial.print("PPM2=");
//  Serial.print(pulse2);
//  Serial.print("\t");
//  Serial.print("PPM3=");
//  Serial.print(pulse3);
//  Serial.print("\t");
//  Serial.print("PPM4=");
//  Serial.print(pulse4);
//  Serial.println("\t");
//  delay(100);
}


ISR(PCINT2_vect) {
  // for all channels, pulse_min is 1012/1016 and pulse_max is 2016/2020 
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
  }
}
