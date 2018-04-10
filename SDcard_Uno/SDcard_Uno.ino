#include <SD.h>
#include "MPU9250.h"
MPU9250 my_mpu9250;

// SPI SD Card Pins
// MOSI = Pin 11
// MISO = Pin 12
// SCLK = Pin 13
int CS_pin  = 10;

float gyro_x, gyro_y, gyro_z;
unsigned long id = 1;

void setup() {
  Wire.begin();
  my_mpu9250.acceltempgyro_config(3, 4, 3, 4);
  delay(100);
  my_mpu9250.mag_config(1);
  delay(100);
  Serial.begin(115200);
  Serial.println("Initializing Card");
  pinMode(CS_pin, OUTPUT);
  if (!SD.begin(CS_pin)) {
    Serial.println("Card Failed");
    return;
  }
  Serial.println("Card Ready");
  File logFile = SD.open("LOG.csv", O_CREAT | O_WRITE);
  if (logFile) {
    logFile.println(" , , , ");
    String header = "ID, Gyro_X, Gyro_Y, Gyro_Z";
    logFile.println(header);
    logFile.flush();
    logFile.close();
    Serial.println(header);
  }
  else {
    Serial.println("Could not open log file");
  }
}

void loop() {
  my_mpu9250.gyro_sample(&gyro_x, &gyro_y, &gyro_z);
  String dataString  = String(id) + ", " + String(gyro_x) + ", " + String(gyro_y) + ", " + String(gyro_z);
  File logFile = SD.open("LOG.csv", FILE_WRITE);
  if (logFile) {
    logFile.println(dataString);
    logFile.flush();
    logFile.close();
    Serial.println(dataString);
  }
  else {
    Serial.println("Couldn't access file");
  }
  id++;
  delay(10);
}
