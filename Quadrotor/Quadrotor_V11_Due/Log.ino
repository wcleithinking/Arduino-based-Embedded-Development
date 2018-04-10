#ifdef SD_SAVE
void Log() {
  // CS Pin is an output
  pinMode(CS_Pin, OUTPUT);
  if (!sd.begin(CS_Pin, SPI_HALF_SPEED)) {
#ifdef DEBUG
    Serial.println("Card Failed");
#endif
    return;
  }
#ifdef DEBUG
  Serial.println("Card Ready");
#endif
  if (logFile.open("LOG.csv", O_RDWR | O_CREAT | O_AT_END)) {
    logFile.println(", , , ,");
    String header = "ID, Roll, Pitch, Yaw";
    logFile.println(header);
    logFile.close();
#ifdef DEBUG
    Serial.println(header);
#endif
  }
  else {
#ifdef DEBUG
    Serial.println("Cound not open log file");
#endif
  }
}

void log_savedata() {
  String dataString = String(id) +  ", " + String(roll * RadToDeg) + ", " + String(pitch * RadToDeg) + ", " + String(yaw * RadToDeg);
  if (logFile.open("LOG.csv", O_RDWR | O_CREAT | O_AT_END)) {
    logFile.println(dataString);
    logFile.close();
  }
  id++;
}
#endif
