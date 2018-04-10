#ifdef LOG
SdFat     Card;
SdFile    Log_csv;
unsigned long id = 1;

void Log_init() {
  pinMode(CSPin, OUTPUT);
  if (Card.begin(CSPin, SPI_HALF_SPEED)) {
    if (Log_csv.open("LOG.csv", O_RDWR | O_CREAT | O_AT_END)) {
#ifdef STR_v1
      String header = "ID, Roll, Pitch, Yaw, a1, a2, b0, b1";
#else
      String header = "ID, Roll, Pitch, Yaw";
#endif
      Log_csv.println(header);
      Log_csv.close();
    }
    else errorCount++;
  }
  else errorCount++;
}

void Log_savedata() {
  if (errorCount == 0) {
    if (Log_csv.open("LOG.csv", O_RDWR | O_CREAT | O_AT_END)) {
#ifdef STR_v1
      String dataString  = String(id)
                           + ", " + String(Angle_measure[IndexRoll] * RadToDeg, 2)
                           + ", " + String(Angle_measure[IndexPitch] * RadToDeg, 2)
                           + ", " + String(Angle_measure[IndexYaw] * RadToDeg, 2)
                           + ", " + String(a1[STR_Index], 4)
                           + ", " + String(a2[STR_Index], 4)
                           + ", " + String(b0[STR_Index], 4)
                           + ", " + String(b1[STR_Index], 4);
#else
      String dataString  = String(id)
                           + ", " + String(Angle_measure[IndexRoll] * RadToDeg, 2)
                           + ", " + String(Angle_measure[IndexPitch] * RadToDeg, 2)
                           + ", " + String(Angle_measure[IndexYaw] * RadToDeg, 2);
#endif
      Log_csv.println(dataString);
      Log_csv.close();
      id++;
    }
    else errorCount++;
  }
}
#endif
