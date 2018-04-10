#ifdef LOG
SdFat     Card;
SdFile    Log_csv;
unsigned long id = 1;

void Log_init() {
  pinMode(CSPin, OUTPUT);
  if (Card.begin(CSPin, SPI_HALF_SPEED)) {
    if (Log_csv.open("LOG.csv", O_RDWR | O_CREAT | O_AT_END)) {
#ifdef STR_v1
      String header = "ID, Rolld, Pitchd, Yawd, Roll, Pitch, Yaw, RateDesired, RateMeasure, a1, a2, b0, b1";
#else
      String header = "ID, Rolld, Pitchd, Yawd, Roll, Pitch, Yaw";
#endif
      Log_csv.println(header);
      Log_csv.close();
    }
  }
}

void Log_savedata() {
  if (Log_csv.open("LOG.csv", O_RDWR | O_CREAT | O_AT_END)) {
#ifdef STR_v1
    String dataString  = String(id)
                         + ", " + String(Angle_desire[IndexRoll], 2)
                         + ", " + String(Angle_desire[IndexPitch], 2)
                         + ", " + String(Angle_desire[IndexYaw], 2)
                         + ", " + String(Angle_measure[IndexRoll] * RadToDeg, 2)
                         + ", " + String(Angle_measure[IndexPitch] * RadToDeg, 2)
                         + ", " + String(Angle_measure[IndexYaw] * RadToDeg, 2)
                         + ", " + String(Rate_desire[STR_Index], 2)
                         + ", " + String(Rate_measure[STR_Index] * RadToDeg, 2)
                         + ", " + String(a1, 4)
                         + ", " + String(a2, 4)
                         + ", " + String(b0, 4)
                         + ", " + String(b1, 4);
#else
    String dataString  = String(id)
                         + ", " + String(Angle_desire[IndexRoll], 2)
                         + ", " + String(Angle_desire[IndexPitch], 2)
                         + ", " + String(Angle_desire[IndexYaw], 2)
                         + ", " + String(Angle_measure[IndexRoll] * RadToDeg, 2)
                         + ", " + String(Angle_measure[IndexPitch] * RadToDeg, 2)
                         + ", " + String(Angle_measure[IndexYaw] * RadToDeg, 2);
#endif
    Log_csv.println(dataString);
    Log_csv.close();
    id++;
  }
}
#endif
