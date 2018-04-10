#ifdef LOG

SdFat     Card;
SdFile    FlightLog_csv;
unsigned long id = 1;

void Log_init() {
  pinMode(CSPin, OUTPUT);
  if (Card.begin(CSPin, SPI_HALF_SPEED)) {
    if (FlightLog_csv.open("flightlog.csv", O_RDWR | O_CREAT | O_AT_END)) {
#ifdef STR_v1
      String header = "ID, RollD, PitchD, YawD, Roll, Pitch, Yaw, RateD, Rate, a1, a2, b0, b1";
#else
      String header = "ID, RollD, PitchD, YawD, Roll, Pitch, Yaw";
#endif
      FlightLog_csv.println(header);
      FlightLog_csv.close();
    }
  }
}

void Log_savedata() {
  if (FlightLog_csv.open("flightlog.csv", O_RDWR | O_CREAT | O_AT_END)) {
#ifdef STR_v1
    String dataString  = String(id)
                         + ", " + String(Angle_eulerdesire[IndexRoll], 2)
                         + ", " + String(Angle_eulerdesire[IndexPitch], 2)
                         + ", " + String(Angle_eulerdesire[IndexYaw], 2)
                         + ", " + String(Angle_newestimate[IndexRoll] * RadToDeg, 2)
                         + ", " + String(Angle_newestimate[IndexPitch] * RadToDeg, 2)
                         + ", " + String(Angle_newestimate[IndexYaw] * RadToDeg, 2)
                         + ", " + String(Rate_middledesire[STR_Index], 2)
                         + ", " + String(Rate_threemeasure[STR_Index] * RadToDeg, 2)
                         + ", " + String(a1, 4)
                         + ", " + String(a2, 4)
                         + ", " + String(b0, 4)
                         + ", " + String(b1, 4);
#else
    String dataString  = String(id)
                         + ", " + String(Angle_eulerdesire[IndexRoll], 2)
                         + ", " + String(Angle_eulerdesire[IndexPitch], 2)
                         + ", " + String(Angle_eulerdesire[IndexYaw], 2)
                         + ", " + String(Angle_newestimate[IndexRoll] * RadToDeg, 2)
                         + ", " + String(Angle_newestimate[IndexPitch] * RadToDeg, 2)
                         + ", " + String(Angle_newestimate[IndexYaw] * RadToDeg, 2);
#endif
    FlightLog_csv.println(dataString);
    FlightLog_csv.close();
    id++;
  }
}

#endif
