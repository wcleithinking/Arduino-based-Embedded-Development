//#define DEBUG
//#define Debug_graph
//#define Debug_table

#define QuadX

//#define BARO
#define SONAR

#define PID_v1
//#define PID_v2
//#define PID_v2_MODIFIED

#define RadToDeg  (180/PI)
#define DegToRad  (PI/180)
#define G_ACCEL 9.8015

#define Throttle_MIN 1016
#define Throttle_MAX 2016

#define ledPin 13
#define motor0Pin 3
#define motor1Pin 9
#define motor2Pin 10
#define motor3Pin 11

#if defined(BARO)
#define BARO_OSR 3
#define BARO_DELAY 5
#define BARO_CPF 0.02
#define P_SEALEVEL 101325
#elif defined(SONAR)
#define echoPin 2
#define trigPin 12
#define SONAR_DELAY 18
#define SONAR_SCALE 58
#define SONAR_CPF 0.05
#define SONAR_MAX 80
#define SONAR_MIN 5
#endif
