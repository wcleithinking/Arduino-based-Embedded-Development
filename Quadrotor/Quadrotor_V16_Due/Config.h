// Debug
//#define BaudRate  115200
//#define DEBUG
//#define Debug_single

// Log
#define LOG

// Frame Type
#define QuadP

// Indexes
#define IndexRoll             0
#define IndexPitch            1
#define IndexYaw              2
#define IndexAltitude         3

// Control Algorithms
#define FEEDDESIRE
#define LOOP_period           10
#define PID_v1
#define STR_v1
#define STR_Index             IndexYaw
#define STR_period            40

// Constants
#define Pulse_MIN_Original    1016
#define Pulse_MAX_Original    2016
#define Pulse_MIN             1000
#define Pulse_MAX             2000
#define PWM_MIN               1000
#define PWM_MAX               2000
#define Roll_MIN              -50
#define Roll_MAX              50
#define Pitch_MIN             -50
#define Pitch_MAX             50
#define Yaw_MIN               -90
#define Yaw_MAX               90
#define RadToDeg              (180/PI)
#define DegToRad              (PI/180)

// Pins
#define channel1Pin           2
#define channel2Pin           3
#define channel3Pin           4
#define channel4Pin           5
#define motor0Pin             6
#define motor1Pin             7
#define motor2Pin             8
#define motor3Pin             9
#define CSPin                 10
#define ledPin                13
