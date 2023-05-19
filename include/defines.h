#ifndef _DEFINES_H_
#define _DEFINES_H_

// Left
#define LPWN 4
#define L1 A0
#define L2 A1
// Right
#define RPWN 5
#define R1 6
#define R2 10
/************************************************************************************************************************/
#define BATTERIES_VOLTAGE 11.5
#define PWM_MIN ((3.15 / BATTERIES_VOLTAGE) * 255)
#define PWM_BASE ((4.0 / BATTERIES_VOLTAGE) * 255)
#define PWM_MAX ((5.1 / BATTERIES_VOLTAGE) * 255)
/*******************************************************PID**************************************************************/
#define Kp 0.9
#define Kd 0.4
#define Ki 0.05
/************************************************************************************************************************/

// UltraSonic pins
// Forward
#define FechoPin A0
#define FtrigPin A1
// Right
#define RechoPin A4
#define RtrigPin A5
// Left
#define LechoPin A2
#define LtrigPin A3

#define MAX_DISTANCE 85
#define wall_threshold 17
#define front_threshold 13

#endif