#define debugSpeeds false

#include <AFMotor.h>
#include <Arduino.h>

extern AF_DCMotor leftMotor;
extern AF_DCMotor rightMotor;

extern int RMS;
extern int LMS;

/************************************************************************************************************************/
enum directions
{
  RobotSTOP,
  RobotFORWARD,
  RobotBACKWARD,
  RobotLEFT,
  RobotRIGHT,
  RobotROTATEACW,
  RobotROTATECW,
};

/************************************************************************************************************************/
void moveRobot(directions dir, int rms = RMS, int lms = LMS);
/************************************************************************************************************************/

#ifdef _pingSensors_h
/**********************************************************Motion Actions**********************************************************/
void ForwardWithLeftwall_PID();

void ForwardWithRightwall_PID();

void Left90andStep();

void Right90andStep();

void Rotate180();

/***************************************************************************************************************************/

void manualControl(bool left);

void manualControl_hardcoded();

/***************************************************************************************************************************/
void PID_fullControl();

/***************************************************************************************************************************/

#endif
