#include "motionShield.h"
#include "../pingSensors/pingSensors.h"
#include "../../include/defines.h"

/************************************************************************************************************************/
#ifndef BATTERIES_VOLTAGE
#define BATTERIES_VOLTAGE 11.7

#define PWM_MIN ((3.4 / BATTERIES_VOLTAGE) * 255)
#define PWM_BASE ((4.4 / BATTERIES_VOLTAGE) * 255)
#define PWM_MAX ((6.3 / BATTERIES_VOLTAGE) * 255)
#endif

/************************************************************************************************************************/

String path = "";
float oldErrorP;
float totalError;
int RMS = PWM_BASE;
int LMS = PWM_BASE;

AF_DCMotor leftMotor(3);
AF_DCMotor rightMotor(4);

/************************************************************************************************************************/

void moveRobot(directions dir, int rms, int lms)
{
    leftMotor.setSpeed(lms);
    rightMotor.setSpeed(rms);
    switch (dir)
    {
    case RobotSTOP:
        leftMotor.run(RELEASE);
        rightMotor.run(RELEASE);
        break;
    case RobotFORWARD:
        leftMotor.run(FORWARD);
        rightMotor.run(FORWARD);
        break;
    case RobotBACKWARD:
        leftMotor.run(BACKWARD);
        rightMotor.run(BACKWARD);
        break;
    case RobotLEFT:
        leftMotor.run(BACKWARD);
        rightMotor.run(FORWARD);
        break;
    case RobotRIGHT:
        leftMotor.run(FORWARD);
        rightMotor.run(BACKWARD);
        break;
    case RobotROTATEACW:
        leftMotor.run(BACKWARD);
        rightMotor.run(FORWARD);
        break;
    case RobotROTATECW:
        leftMotor.run(FORWARD);
        rightMotor.run(BACKWARD);
        break;
    }
}

/************************************************************************************************************************/

/**********************************************************Motion Actions**********************************************************/

void ForwardWithLeftwall_PID()
{
    if (debugSpeeds)
    {
        Serial.println("Forward with PID");
    }
    float errorP = leftDistance - ((rightDistance >= 20) ? 0 : rightDistance);
    float errorD = errorP - oldErrorP;
    float errorI = (2.0 / 3.0) * ((errorI >= 0) ? errorI : 0) + errorP;
    totalError = (Kp * errorP) + (Kd * errorD) + (Ki * errorI);
    oldErrorP = errorP;
    RMS = PWM_BASE + totalError;
    LMS = PWM_BASE - totalError;
    if (RMS < -PWM_MAX)
        RMS = -PWM_MAX;
    if (RMS > PWM_MAX)
        RMS = PWM_MAX;
    if (LMS < -PWM_MAX)
        LMS = -PWM_MAX;
    if (LMS > PWM_MAX)
        LMS = PWM_MAX;
    moveRobot(RobotFORWARD,
              (frontDistance <= 40) ? (RMS - 20)
                                    : ((frontDistance <= 20) ? 0 : (RMS - 20)),
              (frontDistance <= 40) ? (LMS - 20)
                                    : ((frontDistance <= 20) ? 0 : (LMS - 20)));
    if (debugSpeeds)
    {
        Serial.print("RMS");
        Serial.print(RMS);
        Serial.print("\tLMS");
        Serial.println(LMS);
    }
}

void ForwardWithRightwall_PID()
{
    if (debugSpeeds)
    {
        Serial.println("Forward with PID");
    }
    float errorP = rightDistance - ((leftDistance >= 20) ? 0 : leftDistance);
    float errorD = errorP - oldErrorP;
    float errorI = (2.0 / 3.0) * ((errorI >= 0) ? errorI : 0) + errorP;
    totalError = (Kp * errorP) + (Kd * errorD) + (Ki * errorI);
    oldErrorP = errorP;
    RMS = PWM_BASE - totalError;
    LMS = PWM_BASE + totalError;
    if (RMS < -PWM_MAX)
        RMS = -PWM_MAX;
    if (RMS > PWM_MAX)
        RMS = PWM_MAX;
    if (LMS < -PWM_MAX)
        LMS = -PWM_MAX;
    if (LMS > PWM_MAX)
        LMS = PWM_MAX;
    moveRobot(RobotFORWARD,
              (frontDistance <= 40) ? (RMS - 20)
                                    : ((frontDistance <= 20) ? 0 : (RMS - 20)),
              (frontDistance <= 40) ? (LMS - 20)
                                    : ((frontDistance <= 20) ? 0 : (LMS - 20)));
    if (debugSpeeds)
    {
        Serial.print("RMS");
        Serial.print(RMS);
        Serial.print("\tLMS");
        Serial.println(LMS);
    }
}

void Left90andStep()
{
    if (debugSpeeds)
    {
        Serial.println("Step Left");
    }
    int lastleft = leftDistance;
    int lastfront = frontDistance;
    bool rotated = false;
    do
    {
        moveRobot(RobotLEFT, PWM_BASE, PWM_BASE);
        ping_detectWalls();
        rotated = (!leftwall) && (abs(frontDistance - lastleft) <= 5) && (abs(rightDistance - lastfront) <= 5);
    } while (rotated);
    moveRobot(RobotSTOP);
    for (int i = 0; i <= 100; i++)
    {
        ForwardWithLeftwall_PID();
        ping_detectWalls();
        delay(10);
    }
    moveRobot(RobotSTOP);
}

void Right90andStep()
{
    if (debugSpeeds)
    {
        Serial.println("Step Right");
    }
    int lastfront = frontDistance;
    bool rotated = false;
    do
    {
        moveRobot(RobotRIGHT, PWM_MIN, PWM_MIN);
        ping_detectWalls();
        rotated = (!rightwall) && (leftwall) && (abs(rightDistance - lastfront) <= 5);
    } while (rotated);
    moveRobot(RobotSTOP);
    for (int i = 0; i <= 100; i++)
    {
        ForwardWithLeftwall_PID();
        ping_detectWalls();
        delay(10);
    }
    moveRobot(RobotSTOP);
}

void Rotate180()
{
    if (debugSpeeds)
    {
        Serial.print("Rotate 180 ");
    }
    moveRobot(RobotSTOP);
    bool isOut = false;
    bool CW = (leftDistance - rightDistance <= 0);
    Serial.println((CW) ? "ClockWise" : "AntiClockwise");
    do
    {
        isOut = (!frontwall && (leftwall && rightwall));
        moveRobot((CW) ? RobotROTATECW : RobotROTATEACW);
        ping_detectWalls();
    } while (!isOut);
    moveRobot(RobotSTOP);
}

/***************************************************************************************************************************/

void manualControl(bool left)
{
    /**********************************************************LEFT**********************************************************/
    if (left)
    {
        if (!leftwall)
        {
            // Left90andStep();
            moveRobot(RobotLEFT, PWM_BASE, PWM_BASE);
            delay(300);
            moveRobot(RobotFORWARD, PWM_BASE, PWM_BASE);
            delay(100);
        }
        /**********************************************************FORWARD**********************************************************/
        else if (!frontwall)
        {
            ForwardWithLeftwall_PID();
        }
        /**********************************************************RIGHT************************************************************/
        else if (!rightwall)
        {
            // Right90andStep();
            moveRobot(RobotRIGHT, PWM_BASE + 20, PWM_BASE);
            delay(300);
            moveRobot(RobotFORWARD, PWM_BASE, PWM_BASE);
            delay(100);
        }
        /**********************************************************ROTATE***********************************************************/
        else
        {
            Rotate180();
        }
        /***************************************************************************************************************************/
    }
    else
    {
        if (!rightwall)
        {
            // Left90andStep();
            moveRobot(RobotRIGHT, PWM_BASE + 20, PWM_BASE);

            delay(300);
            moveRobot(RobotFORWARD, PWM_BASE, PWM_BASE);
            delay(100);
        }
        /**********************************************************FORWARD**********************************************************/
        else if (!frontwall)
        {
            ForwardWithRightwall_PID();
        }
        /**********************************************************RIGHT************************************************************/
        else if (!leftwall)
        {
            // Right90andStep();
            moveRobot(RobotLEFT, PWM_BASE, PWM_BASE + 20);
            delay(300);
            moveRobot(RobotFORWARD, PWM_BASE, PWM_BASE);
            delay(100);
        }
        /**********************************************************ROTATE***********************************************************/
        else
        {
            Rotate180();
        }
        /***************************************************************************************************************************/
    }
}

void manualControl_hardcoded()
{
    char path_arr[path.length() + 1];
    path.toCharArray(path_arr, path.length() + 1);
    for (unsigned int i = 0; i <= path.length() + 1; i++)
    {
        switch (path_arr[i])
        {
        case 'F':
            moveRobot(RobotFORWARD, PWM_BASE, PWM_BASE);
            delay(250);
            break;
        case 'L':
            moveRobot(RobotLEFT, PWM_BASE, PWM_BASE);
            delay(350);
            moveRobot(RobotFORWARD, PWM_BASE, PWM_BASE);
            delay(120);
            break;
        case 'R':
            moveRobot(RobotRIGHT, PWM_BASE, PWM_BASE);
            delay(350);
            moveRobot(RobotFORWARD, PWM_BASE, PWM_BASE);
            delay(120);
            break;
        }
    }
}

/***************************************************************************************************************************/

void PID_fullControl()
{
    if (frontwall && rightwall && leftwall)
    {
        Rotate180();
    }
    else
    {
        float errorP = leftDistance - rightDistance;
        float errorD = errorP - oldErrorP;
        float errorI = (2.0 / 3.0) * errorI + errorP;
        totalError = Kp * errorP + Kd * errorD + Ki * errorI;
        oldErrorP = errorP;
        RMS = PWM_BASE + totalError;
        LMS = PWM_BASE - totalError;
        if (RMS < -PWM_MAX)
            RMS = -PWM_MAX;
        if (RMS > PWM_MAX)
            RMS = PWM_MAX;
        if (LMS < -PWM_MAX)
            LMS = -PWM_MAX;
        if (LMS > PWM_MAX)
            LMS = PWM_MAX;
        if (RMS < 0)
        {
            /*Right*/
            RMS = map(RMS, -PWM_MIN, -PWM_MAX, PWM_MIN, PWM_MAX);
            moveRobot(RobotFORWARD, RMS, LMS);
        }
        else if (LMS < 0)
        {
            /*Left*/
            LMS = map(LMS, -PWM_MIN, -PWM_MAX, PWM_MIN, PWM_MAX);
            moveRobot(RobotFORWARD, RMS, LMS);
        }
        else
        {
            /*Forward*/
            moveRobot(RobotFORWARD,
                      (frontDistance <= 40) ? (RMS - 20)
                                            : ((frontDistance <= 20) ? 0 : (RMS - 20)),
                      (frontDistance <= 40) ? (LMS - 20)
                                            : ((frontDistance <= 20) ? 0 : (LMS - 20)));
        }
        if (debugSpeeds)
        {
            Serial.print("RMS");
            Serial.print(RMS);
            Serial.print("\tLMS");
            Serial.println(LMS);
        }
    }
}

/***************************************************************************************************************************/
