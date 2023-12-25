#include "pingSensors.h"
#include "../../include/defines.h"

boolean frontwall = true;
boolean leftwall = true;
boolean rightwall = true;
int frontDistance = 0;
int leftDistance = 0;
int rightDistance = 0;

/************************************************************************************************************************/
NewPing ForwardSonar(FtrigPin, FechoPin, MAX_DISTANCE + 20);
NewPing LeftSonar(LtrigPin, LechoPin, MAX_DISTANCE);
NewPing RightSonar(RtrigPin, RechoPin, MAX_DISTANCE);
/************************************************************************************************************************/
void ping_detectWalls()
{
    frontDistance = ForwardSonar.ping_median(3);
    leftDistance = LeftSonar.ping_median(3);
    rightDistance = RightSonar.ping_median(3);
    leftDistance = LeftSonar.convert_cm(leftDistance);
    rightDistance = RightSonar.convert_cm(rightDistance);
    frontDistance = ForwardSonar.convert_cm(frontDistance);

    if (debugDistances)
    {
        Serial.print("Front: ");
        Serial.print(frontDistance);
        Serial.print("cm\t");
        Serial.print("Right: ");
        Serial.print(rightDistance);
        Serial.print("cm\t");
        Serial.print("Left: ");
        Serial.print(leftDistance);
        Serial.println("cm\t");
    }

    if (leftDistance <= wall_threshold)
    {
        leftwall = true;
    }
    else
    {
        leftwall = false;
    }
    if (rightDistance <= wall_threshold)
    {
        rightwall = true;
    }
    else
    {
        rightwall = false;
    }
    if (frontDistance <= front_threshold)
    {
        frontwall = true;
    }
    else
    {
        frontwall = false;
    }
}

void ping_calibrateAxis()
{
    frontDistance = ForwardSonar.ping_cm();
    leftDistance = LeftSonar.ping_cm();
    rightDistance = RightSonar.ping_cm();
}