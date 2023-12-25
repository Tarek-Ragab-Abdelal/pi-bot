#include "defines.h"

// Redefine the volatage level as measured before the game
#ifdef BATTERIES_VOLTAGE
#undef BATTERIES_VOLTAGE
#define BATTERIES_VOLTAGE 9.5
#endif

#include "pingSensors.h"
#include "motionShield.h"

bool left = false;

void setup()
{
  Serial.begin(115200);
}
void loop()
{
  ping_detectWalls();

  manualControl(left); // Work as RIGHT WALL FOLLOWER

  //  PID_fullControl();
}
