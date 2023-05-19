#include "defines.h"
#include "pingSensors.h"
#include "motionShield.h"

bool left = false;
long timer = 0;
long interval = 5000;

#define BATTERIES_VOLTAGE 9.5

void setup()
{
  Serial.begin(115200);
}
void loop()
{
  if (millis() >= interval)
  {
    left = true;
  }
  detectWalls();
  manualControl(left);
  //  PID_fullControl();
}
