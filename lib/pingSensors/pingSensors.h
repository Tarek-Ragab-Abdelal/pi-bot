#define debugDistances false
#ifndef _pingSensors_h
#define _pingSensors_h

#include <NewPing.h>
#include <Arduino.h>

extern boolean frontwall;
extern boolean leftwall;
extern boolean rightwall;
extern int frontDistance;
extern int leftDistance;
extern int rightDistance;

void ping_detectWalls();

void ping_calibrateAxis();

#endif