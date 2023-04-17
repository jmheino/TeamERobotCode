// Required proteus firmware libraries
#include <FEHUtility.h>
#include <FEHLCD.h>
#include <FEHSD.h>
#include <FEHIO.h>
#include <FEHServo.h>
#include <FEHMotor.h>

// Required personal libraries
#include "include/constants.h"
#include "include/globals.h"
#include "include/functions.h"

void Checkpoint1(){

//Line up robot 90 degrees from normal starting position facing wall aligned with less steep ramp
initRobot();
robotStart();
driveDistance(4, LOW_SPEED_PCT);
turnLeftDegree(45);
driveDistance(36, LOW_SPEED_PCT);
left90();
driveDistance(9, LOW_SPEED_PCT);
right90();
driveDistance(16, LOW_SPEED_PCT);

// Boarding Pass Station
driveDistance(16,-LOW_SPEED_PCT);
right90();
driveDistance(14,LOW_SPEED_PCT);
driveDistance(5,-LOW_SPEED_PCT);
right90();
driveDistance(36,LOW_SPEED_PCT);
turnRightDegree(45);
driveDistance(4,LOW_SPEED_PCT);
stopDrive();


}