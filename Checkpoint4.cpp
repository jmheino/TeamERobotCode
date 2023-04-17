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


#pragma region
void Checkpoint4(){
    
    initRobot();
    robotStart();
    driveDistance(4, LOW_SPEED_PCT);
    stopDrive();
    turnLeftDegree(45);
    stopDrive();
    driveDistance(32, LOW_SPEED_PCT);
    stopDrive();
    right90();
    stopDrive();
    correctToWall();
    stopDrive();
    driveDistance(12, -LOW_SPEED_PCT);
    stopDrive();
    left90();
    stopDrive();
    driveDistance(14.5, LOW_SPEED_PCT);
    stopDrive();
    left90();
    passportSequence();

}