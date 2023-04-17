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
void Checkpoint5(){
    
    initRobot();
    robotStart();
    driveDistance(4, LOW_SPEED_PCT);
    stopDrive();
    //turnLeftDegree(45);
    turnHeading(90);
    stopDrive();
    driveDistance(34, LOW_SPEED_PCT);
    stopDrive();
    right90();
    correctToWall();
    stopDrive();
    driveDistance(18.5, -LOW_SPEED_PCT);
    stopDrive();
    right90();
    stopDrive();
    int starttime = TimeNow();
    while(TimeNow() - starttime <= 2)
    {
        motorLeft.SetPercent(25);
        motorRight.SetPercent(25);
    }
    luggageSequence();
    
    

    
}