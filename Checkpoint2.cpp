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
void Checkpoint2(){
//Start with robot facing 90 degrees right of being aligned with button
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
    driveDistance(4, -LOW_SPEED_PCT);
    stopDrive();
    turnLeftDegree(182);
    stopDrive();
    driveDistance(8, LOW_SPEED_PCT);
    stopDrive();
    correctToWall();
    stopDrive();
    Sleep(200);
    driveDistance(9.25, -25);
    stopDrive();
    turnRightDegree(90);
    stopDrive();
    while(CDSColor() != CDS_RED && CDSColor() != CDS_BLUE)
    {
        motorLeft.SetPercent(25);
        motorRight.SetPercent(25);
    }
    Sleep(100);
    driveDistance(0.25, 20);
    stopDrive();
    if (CDSColor() == CDS_RED)
    {
        driveDistance(4, -LOW_SPEED_PCT);
        turnRightDegree(90);
        stopDrive();
        driveDistance(10, LOW_SPEED_PCT);
        stopDrive();
    }
    else if (CDSColor() == CDS_BLUE)
    {
        driveDistance(4, -LOW_SPEED_PCT);
        turnRightDegree(90);
        stopDrive();
        driveDistance(3.5, LOW_SPEED_PCT);
        stopDrive();
    }
    turnLeftDegree(87);
    stopDrive();
    driveDistance(4, LOW_SPEED_PCT);
    stopDrive();
    driveDistance(16,-LOW_SPEED_PCT);
    stopDrive();
    turnRightDegree(90);
    stopDrive();
    driveDistance(8,LOW_SPEED_PCT);
    stopDrive();
    correctToWall();
    stopDrive();
    driveDistance(4,-LOW_SPEED_PCT);
    stopDrive();
    turnRightDegree(90);
    stopDrive();
    driveDistance(30,LOW_SPEED_PCT);
    stopDrive();
    
}
#pragma endregion
