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
void Checkpoint3(){
initRobot();
robotStart();
driveDistance(4, LOW_SPEED_PCT);
stopDrive();
turnLeftDegree(45);
stopDrive();
driveDistance(8, LOW_SPEED_PCT);
stopDrive();
turnLeftDegree(92);
stopDrive();
driveDistance(6, -20);
stopDrive();
driveDistance(24, LOW_SPEED_PCT);
stopDrive();
correctToWall();
stopDrive();
driveDistance(2.3, -LOW_SPEED_PCT);
stopDrive();
if (correctLever==2)
{
    LCD.Clear(RED);
    left90();
    stopDrive();
}
else if (correctLever==1)
{
    LCD.Clear(WHITE);
    driveDistance(3.5, -LOW_SPEED_PCT);
    left90();
    stopDrive();
}
else if (correctLever==0)
{
    LCD.Clear(BLUE);
    driveDistance(7, -LOW_SPEED_PCT);
    stopDrive();
    left90();
    stopDrive();
}
else
{
    LCD.Clear(ORANGE);
    stopDrive();
}
// press lever down, wait 5 seconds, bring lever up
stopDrive();
leverSequence();
stopDrive();

}