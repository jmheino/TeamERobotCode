// Required proteus firmware libraries
#include <FEHUtility.h>
#include <FEHLCD.h>
#include <FEHSD.h>
#include <FEHIO.h>
#include <FEHServo.h>
#include <FEHMotor.h>
#include <FEHRPS.h>

// Required personal libraries
#include "include/constants.h"
#include "include/globals.h"
#include "include/functions.h"

#include <cmath>

void competition(){

    //Place robot facing right wall on the 45 degree angle in the starting box

    //Initialize robot function and prep for run
    initRobot();

    robotStart();

    #pragma region //Drive out and up the ramp
    driveDistance(4.5, 40);

    stopDrive();

    turnLeftDegree(45);

    stopDrive();

    driveDistance(1, 35);
    driveDistance(4, MEDIUM_SPEED_PCT);
    driveDistance(8, 35);
    driveDistance(21, MEDIUM_SPEED_PCT);

    stopDrive();

    right90();

    correctToWall();

    stopDrive();
    #pragma endregion

    #pragma region //Luggage

    driveDistance(2, -30);

    driveDistance(16.5, -MEDIUM_SPEED_PCT); //Drive to luggage area

    stopDrive();

    right90();

    stopDrive();

    double luggageTime = TimeNow();
    while(TimeNow() - luggageTime <= 1.25) //Drive until a 2 second timeout or switch pressed so the robot does not try and overdrive the luggage area
    {
        motorLeft.SetPercent(25);
        motorRight.SetPercent(25);
    }
    stopDrive();

    luggageSequence(); //Deposit luggage and ensure it is in right place

    driveDistance(1, -35); //Drive backwards to prep for next task

    stopDrive();

    #pragma endregion

    #pragma region//Buttons

    turnRightDegree(175); //Turn to face north

    findLine(); //Drive until robot hits line

    followToColor(); //Linefollow white until color is found

    
    int cdsColor = CDSColor(); //Gather the CDS color and store it.

    turnRightDegree(15);

    turnHeading(90); //Turn facing north

    driveDistance(2, -LOW_SPEED_PCT); // Back up to make room to maneuver

    right90(); //Turn right and face passport

    //Determine where to drive based on what button needs pressed.
    if(cdsColor == CDS_BLUE) 
    {
        rpsAdjustX = calculateXDistance(16); //Need Actual coord to button blue
    }
    else if (cdsColor == CDS_RED)
    {
        rpsAdjustX = calculateXDistance(22); //Need Actual coord to button red
    }
    else
    {
        rpsAdjustX = calculateXDistance(16);
    }
    driveDistance(rpsAdjustX, LOW_SPEED_PCT);

    stopDrive();

    left90();

    double buttonTime = TimeNow();

    while(TimeNow() - buttonTime <= 0.75) 
    {
        motorLeft.SetPercent(40);
        motorRight.SetPercent(40);
    }

    stopDrive();

    LCD.Clear(BLACK);
    
    #pragma endregion

    #pragma region // Passport    

    servoPassport.SetDegree(PASSPORT_LEVER_UP_PREP);

    rpsAdjustY = calculateYDistance(59); 

    driveDistance(rpsAdjustY, -45);

    stopDrive();

    left90();

    rpsAdjustX = calculateXDistance(24); 

    stopDrive();

    driveDistance(rpsAdjustX, -45);

    stopDrive();

    passportSequenceFast();

    #pragma endregion

    #pragma region // Fuel Levers
    turnLeftDegree(55);

    stopDrive();

    turnHeading(235);

    rpsAdjustX = calculateXDistance(3); //Need Actual coord to lever

    driveDistance(rpsAdjustX, MEDIUM_SPEED_PCT);  //Distance needs changed

    stopDrive();

    turnRightDegree(55);

    correctToWallTimeout();

    stopDrive();

    driveDistance(3,-LOW_SPEED_PCT);

    stopDrive();

    right90();

    servoCarrier.SetDegree(CARRIER_LUGGAGE_DROP);

    rpsAdjustX = calculateXDistance(20);

    driveDistance(rpsAdjustX, -30);
    stopDrive();

    
    servoCarrier.SetDegree(CARRIER_ZERO_POSITION);

    left90();

    turnHeadingFast(180);

    stopDrive();

    correctToWallTimeout();

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
        left90();
        stopDrive();
    }

    stopDrive();

    turnHeading(270);

    leverSequence();

    stopDrive();

    #pragma endregion

    #pragma region // Final Button
    left90();

    zeroCarrier();

    rpsAdjustX = calculateXDistance(16); 

    driveDistance(rpsAdjustX, MEDIUM_SPEED_PCT);

    stopDrive();

    turnRightDegree(45);

    int endTimeout = TimeNow();

    driveDistance(2, 40);

    motorLeft.SetPercent(70);

    motorRight.SetPercent(70);

    #pragma endregion
    
}