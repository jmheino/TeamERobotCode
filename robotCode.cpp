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

void robotCode(){

    //Place robot facing right wall on the 45 degree angle in the starting box

    //Initialize robot function and prep for run
    initRobot();

    robotStart();

    //Drive out and up the ramp
    driveDistance(4.5, LOW_SPEED_PCT);

    stopDrive();

    turnLeftDegree(45);

    stopDrive();

    driveDistance(34, LOW_SPEED_PCT);

    stopDrive();

    right90();

    correctToWall();

    stopDrive();

    #pragma region //Luggage

    driveDistance(18.5, -LOW_SPEED_PCT); //Drive to luggage area

    stopDrive();

    right90();

    stopDrive();

    int luggageTime = TimeNow();
    while(TimeNow() - luggageTime <= 2) //Drive until a 2 second timeout or switch pressed so the robot does not try and overdrive the luggage area
    {
        motorLeft.SetPercent(25);
        motorRight.SetPercent(25);
    }
    stopDrive();

    luggageSequence(); //Deposit luggage and ensure it is in right place

    driveDistance(3, -LOW_SPEED_PCT); //Drive backwards to prep for next task
    stopDrive();

    #pragma endregion


    #pragma region//Buttons

    turnRightDegree(175); //Turn to face north

    findLine(); //Drive until robot hits line

    followToColor(); //Linefollow white until color is found

    int cdsColor = CDSColor(); //Gather the CDS color and store it.

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

    int buttonTime = TimeNow();

    while(TimeNow() - buttonTime <= 2) //Drive until a 2 second timeout or switch pressed so the robot does not try and overdrive the luggage area
    {
        motorLeft.SetPercent(25);
        motorRight.SetPercent(25);
    }

    stopDrive();

    LCD.Clear(BLACK);
    
    #pragma endregion

    // Passport
    Sleep(1.0);
    
    rpsAdjustY = calculateYDistance(59); //Need Actual coord to lever prep

    rpsAdjustX = calculateXDistance(23); //Need Actual coord to lever

    driveDistance(rpsAdjustY, -LOW_SPEED_PCT);

    stopDrive();

    left90();

    turnHeading(180);

    stopDrive();

    driveDistance(rpsAdjustX, -LOW_SPEED_PCT);

    stopDrive();

    turnHeading(180);

    stopDrive();

    passportSequence();


    // Fuel Levers
    turnLeftDegree(45);

    stopDrive();

    turnHeading(235);

    rpsAdjustX = calculateXDistance(5); 

    driveDistance(rpsAdjustX, LOW_SPEED_PCT);  //Distance needs changed

    stopDrive();

    turnRightDegree(55);

    turnHeading(180);

    correctToWall();

    stopDrive();

    driveDistance(3,-LOW_SPEED_PCT);

    stopDrive();

    left90();

    turnHeading(270);

    servoCarrier.SetDegree(5); ////Originally 10

    encoderRight.ResetCounts();
    encoderLeft.ResetCounts();

    float counts = calculateCounts(22);

    int driveTimeout = TimeNow();
    int driveTout = 0;
    while((encoderLeft.Counts() + encoderRight.Counts()) / 2. < counts && driveTout == 0)
    {
    motorRight.SetPercent(25);
    motorLeft.SetPercent(25); 

    if (TimeNow() - driveTimeout > 20)
    {
        driveTout = 1;

        driveDistance(3, -LOW_SPEED_PCT);
        stopDrive();
        turnLeftDegree(35);
        stopDrive();
        driveDistance(3, LOW_SPEED_PCT);
        stopDrive();
    }

    }

    motorRight.SetPercent(0);
    motorLeft.SetPercent(0);


     ////Orignally 22

    stopDrive();
    
    turnRightDegree(45);

    stopDrive();

    servoCarrier.SetDegree(135);

    turnRightDegree(45);

    stopDrive();

    turnHeading(180);

    correctToWall();

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

    // Final Button
    left90();
    zeroCarrier();
    rpsAdjustX = calculateXDistance(16); 
    driveDistance(rpsAdjustX, LOW_SPEED_PCT);
    stopDrive();
    turnRightDegree(45);
    turnHeadingFast(315);
    int endTimeout = TimeNow();

    while(TimeNow() - endTimeout < 10)
    {
    motorLeft.SetPercent(40);
    motorRight.SetPercent(40);
    }
    stopDrive();

    driveDistance(-4, LOW_SPEED_PCT);
    turnLeftDegree(15);

    motorLeft.SetPercent(40);
    motorRight.SetPercent(40);


    
    
}