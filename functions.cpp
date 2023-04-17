// Required proteus firmware libraries
#include <FEHMotor.h>
#include <FEHServo.h>
#include <FEHIO.h>
#include <FEHLCD.h>
#include <FEHRPS.h>
#include <FEHSD.h>
#include <FEHUtility.h>
#include <FEHBattery.h>
#include <FEHSD.h>


// Required personal libraries
#include "include/constants.h"
#include "include/globals.h"
#include "include/functions.h"

// Required C++ libraries
#include <cmath>
#include <string>

//Class
using namespace std;


#pragma region //Robot Starting Functions
void initRobot() {
    //Setup proteus screen
    LCD.Clear(BLACK);
    LCD.SetFontColor(WHITE);

    //Setup encoders
    encoderLeft.ResetCounts();
    encoderRight.ResetCounts();

    //Setup Servos
    servoPassport.SetMin(PASSPORT_SERVO_MIN);
    servoPassport.SetMax(PASSPORT_SERVO_MAX);
    servoCarrier.SetMin(CARRIER_SERVO_MIN);
    servoCarrier.SetMax(CARRIER_SERVO_MAX);

    //Set Servos
    servoCarrier.SetDegree(CARRIER_ZERO_POSITION);
    servoPassport.SetDegree(PASSPORT_ZERO_POSITION);
    

    //Connect to RPS
    RPS.InitializeTouchMenu();

    //Gather Lever
    correctLever = RPS.GetCorrectLever();

}

void robotStart() //Function for robot at start of course
{
    LCD.Clear();
    LCD.WriteLine("Initializing");
    Sleep(3.0);
    //Print waiting to LCD
    LCD.Clear();
    LCD.WriteLine("");
    LCD.WriteLine("Touch screen to begin");
    LCD.WriteLine("");

    float xTouch, yTouch;
    // Wait until screen is touched
    while(!LCD.Touch(&xTouch, &yTouch) ) {};
    // Wait until screen is released
    while( LCD.Touch(&xTouch, &yTouch) ) {};
    LCD.Clear();

    //Gather current time when screen is touched
    unsigned int screenTouchTime = TimeNowSec();

    //Display waiting for timeout
    LCD.WriteLine("Waiting for light/timeout");

    //Wait for light or a 30 second timeout if light is missed
    while ((cdsCell.Value() > 0.9) && (TimeNowSec() < (screenTouchTime + 30)));

    LCD.Clear(YELLOW);
    LCD.WriteLine("Robot has started");

}
#pragma endregion

#pragma region //Calculation functions

float calculateCounts(float distance) //Function for calculating the counts per certain distance
{
float counts = COUNTS_PER_INCH * distance;
return counts;
}

float degreesToCounts(float degree)
{
//Calculate ratio of 360 degrees to turn
float turnRatio = degree/360.0;
float counts = turnRatio * COUNTS_PER_360;

return counts;
}

float calculateHeadingAngle(float x, float y)
{
    float currentDegree = RPS.Heading();
    float currentX = RPS.X();
    float currentY = RPS.Y();
    float xDifference = y - currentY;
    float yDifference = x - currentX;
    float theta = atan(abs(yDifference/xDifference)) * RADIANS_TO_DEGREES;

    if(xDifference > 0 && yDifference > 0) //0-90 Degrees NE Quadrant
    {
        theta = theta + 0;
    }
    else if (xDifference < 0 && yDifference > 0) //90-180 degrees NW Quadrant
    {
        theta = theta + 90;
    }
    else if (xDifference < 0 && yDifference < 0) //180-270 degrees SW Quadrant
    {
        theta = theta + 180;
    }
    else if (xDifference > 0 && yDifference < 0) //270-360 degrees
    {   
        theta = theta + 270;
    }
    else if (xDifference == 0 && yDifference > 0) //facing north
    {
        theta = 90;
    }
    else if (xDifference > 0 && yDifference == 0) //facing east
    {
        theta = 0;
    }
    else if (xDifference == 0 && yDifference < 0) //facing south
    {
        theta = 180;
    }
    else if (xDifference < 0 && yDifference == 0) //Facing west
    {
        theta = 280;
    }

    return (theta);
}   

float calculateXDistance(float coord)
{

    return(abs(RPS.X() - coord));

}

float calculateYDistance(float coord)
{

    return(abs(RPS.Y() - coord));

}
#pragma endregion

#pragma region //PID Calculations
void resetDrivePID() //Function to reset PID variables
{
    //Reset variables
    PIDLastTimeR = TimeNow();
    PIDLastTimeL = TimeNow();
    previousErrorL = 0;
    previousErrorR = 0;
    encoderRight.ResetCounts();
    encoderLeft.ResetCounts();
    timeDifferenceL = 0;
    timeDifferenceR = 0;
    errorSumL = 0;
    errorSumR = 0;
    errorL = 0;
    errorR = 0;
    outputR = 25;
    outputL = 25;
    pTermL = 0;
    pTermR = 0;
    iTermL = 0;
    iTermR = 0;
    dTermL = 0;
    dTermR = 0;
    
    
    //Add delay to pervent 0 time difference
    Sleep(DELAY);
}

float rightPID(float targetSpdR)
{
    //Set current counts based on encoder reading
    currentCountR = encoderRight.Counts();
    //Calculate count difference
    countDifferenceR = currentCountR - previousCountR;

    //Calculate time difference
    timeDifferenceR = TimeNow() - PIDLastTimeR;

    //Calculate linear speed from encoder counts/time difference
    linSpdR = INCHES_PER_COUNT*(countDifferenceR/timeDifferenceR);

    //Calculate error and add that error to current sum
    errorR = targetSpdR - linSpdR;
    errorSumR = errorSumR + errorR;

    //Calculate PID terms
    pTermR = errorR * DRIVE_kP;
    iTermR = errorSumR * DRIVE_kI;
    dTermR = (errorR - previousErrorR) * DRIVE_kD;

    //Store error, counts, and the last time found for later use
    previousErrorR = errorR;
    previousCountR = currentCountR;
    PIDLastTimeR = TimeNow();

    outputR = outputR + pTermR + iTermR + dTermR;
    
    
    
    //Return new motor speed after converting to percent
    return (outputR);
}

float leftPID(float targetSpdL) //PID for left drive motor
{
    //FEHFile *fptr = SD.FOpen("PID_TEST.txt", "w");
    //Set current counts based on encoder reading
    currentCountL = encoderLeft.Counts();

    //Calculate count difference
    countDifferenceL = currentCountL - previousCountL;

    //Calculate time difference
    timeDifferenceL =  TimeNow() - PIDLastTimeL;

    //Calculate linear speed from encoder counts/time difference
    linSpdL = INCHES_PER_COUNT*(countDifferenceL/timeDifferenceL);

    //Calculate error and add that error to current sum
    errorL = targetSpdL - linSpdL;
    errorSumL = errorSumL + errorL;

    //Calculate PID terms
    pTermL = errorL * DRIVE_kP;
    iTermL = errorSumL * DRIVE_kI;
    dTermL = (errorL - previousErrorL) * DRIVE_kD;

    //Store error, counts, and the last time found for later use
    previousErrorL = errorL;
    previousCountL = currentCountL;
    PIDLastTimeL = TimeNow();

    outputL= outputL + pTermL + iTermL + dTermL;

    LCD.Write(" ");
    LCD.Write(pTermL);
    LCD.Write(" ");
    LCD.Write(iTermL);
    LCD.Write(" ");
    LCD.Write(dTermL);
    LCD.Write(" ");
    LCD.WriteLine(outputL);
    //SD.FPrintf(fptr, "%f %f %f %f", pTermL, iTermL, dTermL, outputL);
    //SD.FClose(fptr);
    //Return new motor speed after converting to percent
    return (outputL);    

}

PID::PID()
{
    
    max;
    min;
    kP;
    kI;
    kD;
    preError = 0;
    errorsum = 0;
    lastTime = TimeNow();
}

void PID::setPID(double max, double min, double kP, double kI, double kD)
{
max = max;
min = min;
kP = kP;
kI = kI;
kD = kD;
}

double PID::calculate(double setpoint, double processVariable)
{
    // Calculate error
    double error = setpoint - processVariable;

    // Proportional term
    double pTerm = kP * error;

    // Integral term
    errorsum = errorsum + error;
    double iTerm = kI * errorsum;

    //Calculate Time Difference
    deltaT = TimeNow() - lastTime;

    // Derivative term
    double derivative = (error - preError) / deltaT;
    double dTerm = kD * derivative;

    // Calculate total output
    double output = pTerm + iTerm + dTerm;

    // Restrict to max/min
    if( output > max )
        output = max;
    else if( output < min)
        output = min;

    // Save error to previous error
    preError = error;
    lastTime = TimeNow();

    return output;

}
#pragma endregion

#pragma region //Turning functions
void left90() //Function for left turn 90 no PID
{
    float counts = COUNTS_PER_90;

    encoderRight.ResetCounts();
    encoderLeft.ResetCounts();

    motorRight.SetPercent(TURN_PERCENT);
    motorLeft.SetPercent(-LOW_SPEED_PCT);

    while((encoderLeft.Counts() + encoderRight.Counts()) / 2. < counts);

    motorRight.SetPercent(0);
    motorLeft.SetPercent(0);
}

void right90() //Function for right turn 90 no PID
{
    float counts = COUNTS_PER_90;

    encoderRight.ResetCounts();
    encoderLeft.ResetCounts();

    motorRight.SetPercent(-TURN_PERCENT);
    motorLeft.SetPercent(TURN_PERCENT);
    
    while((encoderLeft.Counts() + encoderRight.Counts()) / 2. < counts);

    motorRight.SetPercent(0);
    motorLeft.SetPercent(0);
}

void turnLeftDegree(float degree)
{
    
    float counts = degreesToCounts(degree);

    encoderRight.ResetCounts();
    encoderLeft.ResetCounts();

    
    while((encoderLeft.Counts() + encoderRight.Counts()) / 2. < counts)
    {
    motorRight.SetPercent(TURN_PERCENT);
    motorLeft.SetPercent(-TURN_PERCENT);
    }

    motorRight.SetPercent(0);
    motorLeft.SetPercent(0);
}

void turnRightDegree(float degree)
{
    
    float counts = degreesToCounts(degree);

    encoderRight.ResetCounts();
    encoderLeft.ResetCounts();

    
    while((encoderLeft.Counts() + encoderRight.Counts()) / 2. < counts)
    {
    motorRight.SetPercent(-TURN_PERCENT);
    motorLeft.SetPercent(TURN_PERCENT);
    }

    motorRight.SetPercent(0);
    motorLeft.SetPercent(0);
}

void turnRightDegreePID(float degree)
{
    resetDrivePID();
    float counts = degreesToCounts(degree);

    while((encoderLeft.Counts() + encoderRight.Counts()) / 2. < counts)
    {
    motorRight.SetPercent(-rightPID(10));
    motorLeft.SetPercent(leftPID(10));

    Sleep(0.1);
    }

    motorRight.SetPercent(0);
    motorLeft.SetPercent(0);
}

void pulseLeft(float percent, float seconds)
{
    // Set both motors to desired percent
    motorRight.SetPercent(percent);
    motorLeft.SetPercent(-percent);

    LCD.Clear(BLACK);
    LCD.WriteLine(seconds);
    // Wait for the correct number of seconds
    Sleep(seconds);

    // Turn off motors
    stopDrive();
}

void pulseRight(float percent, float seconds)
{
    // Set both motors to desired percent
    motorRight.SetPercent(-percent);
    motorLeft.SetPercent(percent);

    // Wait for the correct number of seconds
    Sleep(seconds);

    // Turn off motors
    stopDrive();
}

void pulseForward(float percent, float seconds)
{
    // Set both motors to desired percent
    motorRight.SetPercent(percent);
    motorLeft.SetPercent(percent);

    // Wait for the correct number of seconds
    Sleep(seconds);

    // Turn off motors
    stopDrive();
}

void pulseBackwards(float percent, float seconds)
{
    // Set both motors to desired percent
    motorRight.SetPercent(-percent);
    motorLeft.SetPercent(-percent);

    // Wait for the correct number of seconds
    Sleep(seconds);

    // Turn off motors
    stopDrive();
}

#pragma endregion

#pragma region //Drive functions
void driveDistance(float distance, int speedPercent) //Function to drive a certain distance no PID
{

    encoderRight.ResetCounts();
    encoderLeft.ResetCounts();

    motorRight.SetPercent(speedPercent);
    motorLeft.SetPercent(speedPercent);

    float counts = calculateCounts(distance);

    while((encoderLeft.Counts() + encoderRight.Counts()) / 2. < counts);

    motorRight.SetPercent(0);
    motorLeft.SetPercent(0);

}

void driveDistancePID(float distance, float speed) //Function to drive a certain distance with PID correction
{

    resetDrivePID();

    int counts = calculateCounts(distance);

    while((encoderLeft.Counts() + encoderRight.Counts()) / 2. < counts)
    {
        driveStraightPID(MEDIUM_SPEED_IN_PER_SEC);
    }

    motorRight.SetPercent(0);
    motorLeft.SetPercent(0);

}

void driveStraightPID(double speed)
{

    motorRight.SetPercent(rightPID(speed));
    motorLeft.SetPercent(leftPID(speed));
    Sleep(100);

}

void stopDrive()
{
    motorRight.SetPercent(0);
    motorLeft.SetPercent(0);
    Sleep(0.05);
}

void drive(float leftpct, float rightpct)
{
    motorLeft.SetPercent(leftpct);
    motorRight.SetPercent(rightpct);
}

#pragma endregion

#pragma region //Course correction
void correctToWall()
{
    //Drive forwards while no switches are pressed
    do
    {
    motorRight.SetPercent(LOW_SPEED_PCT);
    motorLeft.SetPercent(LOW_SPEED_PCT);
    } while(LeftMicroswitch.Value() == 1 && RightMicroswitch.Value() == 1);

    motorLeft.SetPercent(0);
    motorRight.SetPercent(0);

    //Check which switch is pressed and correct until other is as well
    do{
        if (LeftMicroswitch.Value()==0)
        {
            motorRight.SetPercent(LOW_SPEED_PCT);
        }
        else if(RightMicroswitch.Value() == 0)
        {
            motorLeft.SetPercent(LOW_SPEED_PCT);
        }
    } while(LeftMicroswitch.Value() == 1 || RightMicroswitch.Value() == 1);

}

void correctToWallTimeout()
{
    double timeout = TimeNow();
    //Drive forwards while no switches are pressed
    do
    {
    motorRight.SetPercent(40);
    motorLeft.SetPercent(40);
    } while(LeftMicroswitch.Value() == 1 && RightMicroswitch.Value() == 1 && TimeNow() - timeout < 2);

    motorLeft.SetPercent(0);
    motorRight.SetPercent(0);

    //Check which switch is pressed and correct until other is as well
    if(TimeNow() - timeout < 2)
    {
    do{
        if (LeftMicroswitch.Value()==0)
        {
            motorRight.SetPercent(40);
        }
        else if(RightMicroswitch.Value() == 0)
        {
            motorLeft.SetPercent(40);
        }
    } while(LeftMicroswitch.Value() == 1 || RightMicroswitch.Value() == 1 );
    }
    else
    {
        return;
    }


}

void faceCoordinate(float x, float y) 
{
    float heading = calculateHeadingAngle(x, y);
    turnHeading(heading);
}

void turnHeading(float heading)
{
    while (RPS.Heading() >= 0 && (RPS.Heading() < heading - HEADING_TOLERANCE || RPS.Heading() > heading + HEADING_TOLERANCE))
        {
            if (RPS.Heading() > heading + HEADING_TOLERANCE)
            {
                pulseRight(PULSE_POWER, PULSE_DELAY);
            }
            else if (RPS.Heading() < heading - HEADING_TOLERANCE)
            {
                pulseLeft(PULSE_POWER, PULSE_DELAY);
            }
            else
            {
                LCD.Clear(ORANGE);
            }

        Sleep(0.1);
        
        getPrintRPS();

        }
}

void turnHeadingFast(float heading)
{
    while (RPS.Heading() >= 0 && (RPS.Heading() < heading - BIG_HEADING_TOLERANCE || RPS.Heading() > heading + BIG_HEADING_TOLERANCE))
        {
            if (RPS.Heading() > heading + BIG_HEADING_TOLERANCE)
            {
                motorLeft.SetPercent(10);
                motorRight.SetPercent(-10);
            }
            else if (RPS.Heading() < heading - BIG_HEADING_TOLERANCE)
            {
                motorLeft.SetPercent(-10);
                motorRight.SetPercent(10);
            }
            else
            {
                LCD.Clear(ORANGE);
            }
        
        }
        stopDrive();
}


void turnHeadingPID(float heading)
{
    PID turnPID; 

    turnPID.setPID(-100,100,0.75,0,0);

    if (abs(RPS.Heading() - heading) > 1)
    {
    float offset = turnPID.calculate(RPS.Heading(), heading);
    drive(offset, -offset);
    }
}

void followToColor()
{
    linefollowWhite();
    LCD.Clear(YELLOW);
    turnLeftDegree(15);
    driveDistance(2.5, 20);
    stopDrive();
}

void findLine()
{
    while(MOpto.Value() >= BACKGROUND_OPTO_VALUE)
    {
        motorLeft.SetPercent(30);
        motorRight.SetPercent(20);
    }
    stopDrive();
    LCD.Clear(WHITE);

    while(ROpto.Value() >= BACKGROUND_OPTO_VALUE)
    {
        motorRight.SetPercent(10);
        motorLeft.SetPercent(-10);
    }

    return;
}
#pragma endregion

#pragma region //Testing and reading values

float readCDS()
{
    float x = cdsCell.Value();
    LCD.WriteLine(x);
    return(x);
}

void linefollowWhite()
{
    bool lineFollow = true;

    while(lineFollow == true)
    {
    if(MOpto.Value() <= BACKGROUND_OPTO_VALUE && LOpto.Value() <= BACKGROUND_OPTO_VALUE)
    {
        motorRight.SetPercent(25);
        motorLeft.SetPercent(15);
    }
    else if(MOpto.Value() <= BACKGROUND_OPTO_VALUE && ROpto.Value() <= BACKGROUND_OPTO_VALUE)
    {
        motorRight.SetPercent(15);
        motorLeft.SetPercent(25);
    }
    else if (LOpto.Value() <= BACKGROUND_OPTO_VALUE)
    {
        motorRight.SetPercent(10);
        motorLeft.SetPercent(0);
    }
    else if(ROpto.Value() <= BACKGROUND_OPTO_VALUE)
    {
        motorRight.SetPercent(0);
        motorLeft.SetPercent(10);
    }
    else if(MOpto.Value() <= BACKGROUND_OPTO_VALUE)
    {
        motorRight.SetPercent(25);
        motorLeft.SetPercent(25);
    }
    else
    {
        motorRight.SetPercent(0);
        motorLeft.SetPercent(0);
        lineFollow = false;
    }
    
    }
}

int CDSColor()
{
    if (cdsCell.Value() <=  0.9)
    {
        LCD.Clear(RED);
        return(CDS_RED);
    }
    else if (cdsCell.Value() < 2.4 && cdsCell.Value() > 0.9)
    {
        LCD.Clear(BLUE);
        return(CDS_BLUE);
    }
    else if  (cdsCell.Value() >= 2.4)
    {
        LCD.Clear(GREEN);
        return (CDS_NO_LIGHT);

    }
    else
    {
        return (CDS_NO_LIGHT);
    }
}

void speedTest()
{
    int distance = 6;
    float speedPercent = 0;
    float counts = calculateCounts(distance);
    float degreecounts = degreesToCounts(180);
    float LastTime;
    LCD.Clear();

    for(int i=2; i<=20 ; i++)
    {
    //Clear screen and wait for touch to start test
    LCD.WriteLine("");
    LCD.WriteLine("Waiting for touch");
    LCD.WriteLine("Test #:");
    LCD.WriteLine(i);
    LCD.WriteLine("");

    float xTouch, yTouch;
    // Wait until screen is touched
    while( !LCD.Touch(&xTouch, &yTouch) ) {};
    // Wait until screen is released
    while( LCD.Touch(&xTouch, &yTouch) ) {};
    LCD.Clear();
    Sleep(3);

    //Take values for calculations and calculate necessary percent
    LastTime = TimeNow();
    encoderRight.ResetCounts();
    encoderLeft.ResetCounts();
    speedPercent = i * 5;

    motorRight.SetPercent(speedPercent);
    motorLeft.SetPercent(speedPercent);

    //Drive forward for specified distance
    while((encoderLeft.Counts() + encoderRight.Counts()) / 2. < counts);
    

    motorRight.SetPercent(0);
    motorLeft.SetPercent(0);


    //Calculate linear speeds and write to screen
    float linSpdR = (encoderRight.Counts()/(TimeNow()-LastTime));
    float linSpdL = (encoderLeft.Counts()/(TimeNow()-LastTime));
    LCD.WriteLine("Test #:");
    LCD.WriteLine(i-1);
    LCD.WriteLine("Distance:");
    LCD.WriteLine(encoderLeft.Counts() + encoderRight.Counts());
    LCD.WriteLine("Percent:");
    LCD.WriteLine(speedPercent);
    LCD.WriteLine("Right Speed:");
    LCD.WriteLine(linSpdR);
    LCD.WriteLine("Left Speed:");
    LCD.WriteLine(linSpdL);

    Sleep(3);

    encoderRight.ResetCounts();
    encoderLeft.ResetCounts();

    

    motorRight.SetPercent(0);
    motorLeft.SetPercent(0);

    }

}

void servoTest()
{
    LCD.Clear(BLACK);
    servoPassport.SetDegree(0);
    for(int i=0; i<=36 ; i++)
    {
    //Clear screen and wait for touch to start test
    LCD.WriteLine("");
    LCD.WriteLine("Waiting for touch");
    LCD.WriteLine("Test Angle");
    LCD.WriteLine(i*5);
    LCD.WriteLine("");

    float xTouch, yTouch;
    // Wait until screen is touched
    while( !LCD.Touch(&xTouch, &yTouch) ) {};
    // Wait until screen is released
    while( LCD.Touch(&xTouch, &yTouch) ) {};
    LCD.Clear();
    Sleep(1.0);

    int angle = i*5;
    servoPassport.SetDegree(angle);
    }

    LCD.Clear(WHITE);
    LCD.SetFontColor(BLACK);

    for(int i=36; i>=0 ; i--)
    {
    //Clear screen and wait for touch to start test
    LCD.WriteLine("");
    LCD.WriteLine("Waiting for touch");
    LCD.WriteLine("Test Angle");
    LCD.WriteLine(i*5);
    LCD.WriteLine("");

    float xTouch, yTouch;
    // Wait until screen is touched
    while( !LCD.Touch(&xTouch, &yTouch) ) {};
    // Wait until screen is released
    while( LCD.Touch(&xTouch, &yTouch) ) {};
    LCD.Clear();
    Sleep(1.0);

    int angle = i*5;
    servoPassport.SetDegree(angle);
    }
}

void getPrintRPS()
{
        LCD.Clear();
        LCD.WriteLine("Heading:");
        LCD.WriteLine(RPS.Heading());
        LCD.WriteLine("X:");
        LCD.WriteLine(RPS.X());
        LCD.WriteLine("Y:");
        LCD.WriteLine(RPS.Y());
   

}

void readOptosensor()
{
    while(1)
    {
        LCD.Clear(BLACK);
        LCD.WriteLine("Left Opto:");
        LCD.WriteLine(LOpto.Value());
        LCD.WriteLine("Middle Opto:");
        LCD.WriteLine(MOpto.Value());
        LCD.WriteLine("Right Opto:");
        LCD.WriteLine(ROpto.Value());
        Sleep(1.0);
    }
}

#pragma endregion

#pragma region //Servos

void flipLeverDown()
{
    servoCarrier.SetDegree(CARRIER_LEVER_DOWN_ANGLE);
}

void flipLeverUp()
{
    servoCarrier.SetDegree(CARRIER_LEVER_UP_ANGLE);
    
}

void zeroCarrier()
{
    servoCarrier.SetDegree(CARRIER_ZERO_POSITION);

}

void zeroPassport()
{
    servoPassport.SetDegree(PASSPORT_ZERO_POSITION);
}

void leverSequence()
{
    servoCarrier.SetDegree(SET_TO_FLIP_LEVER_DOWN);
    while (RPS.Y() > 22 || RPS.Y() < 21.5)
    {
    if(RPS.Y() > 22)
    {
        motorLeft.SetPercent(15);
        motorRight.SetPercent(15);
    }
    else if (RPS.Y() < 22)
    {
    
        motorLeft.SetPercent(-15);
        motorRight.SetPercent(-15);
        
    }
    }
    stopDrive();
    servoCarrier.SetDegree(CARRIER_LEVER_DOWN_ANGLE);
    Sleep(100);
    driveDistance(3, -LOW_SPEED_PCT);
    servoCarrier.SetDegree(SET_TO_FLIP_LEVER_UP);
    driveDistance(3, LOW_SPEED_PCT);
    Sleep(5.0);
    servoCarrier.SetDegree(CARRIER_LEVER_UP_ANGLE);
    Sleep(100);
    driveDistance(4, -LOW_SPEED_PCT);
}

void passportSequence()
{
    servoPassport.SetDegree(PASSPORT_LEVER_UP_PREP);
    Sleep(500);
    int passportStartTime = TimeNow();
    while(TimeNow() - passportStartTime < 2)
    {
        motorLeft.SetPercent(-25);
        motorRight.SetPercent(-25);
    }
    stopDrive();
    servoPassport.SetDegree(PASSPORT_LEVER_UP);
    Sleep(500);
    driveDistance(2,LOW_SPEED_PCT);
    stopDrive();
    servoPassport.SetDegree(PASSPORT_LEVER_DOWN_PREP);
    Sleep(500);
    driveDistance(2,-LOW_SPEED_PCT);
    stopDrive();
    servoPassport.SetDegree(PASSPORT_LEVER_DOWN);
    zeroPassport();
    driveDistance(3, LOW_SPEED_PCT);
    
}

void passportSequenceFast()
{

    int passportStartTime = TimeNow();
    while(TimeNow() - passportStartTime < 1.0)
    {
        motorLeft.SetPercent(-25);
        motorRight.SetPercent(-25);
    }
    stopDrive();
    servoPassport.SetDegree(PASSPORT_LEVER_UP);
    Sleep(600);
    driveDistance(2,LOW_SPEED_PCT);
    stopDrive();
    servoPassport.SetDegree(PASSPORT_LEVER_DOWN_PREP);
    Sleep(200);
    driveDistance(2,-LOW_SPEED_PCT);
    stopDrive();
    servoPassport.SetDegree(50);
    driveDistance(3, LOW_SPEED_PCT);
    
}

void luggageSequence()
{
    servoCarrier.SetDegree(CARRIER_LUGGAGE_DROP);
    stopDrive();
    Sleep(500);
    servoCarrier.SetDegree(CARRIER_ZERO_POSITION);
} 


#pragma endregion

#pragma region //Random

int scanForColor()
{
    int scannedColor = CDS_NO_LIGHT;
    int degreeTotal = 0;
    do
    {
        for(int i=0; i<6; 1)
        {
            driveDistance(0.5, VERY_LOW_SPEED_PCT);
            if (CDSColor() != CDS_NO_LIGHT)
            {
                scannedColor = CDSColor();
                break;
            }
        }
        driveDistance(3, -LOW_SPEED_PCT);
        turnLeftDegree(5);
    } while  (scannedColor != CDS_RED || scannedColor != CDS_BLUE);
    

    return scannedColor;
}



#pragma endregion