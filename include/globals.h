#ifndef GLOBALS_H
#define GLOBALS_H

#include "FEHIO.h"
#include "FEHMotor.h"
#include "FEHServo.h"

using namespace std;

// Motor definitions
extern FEHMotor motorLeft;
extern FEHMotor motorRight;

// Servo definitions
extern FEHServo servoPassport;
extern FEHServo servoCarrier;


// Sensor definitions
extern AnalogInputPin cdsCell;
extern DigitalInputPin LeftMicroswitch;
extern DigitalInputPin RightMicroswitch;
extern DigitalEncoder encoderLeft;
extern DigitalEncoder encoderRight;
extern AnalogInputPin LOpto;
extern AnalogInputPin MOpto;
extern AnalogInputPin ROpto;

// RPS adjustment values
extern float rpsAdjustX;
extern float rpsAdjustY;

// Error detection and reaction values
extern int encoderErrors;
extern bool encodersEnabled;
extern int rpsErrors;
extern bool rpsEnabled;

//Global ints
extern float distance;
extern int correctLever;

//Global floats
extern int currentTime;
extern float headingDegree;
extern float xRPS, yRPS;


//Global ints for PID
extern float PIDLastTimeR, PIDLastTimeL;
extern float currentCountR;
extern float previousCountR;
extern float countDifferenceR;
extern float currentCountL;
extern float previousCountL;
extern float countDifferenceL;
extern float timeDifferenceR, timeDifferenceL;



//Global floats for PID
extern float targetSpdL;
extern float targetSpdR;
extern float errorR, errorL;
extern float errorSumR, errorSumL;
extern float pTermL, pTermR, iTermL, iTermR, dTermL, dTermR;
extern float previousErrorL, previousErrorR;
extern float linSpdR;
extern float linSpdL;
extern float outputR;
extern float outputL;



#endif