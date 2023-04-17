// Required proteus firmware libraries
#include <FEHMotor.h>
#include <FEHServo.h>
#include <FEHIO.h>
#include <FEHSD.h>

// Other libraries
#include "include/constants.h"
#include "include/globals.h"

using namespace std;

// Motor definitions
FEHMotor motorLeft(MOTOR_PORT_L, MOTOR_VOLTS);
FEHMotor motorRight(MOTOR_PORT_R, MOTOR_VOLTS);

// Servo definitions
FEHServo servoPassport(SERVO_PORT_PASSPORT);
FEHServo servoCarrier(SERVO_PORT_CARRIER);


// Sensor definitions
AnalogInputPin cdsCell(CDS_CELL_PORT);
DigitalInputPin LeftMicroswitch(L_MICROSWITCH_PORT);
DigitalInputPin RightMicroswitch(R_MICROSWITCH_PORT);
DigitalEncoder encoderLeft(ENCODER_LEFT_PORT);
DigitalEncoder encoderRight(ENCODER_RIGHT_PORT);
AnalogInputPin LOpto(FEHIO::P2_3);
AnalogInputPin MOpto(FEHIO::P2_0);
AnalogInputPin ROpto(FEHIO::P1_6);


// RPS adjustment values
 float rpsAdjustX;
 float rpsAdjustY;

// Error detection and reaction values
 int encoderErrors;
 bool encodersEnabled;
 int rpsErrors;
 bool rpsEnabled;

//Global ints
 float distance;
 int correctLever;

//Global floats
 int currentTime;
 float headingDegree;
 float xRPS, yRPS;


//Global ints for PID
 float PIDLastTimeR, PIDLastTimeL;
 float currentCountR;
 float previousCountR;
 float countDifferenceR;
 float currentCountL;
 float previousCountL;
 float countDifferenceL;
 float timeDifferenceR, timeDifferenceL;



//Global floats for PID
 float targetSpdL;
 float targetSpdR;
 float errorR, errorL;
 float errorSumR, errorSumL;
 float pTermL, pTermR, iTermL, iTermR, dTermL, dTermR;
 float previousErrorL, previousErrorR;
 float linSpdR;
 float linSpdL;
 float outputR, outputL;


//Data logging
//FEHFile *fptr = SD.FOpen("Robot_Log.txt", "w");
