#ifndef FUNCTIONS_H
#define FUNCTIONS_H

//Main robot code function
void robotCode(); 
//Code for checkpoint 1
void Checkpoint1(); 
//Code for checkpoint 2
void Checkpoint2(); 
//Code for checkpoint 3
void Checkpoint3();
//Code for checkpoint 4
void Checkpoint4();
//Code for checkpoint 5
void Checkpoint5();


//Initialization of the robot
void initRobot(); 

//Calculate counts per distance (Distance)
float calculateCounts(float); 

//Turn left 90
void left90();
//Turn right
void right90();
//Turn left for a set degree (degree)
void turnLeftDegree(float);
//Turn right for a set degree (degree)
void turnRightDegree(float);
//Turn right for a set degree (degree)
void turnRightDegreePID(float);  
//Pulse left (power, delay in sec)
void pulseLeft(float, float);
//Pulse right (power, delay in sec) 
void pulseRight(float, float); 

void pulseForward(float, float);

void pulseBackwards(float, float);

//Drive distance function (distance, power)
void driveDistance(float, int); 
//Drive stright with PID correction
void driveStraightPID(double); 

//Robot wait for starting light function
void robotStart(); 

//Robot drives straight with PID (distance, power inches per sec)
void driveDistancePID(float, float); 

//Reset PID variables (speed inches per sec)
void resetPID(float); 
//PID for right motor (speed inches per sec)
float rightPID(float); 
//PID for left motor (speed inches per sec)
float leftPID(float);  

//Convert right motor percent to linear speed (percent)
float motorPercentToLinearSpeedR(float); 
//Convert left motor percent to linear speed (percent)
float motorPercentToLinearSpeedL(float); 
//Convert right linear speed to motor percent (speed inches per sec)
float motorLinearSpeedToPercentR(float);
//Convert left linear speed to motor percent (speed inches per sec)
float motorLinearSpeedToPercentL(float); 

//Calculate counts based on degree for turning (degree)
float degreesToCounts(float); 

//Wall correction with microswitches
void correctToWall(); 

//Stop driving
void stopDrive(); 

//Read CDS Cell values
float readCDS(); 

//Test to determine counts per second for PID motor curve
void speedTest(); 

//Face an RPS coordinate (desired x, desired y)
void faceCoordinate(float, float); 
//Turn to a specific heading based on RPS (heading)
void turnHeading(float); 
//Calculate angle to heading based on rps and output as a float (desired x, desired y)
float calculateHeadingAngle(float, float); 
//Step to the light by driving forward then cirrecting angle
void stepToLight(); 
//Read the CDS cell and output the color as an int
int CDSColor(); 
//Drive to specified color button
void driveToColorButton(int);

//Prints RPS x, y, and heading to screen
void getPrintRPS();

// Drive to each button
void driveRed();
void driveBlue();

//Line following on white line
void linefollowWhite();
//scan for color and return color as int
int scanForColor();

//Function for random testing
void test();

//Proteus launch menu
int menu();


void resetDrivePID();

//Turn heading with PID correction
void turnHeadingPID(float);

void drive(float, float);

//Function to run servo to flip lever down
void flipLeverDown();

//Function to run servo to flip lever up
void flipLeverUp();

//Zero all robot subsystems and their starting positions
void zeroRobot();

void zeroCarrier();

void zeroPassport();

//Lever sequence code
void leverSequence();

// Servo
void servoTest();

//Passport
void passportSequence();

// Luggage
void luggageSequence();

// Line Following
void readOptosensor();

float calculateXDistance(float);

float calculateYDistance(float);

void followToColor();

void findLine();

void competition();

void passportSequenceFast();

void turnHeadingFast(float);

void correctToWallTimeout();

class PID
{
    public:
        //PID control definition
        PID();
        //Set variables (max, min, kP, kI, kD)
        void setPID(double, double, double, double, double);
        //Calculate PID (setpoint, process variable)
        double calculate(double, double);
        
    private:
        double deltaT;
        double max;
        double min;
        double kP;
        double kI;
        double kD;
        double preError;
        double errorsum;
        double lastTime;
};

#endif