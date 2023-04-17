// Required proteus firmware libraries
#include <FEHUtility.h>
#include <FEHLCD.h>
#include <FEHSD.h>
#include <FEHIO.h>
#include <FEHServo.h>
#include <FEHMotor.h>
#include <FEHRPS.h>
#include <FEHBattery.h>

// Required personal libraries
#include "include/constants.h"
#include "include/globals.h"
#include "include/functions.h"

void test()
{
    RPS.InitializeTouchMenu();

    while(1)
    {
        
        LCD.WriteLine(" ");
        LCD.Write("CDS VALUE:");
        float CDS = readCDS();
        LCD.WriteLine(CDS);
        LCD.Write("CDS COLOR:");
        LCD.WriteLine(CDSColor());
        LCD.WriteLine("Left Opto:");
        LCD.WriteLine(LOpto.Value());
        LCD.WriteLine("Middle Opto:");
        LCD.WriteLine(MOpto.Value());
        LCD.WriteLine("Right Opto:");
        LCD.WriteLine(ROpto.Value());
        LCD.WriteLine("");
        LCD.Write("HEADING:");
        LCD.WriteLine(RPS.Heading());
        LCD.Write("X:");
        LCD.WriteLine(RPS.X());
        LCD.Write("Y:");
        LCD.WriteLine(RPS.Y());
        Sleep(0.5);
        LCD.Clear();
    }
}