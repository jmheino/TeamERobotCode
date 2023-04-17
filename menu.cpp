// Required proteus firmware libraries
#include <FEHUtility.h>
#include <FEHLCD.h>
#include <FEHSD.h>
#include <FEHIO.h>
#include <FEHServo.h>
#include <FEHMotor.h>
#include <FEHBattery.h>

// Required personal libraries
#include "include/constants.h"
#include "include/globals.h"
#include "include/functions.h"

int menu()
{
    int state;
    int input = 0;
    float x, y;

    //Clear screen and make it black
    LCD.Clear(BLACK);

    // declare an array of four icons called menu
    FEHIcon::Icon menu[3];
 
    // define the four menu labels
    char menu_labels[3][20] = {"SLOW CODE","FAST CODE", "TEST"};
 
    FEHIcon::DrawIconArray(menu, 3, 1, 30, 10, 5, 5, menu_labels, WHITE, WHITE);
    LCD.Write("BATTERY:");
    LCD.WriteLine(Battery.Voltage());

    while(input == 0) 
    {
        

        if (LCD.Touch(&x, &y))
        {
            for (int n = 0; n < 3; n++)
            {
                if (menu[n].Pressed(x,y,0))
                {
                    menu[n].Select();
                    state = n+1;
                    Sleep(100);
                    menu[n].Deselect();
                    input = 1;
                }
            }
        }
        Sleep(100);
    }
    
    return state;
}