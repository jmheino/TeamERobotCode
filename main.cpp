#pragma region

#include <FEHLCD.h>
#include <FEHUtility.h>
#include <FEHMotor.h>
#include <FEHRPS.h>
//Include Libraries Here
#include "include/constants.h"
#include "include/functions.h"
#include "include/globals.h"


int main(void)
{

   int state = 0;

   while(1)
   {
     if (state == 0) //Menu
        {
            state = menu();   
        }
        else if (state == 1) // Slow
        {
            robotCode();
        }
        else if (state == 2) // Fast
        {
            competition();
        }
        else if(state == 3) //Instructions
        {
            test();

           
        }
   }

