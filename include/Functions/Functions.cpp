#include "pros/llemu.hpp"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include <iostream>

using namespace std;
using namespace pros;


int convertUnits(int parameter, string fromUnit, string toUnit, float CircumferenceInCm = 31.91)
{
    if (fromUnit == "cm")
    {
        if (toUnit == "rotations")
        {
            return parameter/CircumferenceInCm;
        }
        else if (toUnit == "degress") 
        {
            return (parameter/CircumferenceInCm)*360;
        }
    }
    else if (fromUnit == "inches") 
    {

        if (toUnit == "cm")
        {
            return parameter*2.54;
        }
        else if (toUnit == "rotations")
        {
            return convertUnits(parameter, "inches", "cm")/CircumferenceInCm;
        }
        else if (toUnit == "degress") 
        {
            return (convertUnits(parameter, "inches", "cm")/CircumferenceInCm)*360;
        }
    }

    return 1;
}




/*void MoveCentimeter(pros::Motor_Group i, float centimeter, int velocity)
{


    //i.move_relative(convertUnits(centimeter, "cm,", "rotations"), velocity);
    
}
*/