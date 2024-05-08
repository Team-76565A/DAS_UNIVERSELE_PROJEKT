#include "pros/llemu.hpp"
#include "pros/motors.h"
#include "pros/motors.hpp"
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




//int moveCentimeter(Motor_Group Drive, float centimeter, int velocity)
//{


    //Drive.move_relative(convertUnits(centimeter, "cm,", "rotations"), velocity);

    /*Drive.move_relative(const double position, const std::int32_t velocity)*/

    /*
    *   Does not work => it only work's with normal pros::Motor not with pros::Motor_Group
    *
    */
    /*
    if (Motor.get_encoder_units() == pros::E_MOTOR_ENCODER_ROTATIONS) 
    {
    Motor.move_relative(convertUnits(centimeter, "cm,", "rotations"), velocity);
    return 0;
    }
    else if (Motor.get_encoder_units() == pros::E_MOTOR_ENCODER_DEGREES) 
    {
        Motor.move_relative(convertUnits(centimeter, "cm,", "degress"), velocity);
        return 0;
    }
    */
    //return 1;
    
//}