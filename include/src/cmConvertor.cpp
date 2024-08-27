#include "pros/llemu.hpp"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include <iostream>

using namespace std;
using namespace pros;

/**
 * Converts units between different measurements.
 * 
 * @param parameter The value to convert.
 * @param fromUnit The current unit of the parameter (e.g., "cm", "inches").
 * @param toUnit The desired unit after conversion (e.g., "rotations", "degrees").
 * @param CircumferenceInCm The circumference of the wheel in centimeters (default is 31.91 cm).
 * @return float The converted value.
 */
float convertUnits(int parameter, const string& fromUnit, const string& toUnit, float CircumferenceInCm = 31.91) {
    if (fromUnit == "cm") {
        if (toUnit == "rotations") {
            return parameter / CircumferenceInCm;
        } else if (toUnit == "degrees") {
            return (parameter / CircumferenceInCm) * 360;
        }
    } else if (fromUnit == "inches") {
        if (toUnit == "cm") {
            return parameter * 2.54;
        } else if (toUnit == "rotations") {
            return convertUnits(parameter, "inches", "cm", CircumferenceInCm) / CircumferenceInCm;
        } else if (toUnit == "degrees") {
            return (convertUnits(parameter, "inches", "cm", CircumferenceInCm) / CircumferenceInCm) * 360;
        }
    }

    return 1; // Default return value if no valid conversion is found
}

/* Example function for moving a motor group by a certain distance
void MoveCentimeter(pros::Motor_Group group, float centimeter, int velocity) {
    group.move_relative(convertUnits(centimeter, "cm", "rotations"), velocity);
}
*/
