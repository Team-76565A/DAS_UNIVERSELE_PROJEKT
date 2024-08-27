#include "pros/adi.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.hpp"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include <iostream>

using namespace std;
using namespace pros;

/**
 * Converts a heading in degrees to the equivalent number of motor turns.
 * 
 * @param toHeading The desired heading in degrees.
 * @param gyro The ADIGyro sensor object used to measure the current heading.
 * @return float The number of motor turns needed to achieve the desired heading.
 */
float degConvertorTurn(float toHeading, ADIGyro& gyro) {
    // Constant for turns per degree
    const float Kt = 0.03;

    // Calculate heading offset
    float headingOffset = gyro.get_value() - toHeading;

    // Convert offset to motor turns
    return headingOffset * Kt;
}
