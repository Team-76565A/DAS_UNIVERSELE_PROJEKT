#include "pros/adi.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.hpp"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include <iostream>

using namespace std;
using namespace pros;

//conversion von degrees zu Turns
float degConvertorTurn(float toHeading, ADIGyro gyro)
{
    float headingOffset;
    float Turns;

    //constante für die turn Pro degree
    float Kt = 0.03;

    headingOffset = gyro.get_value() - toHeading;
    Turns = headingOffset * Kt;
    
    return Turns;
}