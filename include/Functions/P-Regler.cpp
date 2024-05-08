#include "pros/adi.hpp"
#include "pros/llemu.hpp"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include <iostream>

using namespace std;
using namespace pros;

void turnToHeading(float toHeading, ADIGyro gyro) {
    while (true) {
        float currentHeading = gyro.get_value();
        float offsetHeading = currentHeading - toHeading;
        float maxiTurnSpeed = 200;
        if(HeadingOffset > 0 && HeadingOffset < 181)
        {
        if(HeadingOffset < 80)
        {
            TurnSpeed = HeadingOffset * Kp;
        } else if(HeadingOffset >= 80) {
            TurnSpeed = maxiTurnSpeed;
        }
        }else if (HeadingOffset < 0 && HeadingOffset > -181) {
        if(HeadingOffset > -80)
        {
            TurnSpeed = HeadingOffset * Kp;
        } else if(HeadingOffset <= -80) {
            TurnSpeed = -maxiTurnSpeed;
        }
        } else if(HeadingOffset > 180)  {
        if(HeadingOffset < 280)
        {
            TurnSpeed = -maxiTurnSpeed;
        } else if(HeadingOffset >= 280) {
            TurnSpeed = (-360 + HeadingOffset) * Kp;
        }
        } else if(HeadingOffset  < -180){
        if (HeadingOffset < -280)
        {
            TurnSpeed = (360 + HeadingOffset) * Kp;
        } else if(HeadingOffset >= -280) {
            TurnSpeed = maxiTurnSpeed;
        }
        
        }
       
    }

    controller.rumble("---");
    //Drive.resetPosition();
    controller.Screen.clearScreen();
    controller.Screen.setCursor(1,1);
    controller.Screen.print("%f", Gyro.heading(degrees));
    return;
    }


}