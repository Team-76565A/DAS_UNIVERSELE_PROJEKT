#include "pros/adi.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.hpp"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include <array>
#include <iostream>

using namespace std;
using namespace pros;


float turnToHeading(float toHeading, ADIGyro gyro, Controller controller, Motor Left1, Motor Left2, Motor Right1, Motor Right2) {

    Motor_Group RightSide({Right1, Right2});
    Motor_Group LeftSide({Left1, Left2});

    float TurnSpeed = 0;
    float maxiTurnSpeed = 200;
    float Kp = 0.52;
    float currentHeading;
    float HeadingOffset;
    controller.rumble(".--.");
    while(gyro.get_value() < toHeading - 0.8 || gyro.get_value() > toHeading + 0.8)
    {
        currentHeading = gyro.get_value();
        HeadingOffset = currentHeading - toHeading;
        if(HeadingOffset > 0 && HeadingOffset < 181)
        {
            if(HeadingOffset < 80)
            {
                TurnSpeed = HeadingOffset * Kp;
            } else if(HeadingOffset >= 80) {
                TurnSpeed = maxiTurnSpeed;
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
    
    }
    controller.rumble("---");
    return TurnSpeed;
}
