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

/*
* Ein P-Regler
*/
/*float altVersion(float toHeading, ADIGyro gyro, Controller controller, Motor Left1, Motor Left2, Motor Right1, Motor Right2) {

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
}*/

int turnToHeading(float toHeading, ADIGyro gyro, Controller controller, Motor Left1, Motor Left2, Motor Left3, Motor Right1, Motor Right2, Motor Right3){

    Motor_Group RightSide({Right1, Right2, Right3});
    Motor_Group LeftSide({Left1, Left2, Right3});


    float turnSpeed = 0;
    float maxTurnSpeed = 200;
    float currentHeading = 0;
    float error = 0;
    float integral = 0;
    float derivative = 0;

    float last_error = 0;

    //Mit Print drin
    /*float kp = 0.92;
    float ki = 0.000013;
    float kd = 0.005;*/

    //ohne Print
    float kp = 0.87;
    float ki = 0.0000025;
    float kd = 0.005;


    while(!(currentHeading <= toHeading + 0.8 && currentHeading >= toHeading - 0.8))
    {
        currentHeading = gyro.get_value()/10;
        error = currentHeading - toHeading;
        integral += error;
        derivative = error - last_error;
        turnSpeed = (kp*error) + (ki*integral) + (kd*derivative);
        
        if (turnSpeed >= maxTurnSpeed) {
            turnSpeed = maxTurnSpeed;
        } else if (turnSpeed <= -maxTurnSpeed) {
            turnSpeed = -maxTurnSpeed;
        }
        
        if(turnSpeed < 0){
            RightSide.move_velocity(turnSpeed);
            LeftSide.move_velocity(-turnSpeed);
        } else if(turnSpeed > 0)
        {
            RightSide.move_velocity(turnSpeed);
            LeftSide.move_velocity(-turnSpeed);
        } else
        {
            RightSide.brake();
            LeftSide.brake();
        }
        
    }
    RightSide.brake();
    LeftSide.brake();
    return 0;
}


