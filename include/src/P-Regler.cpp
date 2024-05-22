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

void testfunktion(float toHeading, ADIGyro gyro, Controller controller, Motor Left1, Motor Left2, Motor Right1, Motor Right2){

    Motor_Group RightSide({Right1, Right2});
    Motor_Group LeftSide({Left1, Left2});


    float turnSpeed = 0;
    float maxTurnSpeed = 200;
    float currentHeading;
    float error;
    float integral;
    float derivative;

    float last_error;

    float kp = 0.52;
    float ki = 0.5;
    float kd = 0.5;

    while (gyro.get_value() - toHeading != 0) {
        currentHeading = gyro.get_value();
        error = currentHeading - toHeading;
        integral += error;
        derivative = error - last_error;


        turnSpeed = (kp*error) + (ki*integral) + (kd*derivative);

        if (turnSpeed >= maxTurnSpeed) {
            turnSpeed = maxTurnSpeed;
        } else if (turnSpeed <= -maxTurnSpeed) {
            turnSpeed = -maxTurnSpeed;
        }

        if(turnSpeed > 0)
        {
            RightSide.move_velocity(-turnSpeed);
            LeftSide.move_velocity(turnSpeed);
        }   else if(turnSpeed < 0){
            RightSide.move_velocity(turnSpeed);
            LeftSide.move_velocity(-turnSpeed);
        }   else {
            RightSide.brake();
            LeftSide.brake();
        }
        
        last_error = error;

        
    }  
}