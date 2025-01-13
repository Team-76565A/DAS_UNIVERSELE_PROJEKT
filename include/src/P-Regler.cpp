#include "pros/adi.hpp"
#include "pros/imu.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.hpp"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"

using namespace pros;

/**
 * Function to turn the robot to a specified heading using the PID controller.
 * 
 * @param toHeading The desired heading to turn to.
 * @param gyro The gyro sensor to use for heading feedback.
 * @param controller The controller for user input.
 * @param Left1, Left2, Left3 The motors on the left side of the robot.
 * @param Right1, Right2, Right3 The motors on the right side of the robot.
 * @return int Returns 0 upon completion.
 */
int turnToHeading(float toHeading, Imu inertial, Controller controller, Motor Left1, Motor Left2, Motor Left3, Motor Right1, Motor Right2, Motor Right3) {
    // Motor groups for left and right sides
    Motor_Group RightSide({Right1, Right2, Right3});
    Motor_Group LeftSide({Left1, Left2, Left3});

    // PID control variables
    float turnSpeed = 0;
    const float maxTurnSpeed = 200;
    float currentHeading = 0;
    float error = 0;
    float integral = 0;
    float derivative = 0;
    float last_error = 0;

    // PID coefficients
    const float kp = 0.880680, ki = 0.003513, kd = 0.015666;

    //nomalise toHeading to (180/-180) value
    if (toHeading > 180) {
       toHeading = toHeading - 360;
    }


    // Loop until the robot reaches the desired heading
    while (!(currentHeading <= toHeading + 0.8 && currentHeading >= toHeading - 0.8)) {
        currentHeading = inertial.get_yaw(); // Get current heading
        error = currentHeading - toHeading; // Calculate error
        integral += error; // Update integral
        derivative = error - last_error; // Calculate derivative

        // Calculate turn speed using PID formula
        turnSpeed = (kp * error) + (ki * integral) + (kd * derivative);

        // Clamp turn speed to maximum allowable value
        if (turnSpeed >= maxTurnSpeed) {
            turnSpeed = maxTurnSpeed;
        } else if (turnSpeed <= -maxTurnSpeed) {
            turnSpeed = -maxTurnSpeed;
        }

        // Move the motors
        RightSide.move_velocity(turnSpeed);
        LeftSide.move_velocity(-turnSpeed);

        last_error = error; // Update last error
    }

    // Stop the motors when the turn is complete
    RightSide.brake();
    LeftSide.brake();

    return 0;
}
