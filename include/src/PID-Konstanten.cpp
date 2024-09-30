#include "pros/adi.hpp"
#include "pros/imu.hpp"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include <cmath>
#include <cstdio>
#include <ctime>
#include <sstream>
#include <string>

using namespace pros;
using namespace std;

// Define PID constants and thresholds
#define ERROR_THRESHOLD 0.5   // Error tolerance to stop turning
#define MAX_OSCILLATION 5.0   // Limit oscillation to stop
#define MAX_KP 1.0            // Max value for Kp (proportional gain)
#define MAX_KI 1.0            // Max value for Ki (integral gain)
#define MAX_KD 5.0            // Max value for Kd (derivative gain)

// Function to normalize angle between -180 and 180 degrees
float normalizeAngle(float angle) {
    while (angle > 180.0) angle += 360.0;
    while (angle < -180.0) angle -= 360.0;
    return angle;
}

// Function to get the current timestamp (used for logging)
std::string getCurrentTimeStamp() {
    std::time_t now = std::time(nullptr);
    std::tm* tm_now = std::localtime(&now);
    char buffer[80];
    std::strftime(buffer, sizeof(buffer), "%Y-%m-%d_%H-%M-%S", tm_now);
    return std::string(buffer);
}

// Function to log messages to the SD card for debugging
void logToSDCard(const std::string& message, const std::string& filename) {
    FILE* logFile = fopen(filename.c_str(), "a"); // Open log file in append mode
    if (logFile != nullptr) {
        fputs(message.c_str(), logFile);  // Write the message to the file
        fputs("\n", logFile);             // Add a newline for readability
        fclose(logFile);                  // Close the file
    }
}

// Function to adjust PID constants based on performance
void adjustPIDConstants(float& kp, float& ki, float& kd, float totalError, float overshoot, float timeTaken, float maxOscillation) {
    const float targetOvershoot = 0.0;    // Target is no overshoot
    const float targetError = 0.0;        // Target is no error
    const float targetTime = 1.0;         // Target time to complete the turn (in seconds)
    const float targetOscillation = 0.0;  // Minimal oscillation is desired
    const float learningRate = 0.01;      // Learning rate for Kp and Kd adjustments
    const float learningRateI = 0.00001;  // Slower learning rate for Ki (integral gain)

    // Adjust Kp to reduce total error (proportional adjustment)
    kp += learningRate * (totalError - targetError);

    // Adjust Kd to minimize overshoot (derivative adjustment)
    kd += learningRate * (overshoot - targetOvershoot);

    // Adjust Ki to improve time to reach the target (integral adjustment)
    ki += learningRateI * (timeTaken - targetTime);

    // Ensure PID constants stay within predefined limits
    kp = fmin(fmax(kp, 0.0), MAX_KP);
    ki = fmin(fmax(ki, 0.0), MAX_KI);
    kd = fmin(fmax(kd, 0.0), MAX_KD);
}

// Function to perform a turn based on the PID controller
bool performTurn(float targetHeading, Imu& inertial, Motor_Group& LeftSide, Motor_Group& RightSide, float& kp, float& ki, float& kd, 
                 float& totalError, float& overshoot, float& timeTaken, float& maxOscillation, string& logFileName) {
    float error = 0, last_error = 0, integral = 0, derivative = 0;
    float turnSpeed = 0;
    const float maxTurnSpeed = 200;  // Max speed for turning
    const int maxTime = 4000;        // Max time to complete the turn (in milliseconds)

    pros::delay(100);  // Small delay to ensure sensors are ready
    float startTime = pros::millis();  // Start time to measure time taken for the turn
    
    // Reset performance metrics for this turn
    totalError = 0.0;
    overshoot = 0.0;
    maxOscillation = 0.0;

    // Normalize targetHeading to -180 to 180 degrees
    if (targetHeading > 180) {
        targetHeading = targetHeading - 360;
    }

    // PID control loop for turning
    while (true) {
        // Get the current heading from the inertial sensor
        float currentHeading = std::abs(inertial.get_yaw());  
        error = targetHeading - currentHeading;  // Calculate error (target - current)
        integral += error;  // Accumulate integral (sum of errors)
        derivative = error - last_error;  // Derivative (rate of change of error)

        // PID formula to calculate the turn speed
        turnSpeed = (kp * error) + (ki * integral) + (kd * derivative);
        turnSpeed = std::clamp(turnSpeed, -maxTurnSpeed, maxTurnSpeed);  // Clamp speed to maxTurnSpeed

        // Move the motors to turn the robot
        RightSide.move_velocity(turnSpeed);
        LeftSide.move_velocity(-turnSpeed);

        // Measure overshoot if current heading exceeds target
        if (currentHeading > targetHeading) overshoot = currentHeading - targetHeading;

        // Check if the time taken exceeds maxTime (timeout)
        float currentTime = pros::millis();
        if (currentTime - startTime > maxTime) {
            // Calculate total time taken for the turn
            timeTaken = (pros::millis() - startTime) / 1000.0;  // Convert to seconds
            // Log timeout and performance data
            std::string logMessage = "Turn to " + std::to_string(targetHeading) + "\nError = " + std::to_string(totalError)
                                    + ", Overshoot = " + std::to_string(overshoot) + "\nTime = " + std::to_string(timeTaken);
            logToSDCard(logMessage, logFileName);
            return false;  // Return false if the turn took too long (timeout)
        }

        // Stop turning if error is small and oscillation is minimal
        if (std::abs(error) < ERROR_THRESHOLD && maxOscillation < MAX_OSCILLATION) {
            break;  // Break out of the loop when the turn is successful
        }

        totalError = std::abs(error);  // Sum the absolute error for performance metrics
        last_error = error;  // Update last error for the next loop
        pros::delay(10);  // Delay to avoid high-frequency control updates
    }

    // Calculate total time taken for the turn
    timeTaken = (pros::millis() - startTime) / 1000.0;  // Convert to seconds

    // Log successful turn performance data
    std::string logMessage = "Turn to " +  std::to_string(targetHeading) + "\nError = " + std::to_string(totalError)
                            + "\nOvershoot = " + std::to_string(overshoot) + "\nTime = " + std::to_string(timeTaken);
    logToSDCard(logMessage, logFileName);

    return true;  // Return true if the turn was successful within time
}

// Function to train PID constants through multiple trials
void trainPIDConstants(float toHeading, Imu inertial, Motor LBWheel, Motor LMWheel, Motor LFWheel, Motor RBWheel, Motor RMWheel, Motor RFWheel) {
    // Group motors into Left and Right motor groups for control
    Motor_Group RightSide({RBWheel, RMWheel, RFWheel});
    Motor_Group LeftSide({LBWheel, LMWheel, LFWheel});

    // Initial PID constants (these will be adjusted)
    float kp = 0.87, ki = 0.0000025, kd = 0.005;

    // Variables to track the best PID constants
    float bestKP = kp, bestKI = ki, bestKD = kd;
    float lowestError = 180;

    // Generate a new filename for the log file based on current timestamp
    std::string logFileName = "/usd/PID_Log_" + getCurrentTimeStamp() + ".txt";


    // Log the initial PID constants
    logToSDCard("Initial PID Constants: Kp = " + std::to_string(kp) + ", Ki = " + std::to_string(ki) + ", Kd = " + std::to_string(kd), logFileName);

    // Training loop: Perform multiple trials to adjust PID constants
    for (int trial = 0; trial < 50; trial++) {
        float totalError = 0, overshoot = 0, timeTaken = 0, maxOscillation = 0;

        // Perform turn to target heading
        bool success = performTurn(toHeading, inertial, LeftSide, RightSide, kp, ki, kd, totalError, overshoot, timeTaken, maxOscillation, logFileName);

        // Log if turn took too long (timeout)
        if (!success) {
            logToSDCard("Turn too slow (> 4 seconds), adjusting PID...", logFileName);
        }

         // Track the best performing PID constants (based on lowest total error)
        if (totalError < lowestError) {
            lowestError = totalError;
            bestKP = kp;
            bestKI = ki;
            bestKD = kd;
        }

        lowestError = totalError;
        
        // Perform turn back to 0 degrees after each trial
        performTurn(0, inertial, LeftSide, RightSide, kp, ki, kd, totalError, overshoot, timeTaken, maxOscillation, logFileName);

        // Track the best performing PID constants (based on lowest total error)
        if (totalError < lowestError) {
            lowestError = totalError;
            bestKP = kp;
            bestKI = ki;
            bestKD = kd;
        }

        lowestError = totalError;
        // Adjust the PID constants for the next trial based on performance
        adjustPIDConstants(kp, ki, kd, totalError, overshoot, timeTaken, maxOscillation);

        // Log adjusted PID constants
        std::string logMessage = "Adjusted PID Constants: Kp = " + std::to_string(kp) + ", Ki = " + std::to_string(ki*100) + ", Kd = " + std::to_string(kd) + " Trial Nr.:" + std::to_string(trial);
        logToSDCard(logMessage, logFileName);
    }

    // Log the final PID constants after training
    logToSDCard("Final PID Constants: Kp = " + std::to_string(kp) + ", Ki = " + std::to_string(ki) + ", Kd = " + std::to_string(kd), logFileName);
    
    // Log and display the best PID constants found during training
    logToSDCard("Best PID Constants: Kp = " + std::to_string(bestKP) + ", Ki = " + std::to_string(bestKI) + ", Kd = " + std::to_string(bestKD), logFileName);
    printf("Best PID Constants: Kp = %f, Ki = %f, Kd = %f\n", bestKP, bestKI, bestKD);
}