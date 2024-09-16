#include "pros/adi.hpp"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include <cmath>
#include <cstdio> // For file handling functions (fopen, fputs, etc.)
#include <ctime>  // For timestamps
#include <sstream>

using namespace pros;

#define ERROR_THRESHOLD 0.5 // Stop turning when error is below this value
#define MAX_OSCILLATION 5.0 // Stop excessive oscillations

// Function to get current timestamp for logging
std::string getCurrentTimeStamp() {
    std::time_t now = std::time(nullptr);
    std::tm* tm_now = std::localtime(&now);
    char buffer[80];
    std::strftime(buffer, sizeof(buffer), "%Y-%m-%d_%H-%M-%S", tm_now);
    return std::string(buffer);
}

// Function to log data to the SD card
void logToSDCard(const std::string& message) {
    FILE* logFile = fopen("/usd/PID_Log.txt", "a"); // Append mode

    if (logFile != nullptr) {
        fputs(message.c_str(), logFile);  // Write the message to the file
        fputs("\n", logFile);             // Add a newline for readability
        fclose(logFile);                  // Always close the file after writing
    }
}

// Function to adjust PID constants based on performance metrics
void adjustPIDConstants(float& kp, float& ki, float& kd, float totalError, float overshoot, float timeTaken, float maxOscillation) {
    const float targetOvershoot = 0.0;
    const float targetError = 0.0;
    const float targetTime = 1.0; // Ideal time to complete the turn in seconds
    const float targetOscillation = 0.0; // We want minimal oscillation
    const float learningRate = 0.05; // Adjust learning rate to fine-tune

    // Adjust Kp: Reduce total error (proportional term)
    kp += learningRate * (totalError - targetError);

    // Adjust Kd: Reduce overshoot and oscillations (derivative term)
    kd += learningRate * (overshoot - targetOvershoot) + learningRate * (maxOscillation - targetOscillation);

    // Adjust Ki: Improve time to reach target (integral term)
    ki += learningRate * (timeTaken - targetTime);

    // Ensure the constants remain positive
    kp = fmax(kp, 0.0);
    ki = fmax(ki, 0.0);
    kd = fmax(kd, 0.0);
}

// Function to perform a turn and measure performance
bool performTurn(float targetHeading, ADIGyro& gyro, Motor_Group& LeftSide, Motor_Group& RightSide, float& kp, float& ki, float& kd, 
                 float& totalError, float& overshoot, float& timeTaken, float& maxOscillation) {
    float error = 0, last_error = 0, integral = 0, derivative = 0;
    float turnSpeed = 0;
    const float maxTurnSpeed = 200;
    const int maxTime = 4000; // 4 seconds in milliseconds

    gyro.reset();
    pros::delay(100);
    float startTime = pros::millis();
    
    // Reset performance metrics for this turn
    totalError = 0;
    overshoot = 0;
    maxOscillation = 0;

    // Turn loop
    while (true) {
        float currentHeading = std::abs(gyro.get_value()) / 10.0;  // Get current heading
        error = targetHeading - currentHeading;                    // Calculate error
        integral += error;                                         // Accumulate integral
        derivative = error - last_error;                           // Derivative

        // PID formula
        turnSpeed = (kp * error) + (ki * integral) + (kd * derivative);
        turnSpeed = std::clamp(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

        // Move motors
        RightSide.move_velocity(turnSpeed);
        LeftSide.move_velocity(-turnSpeed);

        // Check for overshoot
        if (currentHeading > targetHeading) overshoot = currentHeading - targetHeading;

        // Record oscillation
        if (fabs(derivative) > maxOscillation) {
            maxOscillation = fabs(derivative);  // Largest oscillation
        }

        // Check if the time exceeds 4 seconds (timeout)
        float currentTime = pros::millis();
        if (currentTime - startTime > maxTime) {
            return false;  // Timeout: too slow, end early
        }

        // **NEW: Stop when within error threshold and minimize oscillation**
        if (std::abs(error) < ERROR_THRESHOLD && maxOscillation < MAX_OSCILLATION) {
            break;
        }

        totalError += std::abs(error);  // Sum the absolute error
        last_error = error;
        pros::delay(10);
    }

    // Calculate time taken
    timeTaken = (pros::millis() - startTime) / 1000.0;

    // Log performance
    std::string logMessage = "Turn to " + std::to_string(targetHeading) + ": Error = " + std::to_string(totalError)
                             + ", Overshoot = " + std::to_string(overshoot) + ", Oscillation = " + std::to_string(maxOscillation)
                             + ", Time = " + std::to_string(timeTaken);
    logToSDCard(logMessage);

    return true; // Turn completed within time
}

// Main function to train PID and log data
void trainPIDConstants(float toHeading, ADIGyro gyro, Motor LBWheel, Motor LMWheel, Motor LFWheel, Motor RBWheel, Motor RMWheel, Motor RFWheel) {
    Motor_Group RightSide({RBWheel, RMWheel, RFWheel});
    Motor_Group LeftSide({LBWheel, LMWheel, LFWheel});

    // Initial PID values
    float kp = 0.87, ki = 0.0000025, kd = 0.005;

    // Log initial PID constants
    logToSDCard("Initial PID Constants: Kp = " + std::to_string(kp) + ", Ki = " + std::to_string(ki) + ", Kd = " + std::to_string(kd));

    // Training loop for PID tuning
    for (int trial = 0; trial < 50; trial++) {
        float totalError = 0, overshoot = 0, timeTaken = 0, maxOscillation = 0;

        // Perform turn to target heading
        bool success = performTurn(toHeading, gyro, LeftSide, RightSide, kp, ki, kd, totalError, overshoot, timeTaken, maxOscillation);

        if (!success) {
            // Turn was too slow, log the timeout
            logToSDCard("Turn too slow (> 4 seconds), adjusting PID...");
        }

        // Perform turn back to 0 degrees
        performTurn(0, gyro, LeftSide, RightSide, kp, ki, kd, totalError, overshoot, timeTaken, maxOscillation);

        // Adjust PID constants based on performance metrics from the last turn
        adjustPIDConstants(kp, ki, kd, totalError, overshoot, timeTaken, maxOscillation);

        // Log adjusted PID constants
        std::string logMessage = "Adjusted PID Constants: Kp = " + std::to_string(kp) + ", Ki = " + std::to_string(ki) + ", Kd = " + std::to_string(kd);
        logToSDCard(logMessage);
    }

    // Final log of PID constants after training
    logToSDCard("Final PID Constants: Kp = " + std::to_string(kp) + ", Ki = " + std::to_string(ki) + ", Kd = " + std::to_string(kd));
}
