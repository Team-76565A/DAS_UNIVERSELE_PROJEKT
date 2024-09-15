#include "pros/adi.hpp"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include <cmath>
#include <cstdio> // For file handling functions (fopen, fputs, etc.)
#include <ctime>  // For timestamps
#include <sstream>

using namespace pros;

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

// Function to calculate new PID constants based on performance analysis
void adjustPIDConstants(float& kp, float& ki, float& kd, float totalError, float overshoot, float timeTaken) {
    // Target performance criteria
    const float targetOvershoot = 0.0;
    const float targetError = 0.0;
    const float targetTime = 1.0; // seconds, can be tuned

    // Adjustment rate for PID coefficients
    const float learningRate = 0.1;

    // Adjust Kp: Focus on reducing total error
    if (totalError > targetError) {
        kp += learningRate * (totalError - targetError);
    } else {
        kp -= learningRate * (targetError - totalError);
    }

    // Adjust Kd: Focus on reducing overshoot
    if (overshoot > targetOvershoot) {
        kd += learningRate * (overshoot - targetOvershoot);
    } else {
        kd -= learningRate * (targetOvershoot - overshoot);
    }

    // Adjust Ki: Focus on convergence time (minimizing time to reach the target)
    if (timeTaken > targetTime) {
        ki += learningRate * (timeTaken - targetTime);
    } else {
        ki -= learningRate * (targetTime - timeTaken);
    }

    // Ensure the constants remain positive
    kp = fmax(kp, 0.0);
    ki = fmax(ki, 0.0);
    kd = fmax(kd, 0.0);
}

// Main function to train PID and log data
void trainPIDConstants(float toHeading, ADIGyro gyro, Motor LBWheel, Motor LMWheel, Motor LFWheel, Motor RBWheel, Motor RMWheel, Motor RFWheel) {
    Motor_Group RightSide({RBWheel, RMWheel, RFWheel});
    Motor_Group LeftSide({LBWheel, LMWheel, LFWheel});

    // Initial PID values
    float kp = 0.87, ki = 0.0000025, kd = 0.005;
    float error = 0, last_error = 0, integral = 0, derivative = 0;
    float turnSpeed = 0;
    const float maxTurnSpeed = 200;

    // Log initial PID constants
    logToSDCard("Initial PID Constants: Kp = " + std::to_string(kp) + ", Ki = " + std::to_string(ki) + ", Kd = " + std::to_string(kd));

    // Training loop for PID tuning
    for (int trial = 0; trial < 50; trial++) {
        integral = 0;
        last_error = 0;
        gyro.reset();
        pros::delay(100);

        // Performance metrics
        float totalError = 0;
        float overshoot = 0;
        float startTime = pros::millis();
        float currentHeading = 0;

        // Perform turn control using PID
        do {
            currentHeading = std::abs(gyro.get_value()) / 10.0;  // Get current heading
            error = toHeading - currentHeading;                  // Calculate error
            integral += error;                                   // Accumulate integral
            derivative = error - last_error;                     // Derivative (rate of change)

            turnSpeed = (kp * error) + (ki * integral) + (kd * derivative); // PID formula
            turnSpeed = std::clamp(turnSpeed, -maxTurnSpeed, maxTurnSpeed); // Limit motor speed

            RightSide.move_velocity(turnSpeed);
            LeftSide.move_velocity(-turnSpeed);

            // Collect performance metrics
            totalError += std::abs(error);
            if (currentHeading > toHeading) overshoot = currentHeading - toHeading;  // Record overshoot

            last_error = error;
            pros::delay(10);

        } while (std::abs(error) > 0.5);  // Exit when close to the target heading

        float endTime = pros::millis();
        float timeTaken = (endTime - startTime) / 1000.0; // Time taken in seconds

        // Log trial results
        std::string logMessage = "Trial " + std::to_string(trial) + " - Total Error: " + std::to_string(totalError)
                               + " | Overshoot: " + std::to_string(overshoot) + " | Time: " + std::to_string(timeTaken);
        logToSDCard(logMessage);

        // Adjust PID constants based on performance analysis
        adjustPIDConstants(kp, ki, kd, totalError, overshoot, timeTaken);

        // Log adjusted PID constants
        logMessage = "Adjusted PID Constants: Kp = " + std::to_string(kp) + ", Ki = " + std::to_string(ki) + ", Kd = " + std::to_string(kd);
        logToSDCard(logMessage);
    }

    // Final log of PID constants after training
    logToSDCard("Final PID Constants: Kp = " + std::to_string(kp) + ", Ki = " + std::to_string(ki) + ", Kd = " + std::to_string(kd));
}
