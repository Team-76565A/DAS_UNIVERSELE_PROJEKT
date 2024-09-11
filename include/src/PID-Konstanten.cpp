#include "pros/adi.hpp"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include <cmath>    // for fabs
#include <fstream>  // for file handling
#include <ctime>    // for timestamps
#include <sstream>  // for stringstream

using namespace pros;

// Function to get the current timestamp as a string
std::string getCurrentTimeStamp() {
    std::time_t now = std::time(nullptr);
    std::tm* tm_now = std::localtime(&now);
    char buffer[80];
    std::strftime(buffer, sizeof(buffer), "%Y-%m-%d_%H-%M-%S", tm_now);
    return std::string(buffer);
}

// Function to log messages to a file
void logToFile(const std::string& message) {
    std::ofstream logFile;
    std::string timestamp = getCurrentTimeStamp();
    std::string filePath = "/include/test/PID_Log_.txt";
    logFile.open(filePath, std::ios::app);

    if (!logFile.is_open()) {
        // If file can't be opened, print to standard output (could be replaced with other error handling)
        //+std::cout << "Error opening log file!" << std::endl;
        return;
    }

    logFile << message << std::endl;
    logFile.close();
}

// Function to train PID constants by testing the robot's performance in turning
void trainPIDConstants(float toHeading, ADIGyro gyro, Motor LBWheel, Motor LMWheel, Motor LFWheel, Motor RBWheel, Motor RMWheel, Motor RFWheel) {
    // Motor groups for left and right sides
    Motor_Group RightSide({RBWheel, RMWheel, RFWheel});
    Motor_Group LeftSide({LBWheel, LMWheel, LFWheel});

    // PID control variables
    float turnSpeed = 0;
    const float maxTurnSpeed = 200;
    float currentHeading = 0;
    float error = 0;
    float integral = 0;
    float derivative = 0;
    float last_error = 0;

    // PID coefficients (starting with default values)
    float kp = 0.87;
    float ki = 0.0000025;
    float kd = 0.005;

    // Best PID constants
    float bestKp = kp, bestKi = ki, bestKd = kd;
    float smallestError = 1000;  // Initially set to a large value

    // PID tuning step size (how much to adjust constants in each step)
    float kp_step = 0.05;
    float ki_step = 0.000005;
    float kd_step = 0.001;

    bool trainingComplete = false;

    // Log initial setup
    std::stringstream ss;
    ss << "Timestamp: " << getCurrentTimeStamp() << "\n";
    ss << "Initial PID Constants - Kp: " << kp << ", Ki: " << ki << ", Kd: " << kd;
    logToFile(ss.str());

    // Training loop to adjust PID constants
    while (!trainingComplete) {
        // Reset integral and error values for each test
        integral = 0;
        last_error = 0;

        // Start with a heading reset
        gyro.reset();
        pros::delay(100); // Small delay to ensure gyro resets

        // Loop until the robot reaches the desired heading
        while (!(currentHeading <= toHeading + 0.8 && currentHeading >= toHeading - 0.8)) {
            currentHeading = fabs(gyro.get_value()) / 10; // Get current heading
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

        // Calculate performance metrics (overshoot, total error)
        float overshoot = fabs(currentHeading - toHeading);
        float totalError = fabs(error);

        // Log the results
        ss << "Test Results:\n";
        ss << "Kp: " << kp << ", Ki: " << ki << ", Kd: " << kd << "\n";
        ss << "Overshoot: " << overshoot << ", Total Error: " << totalError << "\n";
        logToFile(ss.str());

        // If the performance is better than before, update best PID constants
        if (totalError < smallestError) {
            smallestError = totalError;
            bestKp = kp;
            bestKi = ki;
            bestKd = kd;
        } else {
            // Adjust K values for the next trial
            kp += kp_step * (rand() % 3 - 1);  // Randomly adjust P, I, D
            ki += ki_step * (rand() % 3 - 1);
            kd += kd_step * (rand() % 3 - 1);
        }

        // End training if error is small and no overshoot
        if (totalError <= 0.5 && overshoot <= 0.5) {
            trainingComplete = true;
        }

        // Log the best PID constants so far
        ss.str("");  // Clear previous content
        ss << "Best PID Constants so far:\n";
        ss << "Kp: " << bestKp << ", Ki: " << bestKi << ", Kd: " << bestKd << "\n";
        logToFile(ss.str());

        pros::delay(1000);  // Short delay to prevent screen flickering
    }

    // Final log entry
    ss.str("");  // Clear previous content
    ss << "Training complete.\n";
    logToFile(ss.str());
}
