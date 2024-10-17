#include <cmath>
#include <cstdio>
#include <ctime>
#include <sstream>
#include <string>

using namespace std;


// Function to get the current timestamp (used for logging)
string getCurrentTimeStamp() {
    time_t now = time(nullptr);
    tm* tm_now = localtime(&now);
    char buffer[80];
    strftime(buffer, sizeof(buffer), "%Y-%m-%d_%H-%M-%S", tm_now);
    return string(buffer);
}

// Function to log messages to the SD card for debugging
void logToSDCard(const string& message, const string& filename) {
    FILE* logFile = fopen(filename.c_str(), "a"); // Open log file in append mode
    if (logFile != nullptr) {
        fputs(message.c_str(), logFile);  // Write the message to the file
        fputs("\n", logFile);             // Add a newline for readability
        fclose(logFile);                  // Close the file
    }
}