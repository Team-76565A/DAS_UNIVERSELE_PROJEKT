#include "main.h"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/rtos.hpp"
#include "pros/adi.hpp"
#include "pros/motors.hpp"
#include "pros/rtos.h"

// Include custom modules
#include "src/cmConvertor.cpp"
#include "src/degConvertorTurn.cpp"
#include "src/PID-Drive.cpp"
#include "src/P-Regler.cpp"
#include "src/PID-Konstanten.cpp"
#include "src/IntakeManager.cpp"
#include "src/toSDCard.cpp"


using namespace pros;
using namespace competition;
using namespace std;

// ------------------ IMPORTANT SECTION ------------------
//                 Definition of Variables
// -------------------------------------------------------

#define learn false

// Define team colors
enum TeamColor { RED, BLUE };
TeamColor current_team = RED;  // Set to RED or BLUE based on your team

bool driving = false;
Stack donutStack(2);

#define normalStakeFlapPos 16500 // Set to normal Flap position + 4
#define maxHoldFlapPos 20000 // Set the max Flap position when holding an donut under it

// ------------------ PORT DEFINE SECTION ------------------
//                   Definition of Ports
// -------------------------------------------------------

// Motor Ports Drive
#define LBWheel_PORT 1
#define LMWheel_PORT 3
#define LFWheel_PORT 12
#define RBWheel_PORT 7
#define RMWheel_PORT 9
#define RFWheel_PORT 17

// Intake Ports
#define Intake1_PORT 15
#define Intake2_PORT 16

// Pneumatic Ports
#define piston_PORT 'H'
#define climb_PORT 'F'

// Sensor Ports
#define gps_PORT 11
#define inertial_PORT 18
#define up_vision_PORT 21
#define low_vision_PORT 13
#define rotation_PORT 10

// Controller
Controller controller(E_CONTROLLER_MASTER);

// Motors
Motor LBWheel(LBWheel_PORT, E_MOTOR_GEARSET_18, false, E_MOTOR_ENCODER_ROTATIONS);
Motor LMWheel(LMWheel_PORT, E_MOTOR_GEARSET_18, true, E_MOTOR_ENCODER_ROTATIONS);
Motor LFWheel(LFWheel_PORT, E_MOTOR_GEARSET_18, true, E_MOTOR_ENCODER_ROTATIONS);
Motor RBWheel(RBWheel_PORT, E_MOTOR_GEARSET_18, true, E_MOTOR_ENCODER_ROTATIONS);
Motor RMWheel(RMWheel_PORT, E_MOTOR_GEARSET_18, false, E_MOTOR_ENCODER_ROTATIONS);
Motor RFWheel(RFWheel_PORT, E_MOTOR_GEARSET_18, false, E_MOTOR_ENCODER_ROTATIONS);

Motor IntakeMotor1(Intake1_PORT, E_MOTOR_GEARSET_06, true, E_MOTOR_ENCODER_ROTATIONS);
Motor IntakeMotor2(Intake2_PORT, E_MOTOR_GEARSET_06, false, E_MOTOR_ENCODER_ROTATIONS);

// Motor Groups
Motor_Group LeftSide({LBWheel, LFWheel, LMWheel});
Motor_Group RightSide({RBWheel, RFWheel, RMWheel});
Motor_Group Intake({IntakeMotor1, IntakeMotor2});
// Sensors
ADIDigitalOut piston(piston_PORT);
ADIDigitalOut climb(climb_PORT);
Gps gps(gps_PORT, -0.11, -0.13, 180);
Imu inertial(inertial_PORT);
Vision up_vision_sensor(up_vision_PORT);
Vision low_vision_sensor(low_vision_PORT);
Rotation rotation_sensor(rotation_PORT);


// ---------------------------------- Explanation flapCheck -----------------------------------
//                               Task for Donut stuck on Stake
//      Function checks if the Rotation of the Flap is higher than the normal Flap position
//        and wiggles the robot back and forth until the flap returns to normal position.
// --------------------------------------------------------------------------------------------
void flapCheck() {
    int angle = rotation_sensor.get_angle();
    int direction = 1;
    int cm = 10;

    // Keep checking as long as the robot is in autonomous mode
    while (is_autonomous()) {
        // Get the current flap angle
        angle = rotation_sensor.get_angle();

        // If the flap angle is greater than normal, perform the wiggle motion
        if (angle > normalStakeFlapPos && angle <= maxHoldFlapPos && !driving) {
            // Wiggle the robot to help the donut fall onto the stake
            LeftSide.move_relative(direction * convertUnits(cm, "cm", "rotations"), 200);
            RightSide.move_relative(direction * convertUnits(cm, "cm", "rotations"), -200);
            
            // Change direction for the wiggle
            direction *= -1;

        // If the flap is back to normal, stop wiggling
        } else if (angle <= normalStakeFlapPos && !driving) {
            LeftSide.brake();
            RightSide.brake();
        }

        // Delay to avoid overwhelming the CPU
        pros::delay(50);
    }
}



// ---------------------------------- Explanation visionTask -----------------------------------
//                                 Task for checking intake Donut
//                  Function checks the Vision Sensor in the Intake for Objects
// ---------------------------------------------------------------------------------------------
void visionTask() {
    bool wrongDonut = false;
    bool correctDonut = false;
    int angle = rotation_sensor.get_angle(); // Get Flap angle
    int correct_signature = (current_team == RED) ? 1 : 2;  // Red = sig 1, Blue = sig 2
    int first = 0;

    while (is_autonomous()) {
        angle = rotation_sensor.get_angle(); // Get Flap angle
        if(up_vision_sensor.get_object_count() >= 1) {
            vision_object_s_t low_obj = low_vision_sensor.get_by_size(0);  // Get the largest object
            if (low_obj.signature != VISION_OBJECT_ERR_SIG) {  // Check if an object is detected
                if (low_obj.signature == correct_signature && low_obj.height >= 50 && low_obj.width >= 100) { // Check if object is the correct Donut
                    if(first == 0) {
                        addDonut(DOWN, low_obj.signature, donutStack);
                        first++;
                    }
                    
                } else { // If not the correct Donut
                    if(low_obj.height >= 50 && low_obj.width >= 100) {
                        first = 0;
                    }
                }
            } else {
                // Handle error case
            }
        }
        pros::delay(20);  // Check vision sensor every 20 ms
    }
}

// ---------------------------------- Explanation initialize -----------------------------------
//                                 Function for initialization
//                                Function runs before everything
//                                   for Reset of Sensors, etc.
// ---------------------------------------------------------------------------------------------
void initialize() {
    pros::lcd::initialize();
    controller.clear(); // Clear screen

    low_vision_sensor.set_exposure(70);
    up_vision_sensor.set_exposure(70);

    // Set brake mode to active holding on position
    LeftSide.set_brake_modes(E_MOTOR_BRAKE_HOLD);   // Brake mode for braking when Velocity = 0
    RightSide.set_brake_modes(E_MOTOR_BRAKE_HOLD);  // Brake mode for braking when Velocity = 0
    inertial.reset(true); // Reset inertial sensor
}

/**
 * Function to turn the robot to a specified heading using the PID controller.
 * @param toHeading The desired heading to turn to.
 */
void drehenAufGrad(float toHeading) {
    driving = true;    
    turnToHeading(toHeading, inertial, controller, LBWheel, LMWheel, LFWheel, RBWheel, RMWheel, RFWheel);
    controller.clear();
    controller.print(1, 1, "Current Heading: %f", inertial.get_heading());
    driving = false;
}

// Drive PID
void drivePID(float driveFor) {
    // Implement drive PID function here
}

/**
 * Function to drive with the robot for a specified amount of cm.
 * @param cm The desired cm to drive for.
 * @param direction The desired driving direction, 1 for forward and -1 for backward
 * @returns int Returns 0 upon completion.
 */
int AutoDrive(float cm, int direction) {
    driving = true;
    LeftSide.move_relative(direction*convertUnits(cm, "cm", "rotations"), 200);
    RightSide.move_relative(direction*convertUnits(cm, "cm", "rotations"), -200);
    driving = false;
    return 0;
}

// Autonomous
void autonomous() {  
    inertial.reset(true);
    // Generate a new filename for the log file based on current timestamp
    string logFileName = "/usd/Donut_Manager_Log.txt";

    // Start vision task in parallel
    pros::Task vision_monitor(visionTask);
    pros::Task flap_Wiggle(flapCheck);

    if (learn == true) {
        trainPIDConstants(180, inertial, LBWheel, LMWheel, LFWheel, RBWheel, RMWheel, RFWheel);
    } else {
        // AutoDrive(40, 1);
        piston.set_value(true);
        Intake.move_velocity(600);

        //////////////////////////
        //      Autocode        //
        //////////////////////////
        /*AutoDrive(95, -1); 
        pros::delay(1000);
        drehenAufGrad(25); 
        pros::delay(1000);
        AutoDrive(50, -1);
        pros::delay(1000);
        piston.set_value(true);
        pros::delay(1000);*/
    }
    // Autonomous actions can continue here
}

// Operator control
void opcontrol() {
    int x = 0;
    int y = 0;
    bool previousButtonStateIntake = false;
    bool previousButtonStateClimb = false;
    bool pistonActive = false;
    bool climbActive = false;

    // Set brake mode to active holding on position
    LeftSide.set_brake_modes(E_MOTOR_BRAKE_HOLD);
    RightSide.set_brake_modes(E_MOTOR_BRAKE_HOLD);

    if (learn == true) {
        trainPIDConstants(180, inertial, LBWheel, LMWheel, LFWheel, RBWheel, RMWheel, RFWheel);
    } else {
        while (true) {        
            // Print gyro heading to controller
            controller.clear();
            controller.print(1, 1, "Current Heading: %f", inertial.get_heading());


          // Drive control
            LeftSide.move((controller.get_analog(E_CONTROLLER_ANALOG_LEFT_Y)) + 
                        (controller.get_analog(E_CONTROLLER_ANALOG_LEFT_X)));
            RightSide.move((controller.get_analog(E_CONTROLLER_ANALOG_LEFT_Y)) - 
                        (controller.get_analog(E_CONTROLLER_ANALOG_LEFT_X)));

            // Piston control
            bool currentButtonStateIntake = controller.get_digital(E_CONTROLLER_DIGITAL_A);
            if (currentButtonStateIntake && !previousButtonStateIntake) {
                pistonActive = !pistonActive;
                piston.set_value(pistonActive);
            }
            previousButtonStateIntake = currentButtonStateIntake;
            


            // Climb control
            bool currentButtonStateClimb = controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN);
            if (currentButtonStateClimb && !previousButtonStateClimb) {
                climbActive = !climbActive;
                climb.set_value(climbActive);
            }
            previousButtonStateClimb = currentButtonStateClimb;

            // Intake control
            if (controller.get_digital(E_CONTROLLER_DIGITAL_R1)) {
                Intake.move(127);  // Spin intake inwards at full speed
            } else if (controller.get_digital(E_CONTROLLER_DIGITAL_R2)) {
                Intake.move(-127); // Spin intake outwards at full speed
            } else {
                Intake.move(0);    // Stop intake
            }

            pros::delay(20);  // Delay to prevent excessive CPU usage
        }
    }
}
