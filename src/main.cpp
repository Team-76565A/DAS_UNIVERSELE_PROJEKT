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

using namespace pros;

// ------------------ WICHTIGER BEREICH ------------------
//                Definition von Variablen
// -------------------------------------------------------

#define learn false

// Define team colors
enum TeamColor { RED, BLUE };
TeamColor current_team = RED;  // Set to RED or BLUE based on your team

// ------------------ PORT DEFINE BEREICH ------------------
//                   Definition von Ports
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

// Sensoren Ports
#define gps_PORT 11
#define inertial_PORT 18
#define vision_PORT 21

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
Vision vision_sensor(vision_PORT);

// Task for vision monitoring
void visionTask() {
    while (true) {
        vision_object_s_t obj = vision_sensor.get_by_size(0);  // Get the largest object

        if (obj.signature != VISION_OBJECT_ERR_SIG) {  // Check if an object is detected
            int correct_signature = (current_team == RED) ? 1 : 2;  // Red = sig 1, Blue = sig 2
            if (obj.signature == correct_signature) {
                Intake.move(127);  // Continue intake normally
            } else {
                Intake.move(-127); // Reverse intake to spit out the wrong object
            }
        } else {
            
        }

        pros::delay(100);  // Check vision sensor every 100 ms
    }
}

// Initialization
void initialize() {
    pros::lcd::initialize();
    inertial.reset(true);
    controller.clear();

    // Set brake mode to active holding on position
    LeftSide.set_brake_modes(E_MOTOR_BRAKE_HOLD);
    RightSide.set_brake_modes(E_MOTOR_BRAKE_HOLD);
}

// Turn to a specified heading
void drehenAufGrad(float toHeading) {    
    turnToHeading(toHeading, inertial, controller, LBWheel, LMWheel, LFWheel, RBWheel, RMWheel, RFWheel);
    controller.clear();
    controller.print(1, 1, "Current Heading: %f", inertial.get_heading());
}

// Drive PID
void drivePID(float driveFor) {
    // Implement drive PID function here
}

// AutoDrive
int AutoDrive(float cm, int direction) {
    LeftSide.move_relative(direction*convertUnits(cm, "cm", "rotations"), 200);
    RightSide.move_relative(direction*convertUnits(cm, "cm", "rotations"), -200);

    return 0;
}

// Autonomous
void autonomous() {  
    inertial.reset(true);

    // Start vision task in parallel
    pros::Task vision_monitor(visionTask);

    if (learn == true) {
        trainPIDConstants(180, inertial, LBWheel, LMWheel, LFWheel, RBWheel, RMWheel, RFWheel);
    } else {
        AutoDrive(95, -1); 
        pros::delay(1000);
        drehenAufGrad(25); 
        pros::delay(1000);
        AutoDrive(50, -1);
        pros::delay(1000);
        piston.set_value(true);
        pros::delay(1000);
    }

    // Autonomous actions can continue here, while vision monitoring runs in parallel
}

// Operator control
void opcontrol() {
    int x = 0;
    int y = 0;
    bool previousButtonStateIntake = false;
	bool previousButtonStateClimb = false;
    bool pistonActive = false;
	bool climbActive = false;

    //Set Breake mode to active holding on position
    LeftSide.set_brake_modes(E_MOTOR_BRAKE_HOLD);
    RightSide.set_brake_modes(E_MOTOR_BRAKE_HOLD);

    if(learn == true)
    {
        trainPIDConstants(180, inertial, LBWheel, LMWheel, LFWheel, RBWheel, RMWheel, RFWheel);
    } else {
        // Start vision task in parallel
        pros::Task vision_monitor(visionTask);
        
        while (true) {        
            // Print gyro value on controller screen
            controller.print(0, 0, "Heading: %.2f", inertial.get_yaw());

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