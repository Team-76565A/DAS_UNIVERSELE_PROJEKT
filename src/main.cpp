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
using namespace competition;

// ------------------ WICHTIGER BEREICH ------------------
//                Definition von Variablen
// -------------------------------------------------------

#define learn false

// Define team colors
enum TeamColor { RED, BLUE };
TeamColor current_team = RED;  // Set to RED or BLUE based on your team

bool driving = false;

#define normalStakeFlapPos 16500 // Set to normal Flap position + 4


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
Vision vision_sensor(vision_PORT);
Rotation rotation_sensor(rotation_PORT);

// Task for Donut stuck on Stake
void flapCheck() {
    int angle = rotation_sensor.get_angle();
    int direction = 1;
    int cm = 10;
    while(is_autonomous()) {
        angle = rotation_sensor.get_angle();
        if(angle >= normalStakeFlapPos && !driving) {
            LeftSide.move_relative(direction*convertUnits(cm, "cm", "rotations"), 200);
            RightSide.move_relative(direction*convertUnits(cm, "cm", "rotations"), -200);
            direction= direction * -1;
        } else if(!driving){
            LeftSide.brake();
            RightSide.brake();
        }
        pros::delay(50);  // Check vision sensor every 50 ms
    }
}

// Task for vision monitoring
void visionTask() {
    bool wrongDonut = false;
    bool correctDonut = false;
    int angle = rotation_sensor.get_angle();
    while (is_autonomous()) {
        vision_object_s_t obj = vision_sensor.get_by_size(0);  // Get the largest object
        angle = rotation_sensor.get_angle(); // Get Flap angle
        if(vision_sensor.get_object_count() >= 1) {
            if (obj.signature != VISION_OBJECT_ERR_SIG) {  // Check if an object is detected
                int correct_signature = (current_team == RED) ? 1 : 2;  // Red = sig 1, Blue = sig 2
                if (obj.signature == correct_signature && obj.height >= 50 && obj.width >= 200) {
                    Intake.move_velocity(600);  // Continue intake normally
                    correctDonut = true;
                    wrongDonut = false;
                } else {
                    if(obj.height >= 50 && obj.width >= 200) {
                        if(wrongDonut) {
                            Intake.move_velocity(-600);
                        } else if(!wrongDonut && angle >= normalStakeFlapPos) {
                            wrongDonut = true;
                        } else if(!wrongDonut && correctDonut && !(angle >= normalStakeFlapPos)) {
                            Intake.move_velocity(600);
                        }

                    }
                }
            } else {
                
            }
        }
        

        pros::delay(20);  // Check vision sensor every 20 ms
    }
}

// Initialization
void initialize() {
    pros::lcd::initialize();
    
    controller.clear();

    // Set brake mode to active holding on position
    LeftSide.set_brake_modes(E_MOTOR_BRAKE_HOLD);
    RightSide.set_brake_modes(E_MOTOR_BRAKE_HOLD);
    inertial.reset(true);
}

// Turn to a specified heading
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

// AutoDrive
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

    // Start vision task in parallel
    pros::Task vision_monitor(visionTask);
    pros::Task flap_Wiggle(flapCheck);

    if (learn == true) {
        trainPIDConstants(180, inertial, LBWheel, LMWheel, LFWheel, RBWheel, RMWheel, RFWheel);
    } else {
        //AutoDrive(40, 1);
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


    if(learn == true) {
        trainPIDConstants(180, inertial, LBWheel, LMWheel, LFWheel, RBWheel, RMWheel, RFWheel);
    } else {
        // Start vision task in parallel
        //pros::Task vision_monitor(visionTask);

        while (true) {        
            // Print gyro value on controller screen
            //ontroller.print(0, 0, "Heading: %.2f", inertial.get_yaw());
            controller.print(0, 0, "Rotation: %d", rotation_sensor.get_angle());


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