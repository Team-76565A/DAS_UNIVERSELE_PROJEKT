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

// Controller
Controller controller(E_CONTROLLER_MASTER);

// Motors
Motor LBWheel(1, E_MOTOR_GEARSET_18, false, E_MOTOR_ENCODER_ROTATIONS);
Motor LMWheel(3, E_MOTOR_GEARSET_18, true, E_MOTOR_ENCODER_ROTATIONS);
Motor LFWheel(4, E_MOTOR_GEARSET_18, true, E_MOTOR_ENCODER_ROTATIONS);
Motor RBWheel(18, E_MOTOR_GEARSET_18, true, E_MOTOR_ENCODER_ROTATIONS);
Motor RMWheel(9, E_MOTOR_GEARSET_18, false, E_MOTOR_ENCODER_ROTATIONS);
Motor RFWheel(17, E_MOTOR_GEARSET_18, false, E_MOTOR_ENCODER_ROTATIONS);

Motor IntakeMotor1(16, E_MOTOR_GEARSET_06, true, E_MOTOR_ENCODER_ROTATIONS);
Motor IntakeMotor2(15, E_MOTOR_GEARSET_06, false, E_MOTOR_ENCODER_ROTATIONS);

// Motor Groups
Motor_Group LeftSide({LBWheel, LFWheel, LMWheel});
Motor_Group RightSide({RBWheel, RFWheel, RMWheel});
Motor_Group Intake({IntakeMotor1, IntakeMotor2});

// Sensors
ADIDigitalOut piston('H');
ADIDigitalOut climb('F');
Gps gps(11, -0.11, -0.13, 180);
Imu inertial(12);


// Important Variables
bool learn = true;

// Initialization
void initialize() {
    pros::lcd::initialize();
    inertial.reset(true);
    controller.clear();

    //Set Breake mode to active holding on position
    LeftSide.set_brake_modes(E_MOTOR_BRAKE_HOLD);
    RightSide.set_brake_modes(E_MOTOR_BRAKE_HOLD);
}

// Turn to a specified heading
void drehenAufGrad(float toHeading) {    
    turnToHeading(toHeading, inertial, controller, LBWheel, LMWheel, LFWheel, RBWheel, RMWheel, RFWheel);
    controller.clear();
    controller.print(1, 1, "Current Heading: %f", inertial.get_heading());
}

// Map analog value
int mapAnalogValue(int x) {
    return -200 + (200.0 / 127) * (x + 127);
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


// Disabled state
void disabled() {}

// Competition initialization
void competition_initialize() {}

// Autonomous
void autonomous() {  

    inertial.reset(true);

    //TODO etwas weiter fahren
    if(learn == true)
    {
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
        //trainPIDConstants(180, gyro, LBWheel, LMWheel, LFWheel, RBWheel, RMWheel, RFWheel);
        // Additional autonomous actions can be added here
    }
    
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
