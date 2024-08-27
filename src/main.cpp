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
ADIGyro gyro('A');
ADIDigitalOut piston('H');

// Initialization
void initialize() {
    pros::lcd::initialize();
    gyro.reset();
    controller.clear();
}

// Turn to a specified heading
void drehenAufGrad(float toHeading) {    
    turnToHeading(toHeading, gyro, controller, LBWheel, LMWheel, LFWheel, RBWheel, RMWheel, RFWheel);
    controller.clear();
    controller.print(1, 1, "Current Heading: %f", gyro.get_value());
}

// Map analog value
int mapAnalogValue(int x) {
    return -200 + (200.0 / 127) * (x + 127);
}

// Drive PID
void drivePID(float driveFor) {
    // Implement drive PID function here
}

// Disabled state
void disabled() {}

// Competition initialization
void competition_initialize() {}

// Autonomous
void autonomous() {    
    c::delay(1000);
    drehenAufGrad(180);
    // Additional autonomous actions can be added here
}

// Operator control
void opcontrol() {
    int x = 0;
    int y = 0;
    bool previousButtonState = false;
    bool pistonActive = false;

    while (true) {        
        // Print gyro value on controller screen
        controller.print(0, 0, "Heading: %.2f", gyro.get_value() / 10);

        // Drive control
        LeftSide.move((controller.get_analog(E_CONTROLLER_ANALOG_LEFT_Y)) + 
                      (controller.get_analog(E_CONTROLLER_ANALOG_LEFT_X)));
        RightSide.move((controller.get_analog(E_CONTROLLER_ANALOG_LEFT_Y)) - 
                       (controller.get_analog(E_CONTROLLER_ANALOG_LEFT_X)));
        
        // Piston control
        bool currentButtonState = controller.get_digital(E_CONTROLLER_DIGITAL_A);
        if (currentButtonState && !previousButtonState) {
            pistonActive = !pistonActive;
            piston.set_value(pistonActive);
        }
        previousButtonState = currentButtonState;

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
