#include "main.h"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/rtos.hpp"
#include "src/cmConvertor.cpp"
#include "src/degConvertorTurn.cpp"
#include "src/P-Regler.cpp"
#include "pros/adi.hpp"
#include "pros/motors.hpp"
#include "pros/rtos.h"

using namespace pros;

//Controller
Controller controller (E_CONTROLLER_MASTER);

//Motoren
Motor LFWheel(1, pros::E_MOTOR_GEARSET_18, false,	pros::E_MOTOR_ENCODER_ROTATIONS);
//Motor LMWheel(10, pros::E_MOTOR_GEARSET_18, false,pros::E_MOTOR_ENCODER_ROTATIONS);
Motor LBWheel(2, pros::E_MOTOR_GEARSET_18, false,	pros::E_MOTOR_ENCODER_ROTATIONS);
Motor RFWheel(3, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_ROTATIONS);
//Motor RMWheel(5, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_ROTATIONS);
Motor RBWheel(4, pros::E_MOTOR_GEARSET_18, true,	pros::E_MOTOR_ENCODER_ROTATIONS);

//Motor Groups
Motor_Group Drive({LBWheel, LFWheel, /*LMWheel,*/ RBWheel, RFWheel/*, RMWheel*/});
Motor_Group LeftSide({LBWheel, LFWheel/*, LMWheel*/});
Motor_Group RightSide({RBWheel, RFWheel/*, RMWheel*/});

//Sensoren
ADIGyro gyro('A');


//floats
float turnSpeed = 0;
float headingOffset;

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	gyro.reset();
}

void drehenAufGrad(float toHeading)
{	

	turnToHeading(toHeading, gyro, controller, LBWheel, LFWheel, RBWheel, RFWheel);
	/*controller.rumble("--");
	//headingOffset = gyro.get_value() - wunschHeading;
	controller.clear();
	controller.print(1, 1, "f%",  gyro.get_value());
	while(!(headingOffset >= headingOffset - 1 && headingOffset <= headingOffset + 1))	{
		controller.rumble("..");
		turnSpeed = turnToHeading(wunschHeading, gyro, controller);
		LeftSide.move_absolute(degConvertorTurn(wunschHeading, gyro), turnSpeed);
		RightSide.move_absolute(degConvertorTurn(wunschHeading, gyro), turnSpeed);
	}*/
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() 
{	
	c::delay(1000);
	gyro.reset();

	
	drehenAufGrad(180);
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {

}
