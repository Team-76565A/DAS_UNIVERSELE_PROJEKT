#include "main.h"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/rtos.hpp"
#include "src/cmConvertor.cpp"
#include "src/degConvertorTurn.cpp"
#include "src/PID-Drive.cpp"
#include "src/P-Regler.cpp"
#include "pros/adi.hpp"
#include "pros/motors.hpp"
#include "pros/rtos.h"

using namespace pros;

//Controller
Controller controller (E_CONTROLLER_MASTER);

//Motoren
Motor LBWheel(10, pros::E_MOTOR_GEARSET_18, true,	pros::E_MOTOR_ENCODER_ROTATIONS);
Motor LMWheel(8, pros::E_MOTOR_GEARSET_18, true,	pros::E_MOTOR_ENCODER_ROTATIONS);
Motor LFWheel(9, pros::E_MOTOR_GEARSET_18, true,	pros::E_MOTOR_ENCODER_ROTATIONS);
Motor RBWheel(1, pros::E_MOTOR_GEARSET_18, false,	pros::E_MOTOR_ENCODER_ROTATIONS);
Motor RMWheel(2, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_ROTATIONS);
Motor RFWheel(3, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_ROTATIONS);

//Motor Groups
//Motor_Group Drive({LBWheel, LFWheel, RBWheel, RFWheel});
Motor_Group LeftSide({LBWheel, LFWheel, LMWheel});
Motor_Group RightSide({RBWheel, RFWheel, RMWheel});

//Sensoren
ADIGyro gyro('A');
ADIDigitalOut piston ('H');



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
	turnToHeading(toHeading, gyro, controller, LBWheel, LMWheel, LFWheel, RBWheel, RMWheel, RFWheel);
	controller.clear();
	controller.print(1, 1, "CurrentHeading f%", gyro.get_value());
}

void drivePID(float driveFor)
{
	//driveStraight(driveFor, gyro.get_value(), gyro, controller, LBWheel, LFWheel, RBWheel, RFWheel);
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
	drehenAufGrad(180);
	/*c::delay(3000);
	drehenAufGrad(170);
	c::delay(3000);
	drehenAufGrad(180);
	c::delay(3000);
	drehenAufGrad(190);*/
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


	while(true)
	{	
		int LVelocity = ((controller.get_analog(E_CONTROLLER_ANALOG_LEFT_Y)/127*100) + (controller.get_analog(E_CONTROLLER_ANALOG_LEFT_X)/127*100)) + (
			(controller.get_analog(E_CONTROLLER_ANALOG_RIGHT_Y)/127*100) + (controller.get_analog(E_CONTROLLER_ANALOG_RIGHT_X)/127*100));
		int RVelocity = ((controller.get_analog(E_CONTROLLER_ANALOG_LEFT_Y)/127*200) - (controller.get_analog(E_CONTROLLER_ANALOG_LEFT_X)/127*200)) + (
			(controller.get_analog(E_CONTROLLER_ANALOG_RIGHT_Y)/127*100) - (controller.get_analog(E_CONTROLLER_ANALOG_RIGHT_X)/127*100));

		LeftSide.move_velocity(LVelocity);
		RightSide.move_velocity(RVelocity);


		if(controller.get_digital(E_CONTROLLER_DIGITAL_A))
		{
			piston.set_value(true);
		}	else {
			piston.set_value(false);
		}
	}
}
