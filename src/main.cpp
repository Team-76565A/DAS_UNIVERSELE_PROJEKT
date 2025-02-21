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
#include <string>




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
#define stakeSide true
#define skills true

int team_color;

bool driving = false;
Stack stakeStack(2);

bool intakeOn = false;
bool intakeReverse = false;


// Generate a new filename for the log file based on current timestamp
string logFileName = "/usd/Log_File_" + getCurrentTimeStamp() + ".txt";

#define normalStakeFlapPos 5000 // Set to normal Flap position + 4
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

// Bumper Port
#define lineTracker_Port 'D'

// Sensor Ports
#define gps_PORT 6
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
ADIAnalogIn intakeTracker(lineTracker_Port);


bool is_Driving() {
    // Fetch velocities of all motors in LeftSide and RightSide
    std::vector<double> left_velocities = LeftSide.get_actual_velocities();
    std::vector<double> right_velocities = RightSide.get_actual_velocities();

    // Check if any motor in LeftSide or RightSide is moving
    for (double velocity : left_velocities) {
        if (velocity != 0) return true;
    }
    for (double velocity : right_velocities) {
        if (velocity != 0) return true;
    }

    return false;  // Return false if all motors are stationary
}



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
            if(!LeftSide.at(1).is_stopped() && !RightSide.at(1).is_stopped()) {
                // Change direction for the wiggle
                direction *= -1;
            } else {
                delay(10);
            }
            

        // If the flap is back to normal, stop wiggling
        } else if (angle <= normalStakeFlapPos && !is_Driving()) {
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
    int angle = rotation_sensor.get_angle(); // Get Flap angle
    int correct_signature = (current_team == RED) ? 1 : 2;  // Red = sig 1, Blue = sig 2
    int lowScreenPos;
    int upScreenPos;
    Position donutPosition;
    bool correctDonut;
    bool readyForNewLow = true;
    bool readyForNewMiddle = true;

    while (is_autonomous()) {
        angle = rotation_sensor.get_angle(); // Get updated Flap angle

        // Get the largest object detected by each sensor
        vision_object_s_t low_obj = low_vision_sensor.get_by_size(0);
        vision_object_s_t up_obj = up_vision_sensor.get_by_size(0);

        // Check if both sensors detect a donut
        if (low_obj.signature != VISION_OBJECT_ERR_SIG || up_obj.signature != VISION_OBJECT_ERR_SIG) {

            // Set onScreenPosition
            lowScreenPos = low_obj.x_middle_coord;
            upScreenPos = up_obj.x_middle_coord;

            // Determine the position of the donut
            if (low_obj.width > 50 && low_obj.height > 50) {
                donutPosition = DOWN;
            } else if (up_obj.width > 50 && up_obj.height > 50) {
                donutPosition = MIDDLE;
            } else if (angle >= normalStakeFlapPos) {
                donutPosition = TOP;
            } else {
                donutPosition = NONE;
            }

            // Check if the detected donut is correct based on the signature
            correctDonut = (low_obj.signature == correct_signature || up_obj.signature == correct_signature);

            // Update or add donut if the donut is correct
            if (donutPosition != NONE) {
                if (correctDonut) {     
                } else {
                    // Push incorrect donut out of the intake
                    Intake.move(-127);  // Spin intake in reverse
                    pros::delay(1000);  // Delay for 1 second to ensure it's pushed out
                    Intake.move(127);  // Stop the intake
                    
                }
            } else {

            }
        }

        pros::delay(20);  // Check sensors every 20 ms
    }
}

void intakeTask() {
    while(is_autonomous()){
        if(intakeOn) {
            if(intakeReverse) {
                Intake.move_velocity(-600);
            } else {
                Intake.move_velocity(600);
            }
        } else {
            Intake.brake();
        }
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
    rotation_sensor.reset_position();

    low_vision_sensor.set_exposure(70);
    up_vision_sensor.set_exposure(70);

    if(current_team == RED) {
        team_color = -1;
    } else if(current_team == BLUE) {
        team_color = 1;
    }

    // Set brake mode to active holding on position
    LeftSide.set_brake_modes(E_MOTOR_BRAKE_HOLD);   // Brake mode for braking when Velocity = 0
    RightSide.set_brake_modes(E_MOTOR_BRAKE_HOLD);  // Brake mode for braking when Velocity = 0
    inertial.reset(false); // Reset inertial sensor
    
}

/**
 * Function to turn the robot to a specified heading using the PID controller.
 * @param toHeading The desired heading to turn to.
 */
int drehenAufGrad(float toHeading) { 
    turnToHeading(toHeading, inertial, controller, LBWheel, LMWheel, LFWheel, RBWheel, RMWheel, RFWheel);
    logToSDCard("Turn", logFileName);
    controller.clear();
    controller.print(1, 1, "Current Heading: %f", inertial.get_yaw());
    delay(100);
    while(is_Driving()) {delay(50);}
    return 0;
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
int AutoDrive(float cm, int direction = 1, int speed = 200) {
    LeftSide.move_relative(direction*convertUnits(cm, "cm", "rotations"), speed);
    RightSide.move_relative(direction*convertUnits(cm, "cm", "rotations"), -speed);
    delay(100);
    while(is_Driving()) {delay(50);}
    return 0;
}

/**
 * Function to turn the robot to a specified heading using the PID controller.
 * @param toHeading The desired heading to turn to.
 */
int turn(float toHeading, int direction = 1, float speed = 200) {
    LeftSide.move_relative(team_color*direction*(toHeading/100), speed);
    RightSide.move_relative(team_color*-direction*(toHeading/100), speed);
    logToSDCard("Turn", logFileName);
    delay(100);
    while(is_Driving()) {delay(50);}
    return 0;
}

/**
 * Use Function 5 cm before Stake to get the best out of the function
 */
void driveToStake(int direction = 1) {
    while(intakeTracker.get_value() > 2000) {
        LeftSide.move_velocity(direction*50);
        RightSide.move_velocity(direction*50);
    }
}

/**
 * The Flapcheck if donut on Stake
 */
void intakeCheck() {
    Position donutPosition = START;
    int angle = rotation_sensor.get_position();
    while (donutPosition != NONE) {
        // Get the largest object detected by each sensor
        vision_object_s_t low_obj = low_vision_sensor.get_by_size(0);
        vision_object_s_t up_obj = up_vision_sensor.get_by_size(0);

        angle = rotation_sensor.get_position();

        // Check if both sensors detect a donut
        if (low_obj.signature != VISION_OBJECT_ERR_SIG || up_obj.signature != VISION_OBJECT_ERR_SIG) {


            // Determine the position of the donut
            if (low_obj.width > 80 && low_obj.height > 80) {
                donutPosition = DOWN;
            } else if (up_obj.width > 80 && up_obj.height > 80) {
                donutPosition = MIDDLE;
            } else if (angle >= 6000) {
                donutPosition = TOP;
            } else {
                donutPosition = NONE;
            }
        }
        
        if(donutPosition == TOP2) {
            intakeReverse = true;
        } else {
            intakeReverse = false;
        }
    }
}

/**
 * FLap Check
 */
void flapFree(int i = normalStakeFlapPos) {
    int angle = rotation_sensor.get_position();
    while(rotation_sensor.get_angle() >= i) {
        delay(50);
    }
}

void intake() {
    intakeCheck();
    intakeReverse = true;
    flapFree();
    intakeReverse = false;
    flapFree();
    intakeReverse = true;
    flapFree();
    intakeReverse = false;
    flapFree();
}

// Autonomous
void autonomous() {  
    // Start vision task in parallel
    //pros::Task vision_monitor(visionTask);
    //pros::Task flap_Wiggle(flapCheck);
    Task intake_Task(intakeTask);

    if(learn) {
        trainPIDConstants(180, inertial, LBWheel, LMWheel, LFWheel, RBWheel, RMWheel, RFWheel);
    } else if(!learn && !skills){

        //////////////////////////
        //      Autocode        //
        //////////////////////////
        if(stakeSide) {
            AutoDrive(5, -1, 50);
            AutoDrive(82.5, -1, 170);
            turn(33); 
            AutoDrive(21.7, -1, 80);    // Cm is in -5 cause of testing driveToStake
            delay(200);
            driveToStake(-1);
            piston.set_value(true);
            delay(20);
            intakeOn = true;
            turn(50, -1);
            delay(500);
            intakeReverse = true;
            AutoDrive(20);
            intakeOn = true;
            intakeReverse = false;
            AutoDrive(15);
            delay(500);
            intakeReverse = true;
        } else {
            AutoDrive(5, -1, 50);
            AutoDrive(38, -1, 170);
            driveToStake(-1);
            piston.set_value(true);
            intakeOn = true;
            turn(80, -1);
            delay(200);
            intakeReverse = true;
            delay(500);
            intakeReverse = false;
            AutoDrive(40, 1, 80);
            AutoDrive(30,1,200);
            delay(200);
            intakeReverse = true;
            turn(80, -1);
            intakeReverse = false;
            AutoDrive(20,1,170);
            turn(40, -1);
            intakeReverse = true;
            delay(500);
            intakeReverse = false;
            AutoDrive(15, 1, 170);
        }

    } else if(!learn && skills) {
        driveToStake(-1);
        piston.set_value(true);
        intakeOn = true;        
        turn(47, -1);
        intake(); // PreLoad
        AutoDrive(35, 1, 170);
        intake(); // Erster Donut
        turn(45, -1);
        intake(); // Flap Check for wrong donut
        AutoDrive(30, 1, 170);
        intake(); // Zweiter Donut
        turn(90, -1);
        intake(); // Flap Check for wrong donut
        AutoDrive(30, 1, 170);
        intake(); // Dritter Donut
        turn(30);
        intake(); // Flap Check for wrong donut
        AutoDrive(60, 1, 170);
        intake(); // Vierter Donut
        turn(100, -1);
        intake(); // Flap Check for wrong donut
        AutoDrive(35, 1, 170);
        intake(); // Fünfter Donut
        turn(90, -1);
        intake(); // Flap Check for wrong donut
        AutoDrive(40, 1, 170);
        intake(); // Sechster Donut
        turn(145);
        intake(); // Flap Check for wrong donut
        AutoDrive(75, -1, 170);
        intake(); // Flap Check for wrong donut
        piston.set_value(false); // First Stake corner

        // Driving to second Stake
        AutoDrive(20, 1, 170);
        turn(140, 1);
        AutoDrive(147.4, -1, 170);

        // Second stake
        driveToStake(-1);
        piston.set_value(true);
        AutoDrive(10, -1, 170);
        turn(95, -1);
        intake();
        AutoDrive(60.96, 1, 170);
        intake(); // First Donut
        turn(85, -1);
        intake();
        AutoDrive(60.96, 1, 170);
        intake(); // Second Donut
        turn(90, -1);
        intake();
        AutoDrive(60.96, 1, 170);
        intake();
        AutoDrive(20, 1, 170);
        intake();
        turn(135, 1);
        intake();
        AutoDrive(28.28, 1, 170);
        intake();
        turn(75, 1);
        intake();
        AutoDrive(20, -1, 170);
        intake();
        piston.set_value(false);







        



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
            controller.print(0, 0, "Current Heading: %f", inertial.get_heading());


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