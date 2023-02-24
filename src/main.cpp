#include "main.h"

//DEFINES
#define INTAKE_MOTOR 15
#define FLYWHEEL_MOTOR 19
#define EXPANSION_PIN 'A'
#define LIMIT_PIN 'E'
#define EXPANSION_BLOCKER 'C'

//GLOBALS
pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::Motor intake(INTAKE_MOTOR, pros::E_MOTOR_GEARSET_18);
pros::ADIDigitalOut expansion(EXPANSION_PIN, false);
pros::ADIDigitalIn intake_limit(LIMIT_PIN);
pros::ADIDigitalOut expansion_blocker(EXPANSION_BLOCKER, false);
pros::Rotation rotl(4, false); // port 4, not reversed
pros::Rotation rotr(5, true); // port 5, reversed
pros::Rotation rotb(6); //port 6

pros::Motor lb(-18);
pros::Motor lm(-9);
pros::Motor lf(-11);
pros::Motor rb(14);
pros::Motor rm(13);
pros::Motor rf(1);

pros::MotorGroup leftdrive({lb, lm, lf});
pros::MotorGroup rightdrive({rb,rm,rf});


lemlib::TrackingWheel left_tracking_wheel(&rotl, 2.75, 4.3); // 2.75" wheel diameter, -4.6" offset from tracking center
lemlib::TrackingWheel right_tracking_wheel(&rotr, 2.75, 4.3);
lemlib::TrackingWheel back_tracking_wheel(&rotb, 2.75, 4.3);

lemlib::TrackingWheel left_drive(&leftdrive, 4.125, 6.25, 600 );
lemlib::TrackingWheel right_drive(&rightdrive, 4.125, 6.25, 600 );

pros::Imu inertial_sensor(21); // port 21


// odometry struct
lemlib::OdomSensors_t sensors {
    //&left_tracking_wheel, // vertical tracking wheel 1
    //&right_tracking_wheel, // vertical tracking wheel 2
    //&back_tracking_wheel, // horizontal tracking wheel 1
	&left_drive,
	&right_drive,
	nullptr,
    nullptr, // we don't have a second tracking wheel, so we set it to nullptr
    &inertial_sensor // inertial sensor
};
// forward/backward PID
lemlib::ChassisController_t lateralController {
    10, // kP
    30, // kD
    1, // smallErrorRange
    100, // smallErrorTimeout
    3, // largeErrorRange
    500, // largeErrorTimeout
    5 // slew rate
};
 
// turning PID
lemlib::ChassisController_t angularController {
    2, // kP
    10, // kD
    1, // smallErrorRange
    100, // smallErrorTimeout
    3, // largeErrorRange
    500, // largeErrorTimeout
    5 // slew rate
};
lemlib::Drivetrain_t drivetrain {
    &leftdrive, // left drivetrain motors
    &rightdrive, // right drivetrain motors
    13, // track width
    4.125, // wheel diameter
    360 // wheel rpm
};
//BASE
extern lemlib::Chassis chassis(drivetrain, lateralController, angularController, sensors);

//----------------------------------------------------------------------------------------------------------------------------------------

void screen() {
    // loop forever
    while (true) {
        lemlib::Pose pose = chassis.getPose(); // get the current position of the robot
        pros::lcd::print(0, "x: %f", pose.x); // print the x position
        pros::lcd::print(1, "y: %f", pose.y); // print the y position
        pros::lcd::print(2, "heading: %f", pose.theta); // print the heading
        pros::delay(10);
    }
}
 

void initialize() {
	chassis.calibrate();
	chassis.setPose(0, 0, 0); // X: 0, Y: 0, Heading: 0
	pros::Task screenTask(screen);
}


void disabled() {}


void competition_initialize() {}



//AUTONOMOUS FUNCTIONS--------------------------------------------------------
void left_auton(){
    chassis.turnTo(-20, 32, 1500, true);
}

void right_auton(){

}

void solo_awp(){

}

void autonomous() {
	left_auton();
}


void opcontrol() {
	double intake_mode;
	while(true){

		//drive tank
		if(master.get_analog(ANALOG_LEFT_Y) < 10){
			leftdrive.move(0);
		}else{
			leftdrive.move(master.get_analog(ANALOG_LEFT_Y));
		}
		if(master.get_analog(ANALOG_RIGHT_X) < 10){
			rightdrive.move(0);
		}else{
			rightdrive.move(master.get_analog(ANALOG_RIGHT_X));
		}

		//EXPANSION/EXPANSIONBLOCKER
		if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)){
			expansion.set_value(true);
			expansion_blocker.set_value(true);
		}
		
		//INTAKE/OUTTTAKE
		if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)){
			
				intake.move_voltage(12000);
				intake_mode = 0;
			

		}
		if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)){
			
			intake.move_voltage(-12000);
			pros::delay(3000);
			intake.move_voltage(0);
			
		}

		if(intake_limit.get_value() == true){
			intake.move_voltage(0);
		}
	}
	
}
