#include "main.h"



//BASE
Drive robotchassis (
  // Left Chassis Ports (negative port will reverse it!)
  {-11,-9,-18},

  // Right Chassis Ports (negative port will reverse it!)
  {1, 13, 14}

  // IMU Port
  ,21

  // Tracking Wheel Diameter (Remember, 4" wheels are actually 4.125!)
  ,4.125

  // Ticks per Rotation of Encoder
  ,600

  // External Gear Ratio of Tracking Wheel (MUST BE DECIMAL)
  // eg. if your drive is 84:36 where the 36t is sensored, your RATIO would be 2.333.
  // eg. if your drive is 36:60 where the 60t is sensored, your RATIO would be 0.6.
  ,2.333

);

//BASE
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

//ODOMETRY GLOBALS
lemlib::TrackingWheel left_tracking_wheel(&rotl, 2.75, 4.3); // 2.75" wheel diameter, -4.6" offset from tracking center
lemlib::TrackingWheel right_tracking_wheel(&rotr, 2.75, 4.3);
lemlib::TrackingWheel back_tracking_wheel(&rotb, 2.75, 4.3);

lemlib::TrackingWheel left_drive(&leftdrive, 4.125, 6.25, 600 );
lemlib::TrackingWheel right_drive(&rightdrive, 4.125, 6.25, 600 );

pros::Imu inertial_sensor(21); // port 21


//ODOMETRY STRUCT
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
//FORWARD/BACKWARD PID
/*

increase kP until the robot starts oscillating
increase kD until the oscillation stops
record kP and kD values
repeat steps 2-4 until you can't stop the oscillation. At this point, use the last kP and kD values you recorded.

*/
lemlib::ChassisController_t lateralController {
    10, // kP
    30, // kD
    1, // smallErrorRange
    100, // smallErrorTimeout
    3, // largeErrorRange
    500, // largeErrorTimeout
    5 // slew rate
};
 
//TURNING PID
lemlib::ChassisController_t angularController {
    2, // kP
    10, // kD
    1, // smallErrorRange
    100, // smallErrorTimeout
    3, // largeErrorRange
    500, // largeErrorTimeout
    5 // slew rate
};

//DRIVETRAIN
lemlib::Drivetrain_t drivetrain {
    &leftdrive, // left drivetrain motors
    &rightdrive, // right drivetrain motors
    13, // track width
    4.125, // wheel diameter
    600 // wheel rpm
};

//BASE
extern lemlib::Chassis chassis(drivetrain, lateralController, angularController, sensors);

//SCREEN VITALS
extern void screen() {
	while(true){
		pros::Controller master(pros::E_CONTROLLER_MASTER);
		int battery_level = pros::battery::get_current();
		double battery_temp = pros::battery::get_temperature();
		double controller_battery = master.get_battery_level();
		double imu_heading = inertial_sensor.get_heading();
		double imu_status = inertial_sensor.get_status();
		double radio_connectivity = master.is_connected();
        lemlib::Pose pose = chassis.getPose(); // get the current position of the robot
        pros::lcd::print(0, "x: %f", pose.x); // print the x position
        pros::lcd::print(1, "y: %f", pose.y); // print the y position
        pros::lcd::print(2, "heading: %f", pose.theta); // print the heading
		pros::lcd::print(3, "battery level: %f", battery_level);
		pros::lcd::print(4, "battery temp: %f", battery_temp);
		pros::lcd::print(5, "controller-battery-level: %f", controller_battery);
		pros::lcd::print(6, "imu-heading: %f", imu_heading);
		pros::lcd::print(7, "imu-status: %f", imu_status);
		pros::lcd::print(8, "controller is connected.", radio_connectivity);
        pros::delay(10);
		std::cout<<battery_level<<"battery_level";
	}
}


//DEFINES
#define INTAKE_MOTOR 15
#define EXPANSION_PIN 'A'
#define LIMIT_PIN 'E'
#define EXPANSION_BLOCKER 'C'

//GLOBALS
pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::Motor intake(INTAKE_MOTOR, pros::E_MOTOR_GEARSET_18);
pros::ADIDigitalOut expansion(EXPANSION_PIN, false);
pros::ADIDigitalIn intake_limit(LIMIT_PIN);
pros::ADIDigitalOut expansion_blocker(EXPANSION_BLOCKER, false);


//AUTONOMOUS FUNCTIONS--------------------------------------------------------
void left_auton(){
	pros::Task screenTask(screen);
    chassis.turnTo(10, 10, 1500, true);

}

void right_auton(){
	pros::Task screenTask(screen);
	chassis.moveTo(10,10,400);
}

void solo_awp(){

}

//----------------------------------------------------------------------------------------------------------------------------------------

void initialize() {

	chassis.calibrate();
	chassis.setPose(0, 0, 0); // X: 0, Y: 0, Heading: 0


	robotchassis.toggle_modify_curve_with_controller(true); // Enables modifying the controller curve with buttons on the joysticks
  	robotchassis.set_active_brake(0.2); // Sets the active brake kP. We recommend 0.1.
  	robotchassis.set_curve_default(0.1, 0); // Defaults for curve. If using tank, only the first parameter is used. (Comment this line out if you have an SD card!)  

  // These are already defaulted to these buttons, but you can change the left/right curve buttons here!
  	robotchassis.set_left_curve_buttons (pros::E_CONTROLLER_DIGITAL_LEFT, pros::E_CONTROLLER_DIGITAL_RIGHT); // If using tank, only the left side is used. 
  	robotchassis.set_right_curve_buttons(pros::E_CONTROLLER_DIGITAL_Y,    pros::E_CONTROLLER_DIGITAL_A);
	ez::as::initialize();
	ez::as::auton_selector.add_autons(
    {
      Auton("left autonomous", left_auton),
	  Auton("right autonomous", right_auton),
	  Auton("solo_awp", solo_awp),
    });

  
}





void disabled() {
	ez::as::auton_selector.add_autons(
    {
      Auton("left autonomous", left_auton),
	  Auton("right autonomous", right_auton),
	  Auton("solo_awp", solo_awp),
    });
}


void competition_initialize() {
	ez::as::auton_selector.call_selected_auton();

}



//--------------------------------------------------------------------------------------------------------------------------------------------------------------------


void autonomous() {
	ez::as::auton_selector.call_selected_auton();

}

//OP-CONTROL

void opcontrol() {
	double intake_mode;
	auto time = pros::c::millis();
	while(true){
		lemlib::Pose pose = chassis.getPose(); // get the current position of the robot

		robotchassis.tank();
		//EXPANSION/EXPANSIONBLOCKER
		if(time == time + 10000){
			if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)){
			expansion.set_value(true);
			expansion_blocker.set_value(true);
			}
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
		if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)){
			intake.move_voltage(-12000);
			pros::delay(200);
			intake.move_voltage(0);
			pros::delay(10);
			intake.move_voltage(-12000);
			pros::delay(200);
			intake.move_voltage(0);
			pros::delay(10);
			intake.move_voltage(-12000);
			pros::delay(200);
			intake.move_voltage(0);
		}

		/* limit
		if(intake_limit.get_value() == true){
			intake.move_voltage(0);
		}
		*/



		//LOGGING
		pros::Task screentask(screen);
	}
	
}
