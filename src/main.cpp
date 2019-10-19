#include "main.h"

#pragma region "Definitions"
//Polling rate - used for calculations for gentler DR4B/drive
const float POLL_RATE = 20.0;

//Controller(s?)
pros::Controller puppeteer(pros::E_CONTROLLER_MASTER);

//Drive motors
pros::Motor drive_FR(18, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor drive_FL(17, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor drive_BR(16, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor drive_BL(15, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
int leftPower = 0;
int rightPower = 0;

//DR4B motors
pros::Motor DR4B_L(2, pros::E_MOTOR_GEARSET_36, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor DR4B_R(1, pros::E_MOTOR_GEARSET_36, false, pros::E_MOTOR_ENCODER_DEGREES);
const double DR4B_ACCEL = 10.0;

double DR4BOffset = 0;
double DR4BVelocity = 0;

//Claw - TODO: Change for 36:1 gearset
pros::Motor claw(10, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
bool claw_open = false;
const float CLAW_MAX_ROTATION = 215;
//Spool - TODO: Change for 36:1 gearset
pros::Motor spool(3, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);

//Helper functions
int limitAbs(int n, int max)
{
	int sign;
	if (n > 0)
	{
		sign == 1;
	}
	else if (n < 0)
	{
		sign == -1;
	}
	else
	{
		return 0;
	}
	return std::min(max, abs(n)) * sign;
}

double lerp(double a, double b, double t){
	return a + t * (b - a);
}
#pragma endregion
/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */

void initialize()
{
	//All drive motors coast - reduce jerk
	drive_FR.set_brake_mode(MOTOR_BRAKE_COAST);
	drive_FL.set_brake_mode(MOTOR_BRAKE_COAST);
	drive_BR.set_brake_mode(MOTOR_BRAKE_COAST);
	drive_BL.set_brake_mode(MOTOR_BRAKE_COAST);

	//DR4B set to hold - more stability
	DR4B_L.set_brake_mode(MOTOR_BRAKE_HOLD);
	DR4B_R.set_brake_mode(MOTOR_BRAKE_HOLD);

	//Claw set to coast - rubber bands provide most of the clamping force
	claw.set_brake_mode(MOTOR_BRAKE_COAST);

	//Spool set to hold - more constant pressure
	spool.set_brake_mode(MOTOR_BRAKE_HOLD);
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
	//Deploy claw
	drive_FR.move(127);
	drive_FL.move(127);
	drive_BR.move(127);
	drive_BL.move(127);
	pros::delay(250);
	drive_FR.move(-127);
	drive_FL.move(-127);
	drive_BR.move(-127);
	drive_BL.move(-127);
	pros::delay(200);
	drive_FR.move(0);
	drive_FL.move(0);
	drive_BR.move(0);
	drive_BL.move(0);
	
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
void opcontrol()
{

	while (true)
	{
		//Claw: open w/ A button
		if(puppeteer.get_digital(pros::E_CONTROLLER_DIGITAL_A) && claw.get_position() < CLAW_MAX_ROTATION){
			claw.move(127);
		}
		else{
			claw.move(0);
		}
		//DR4B: move w/ up/down directional buttons
		//If the two sides become offset, the side that is ahead slows down to compensate.
		DR4BOffset = DR4B_L.get_position() - DR4B_R.get_position();
		if (puppeteer.get_digital(pros::E_CONTROLLER_DIGITAL_UP))
		{
			// DR4BVelocity = lerp(DR4BVelocity, 127, (POLL_RATE * DR4B_ACCEL));

			DR4B_L.move(std::min((127 + DR4BOffset), 127.0));
			DR4B_R.move(std::min((127 - DR4BOffset), 127.0));
		}
		else if (puppeteer.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN))
		{
			// DR4BVelocity = lerp(DR4BVelocity, -127, (POLL_RATE * DR4B_ACCEL));
			
			DR4B_L.move(std::max((-64 - DR4BOffset), -64.0));
			DR4B_R.move(std::max((-64 + DR4BOffset), -64.0));
		}
		else
		{
			DR4B_L.move(-DR4BOffset);
			DR4B_R.move(DR4BOffset);
		}

		//Drive: Arcade drive split on two sticks: forward/back on left, turning on right
		leftPower = puppeteer.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) + puppeteer.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
		rightPower = puppeteer.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) - puppeteer.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

		drive_FR.move(rightPower);
		drive_BR.move(rightPower);

		drive_FL.move(leftPower);
		drive_BL.move(leftPower);

		pros::delay(POLL_RATE);
	}
}
