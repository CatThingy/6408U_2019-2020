#include "main.h"

#pragma region "Definitions"
//Polling rate - used for calculations for gentler DR4B/drive
const float POLL_RATE = 20.0;

//Controller(s?)
pros::Controller puppeteer(pros::E_CONTROLLER_MASTER);

//Drive motors

pros::Motor driveH(15, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor driveFR(14, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor driveFL(13, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor driveBR(12, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor driveBL(11, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
bool slowedMovement = false;

int leftPower = 0;
int rightPower = 0;
/*
    Maps from (-100) -> 100 to itself using the function
    ((100 * pow(4, ((abs(x) - 50) / 12.5))) / (pow(4, ((abs(x) - 50) / 12.5))+1))) * ((x > 0) - (x < 0))

    Used to ease in/out joystick movement for more precise control.

    If the horizontal input is at 25%, it only has 5% of the power, reducing the
    amount that the robot will slowly veer off course from an imperfect control
    stick.
*/
int sigmoid_map[255] = {-100, -100, -100, -100, -100, -100, -100, -100, -100, -100, -100, -100, -100, -100, -100, -100, -99, -99, -99, -99, -99, -99, -99, -99, -99, -99, -99, -99, -99, -99, -99, -99, -99, -99, -99, -99, -99, -98, -98, -98, -98, -98, -98, -97, -97, -97, -96, -96, -96, -95, -95, -94, -94, -93, -92, -92, -91, -90, -89, -88, -86, -85, -84, -82, -80, -79, -77, -75, -73, -70, -68, -66, -63, -61, -58, -55, -52, -50, -47, -44, -41, -39, -36, -34, -31, -29, -27, -24, -22, -21, -19, -17, -16, -14, -13, -12, -10, -9, -8, -8, -7, -6, -5, -5, -4, -4, -3, -3, -3, -2, -2, -2, -2, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 4, 4, 5, 5, 6, 7, 8, 8, 9, 10, 12, 13, 14, 16, 17, 19, 21, 22, 24, 27, 29, 31, 34, 36, 39, 41, 44, 47, 50, 52, 55, 58, 61, 63, 66, 68, 70, 73, 75, 77, 79, 80, 82, 84, 85, 86, 88, 89, 90, 91, 92, 92, 93, 94, 94, 95, 95, 96, 96, 96, 97, 97, 97, 98, 98, 98, 98, 98, 98, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100};

//DR4B motors
pros::Motor DR4BL(9, pros::E_MOTOR_GEARSET_36, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor DR4BR(10, pros::E_MOTOR_GEARSET_36, false, pros::E_MOTOR_ENCODER_DEGREES);
const double DR4B_ACCEL = 10.0;
const double DR4B_MAX = 595.0;
double DR4BOffset = 0;
double DR4BVelocity = 0;

//Claw
pros::Motor claw(2, pros::E_MOTOR_GEARSET_36, true, pros::E_MOTOR_ENCODER_DEGREES);
bool claw_open = false;
const float CLAW_MAX_ROTATION = 360;

//Helper functions
template <class T>
T limitAbs(T n, T max)
{
	T sign;
	if (n > 0)
	{
		sign = 1;
	}
	else if (n < 0)
	{
		sign = -1;
	}
	else
	{
		return 0;
	}
	return std::min(max, abs(n)) * sign;
}

template <class T>
T lerp(T a, T b, T w)
{
	return a + w * (b - a);
}

//Generalized PID controller
double PID(double setpoint, double sensorValue, double &integral, double &prevError, double kP, double kI = 0, double kD = 0)
{
	double error = setpoint - sensorValue;
	double derivative = error - prevError;

	integral += error;
	prevError = error;
	return (error * kP) + (integral * kI) + (derivative * kP);
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
	//All drive motors hold - prevent being pushed around
	driveFR.set_brake_mode(MOTOR_BRAKE_HOLD);
	driveFL.set_brake_mode(MOTOR_BRAKE_HOLD);
	driveBR.set_brake_mode(MOTOR_BRAKE_HOLD);
	driveBL.set_brake_mode(MOTOR_BRAKE_HOLD);
	driveH.set_brake_mode(MOTOR_BRAKE_HOLD);

	//DR4B set to hold - more stability
	DR4BL.set_brake_mode(MOTOR_BRAKE_HOLD);
	DR4BR.set_brake_mode(MOTOR_BRAKE_HOLD);

	//Claw set to hold - prevent claw from being forced open
	claw.set_brake_mode(MOTOR_BRAKE_HOLD);
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
	driveFR.move(127);
	driveFL.move(127);
	driveBR.move(127);
	driveBL.move(127);
	pros::delay(250);
	driveFR.move(-127);
	driveFL.move(-127);
	driveBR.move(-127);
	driveBL.move(-127);
	pros::delay(200);
	driveFR.move(0);
	driveFL.move(0);
	driveBR.move(0);
	driveBL.move(0);

	//drive forward
	//raise dr4b
	//drive forward
	//lower dr4b
	//open claw
	//lower dr4b to ground
	//close claw
	//turn
	//drive to goal
	//open claw
	//raise dr4b
	//back off
	//lower dr4b
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
		//Claw: open/close w/ right triggers
		if (puppeteer.get_digital(pros::E_CONTROLLER_DIGITAL_L1) && claw.get_position() < CLAW_MAX_ROTATION)
		{
			claw.move(127);
		}
		else if (puppeteer.get_digital(pros::E_CONTROLLER_DIGITAL_L2) && claw.get_position() > -30)
		{
			claw.move(-127);
		}
		else
		{
			claw.move(0);
		}
		//DR4B: move w/ up/down directional buttons
		//If the two sides become offset, the side that is ahead slows down to compensate.
		DR4BOffset = DR4BL.get_position() - DR4BR.get_position();
		if (puppeteer.get_digital(pros::E_CONTROLLER_DIGITAL_R2) && DR4BL.get_position() < DR4B_MAX && DR4BR.get_position() < DR4B_MAX)
		{
			// DR4BVelocity = lerp(DR4BVelocity, 127, (POLL_RATE * DR4B_ACCEL));

			DR4BL.move(std::min((127 + DR4BOffset), 127.0) * (slowedMovement ? 0.5 : 1));
			DR4BR.move(std::min((127 - DR4BOffset), 127.0) * (slowedMovement ? 0.5 : 1));
		}
		else if (puppeteer.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
		{
			// DR4BVelocity = lerp(DR4BVelocity, -127, (POLL_RATE * DR4B_ACCEL));

			DR4BL.move(std::max((-64 - DR4BOffset), -64.0));
			DR4BR.move(std::max((-64 + DR4BOffset), -64.0));
		}
		else
		{
			if (DR4BOffset > 0)
			{
				DR4BL.move(0);
				DR4BR.move(DR4BOffset * 2);
			}
			else
			{
				DR4BL.move(-DR4BOffset * 2);
				DR4BR.move(0);
			}
		}

		//Drive: Arcade drive split on two sticks: forward/back on left, turning on right
		leftPower = sigmoid_map[puppeteer.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) + 127] + sigmoid_map[puppeteer.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X) + 127];
		rightPower = sigmoid_map[puppeteer.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) + 127] - sigmoid_map[puppeteer.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X) + 127];

		driveFR.move(rightPower * (slowedMovement ? 0.5 : 1));
		driveBR.move(rightPower * (slowedMovement ? 0.5 : 1));

		driveFL.move(leftPower * (slowedMovement ? 0.5 : 1));
		driveBL.move(leftPower * (slowedMovement ? 0.5 : 1));

		//H-drive on horizontal axis of left control stick
		driveH.move(puppeteer.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X) * (slowedMovement ? 0.5 : 1));

		slowedMovement = puppeteer.get_digital(pros::E_CONTROLLER_DIGITAL_A);
		pros::delay(POLL_RATE);
	}
}
