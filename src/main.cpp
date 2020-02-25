#include "main.h"
#pragma region Definitions
//Polling rate - used for calculations for gentler DR4B/drive
const float POLL_RATE = 20.0;

//Controller(s?)
Controller puppeteer(E_CONTROLLER_MASTER);

//Drive motors
Motor driveH(15, E_MOTOR_GEARSET_18, true, E_MOTOR_ENCODER_DEGREES);
Motor driveFR(14, E_MOTOR_GEARSET_18, true, E_MOTOR_ENCODER_DEGREES);
Motor driveFL(13, E_MOTOR_GEARSET_18, false, E_MOTOR_ENCODER_DEGREES);
Motor driveBR(12, E_MOTOR_GEARSET_18, true, E_MOTOR_ENCODER_DEGREES);
Motor driveBL(11, E_MOTOR_GEARSET_18, false, E_MOTOR_ENCODER_DEGREES);
bool slowedMovement = false;

int leftPower = 0;
int rightPower = 0;
/*
	Maps from (-127) -> 127 to (-100) -> 100 using the function
	((100 * pow(4, ((abs(x) - 50) / 12.5))) / (pow(4, ((abs(x) - 50) / 12.5))+1))) * ((x > 0) - (x < 0))
	https://www.desmos.com/calculator/w7dktkaote

	Used to ease in/out joystick movement for more precise control.

	If the horizontal input is at 25%, it only has 5% of the power, reducing the
	amount that the robot will slowly veer off course from an imperfect control
	stick.
*/
int sigmoid_map[255] = {-100, -100, -100, -100, -100, -100, -100, -100, -100, -100, -100, -100, -100, -100, -100, -100, -99, -99, -99, -99, -99, -99, -99, -99, -99, -99, -99, -99, -99, -99, -99, -99, -99, -99, -99, -99, -99, -98, -98, -98, -98, -98, -98, -97, -97, -97, -96, -96, -96, -95, -95, -94, -94, -93, -92, -92, -91, -90, -89, -88, -86, -85, -84, -82, -80, -79, -77, -75, -73, -70, -68, -66, -63, -61, -58, -55, -52, -50, -47, -44, -41, -39, -36, -34, -31, -29, -27, -24, -22, -21, -19, -17, -16, -14, -13, -12, -10, -9, -8, -8, -7, -6, -5, -5, -4, -4, -3, -3, -3, -2, -2, -2, -2, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 4, 4, 5, 5, 6, 7, 8, 8, 9, 10, 12, 13, 14, 16, 17, 19, 21, 22, 24, 27, 29, 31, 34, 36, 39, 41, 44, 47, 50, 52, 55, 58, 61, 63, 66, 68, 70, 73, 75, 77, 79, 80, 82, 84, 85, 86, 88, 89, 90, 91, 92, 92, 93, 94, 94, 95, 95, 96, 96, 96, 97, 97, 97, 98, 98, 98, 98, 98, 98, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100};

//DR4B motors
Motor DR4BL(4, E_MOTOR_GEARSET_36, true, E_MOTOR_ENCODER_DEGREES);
Motor DR4BR(3, E_MOTOR_GEARSET_36, false, E_MOTOR_ENCODER_DEGREES);
const double DR4B_ACCEL = 10.0;
const double DR4B_MAX = 595.0;
double DR4BOffset = 0;
double DR4BVelocity = 0;

//Intake
Motor Intake(2, E_MOTOR_GEARSET_36, true, E_MOTOR_ENCODER_DEGREES);

//Sensors
ADIUltrasonic rangeL(1, 2);
ADIUltrasonic rangeR(3, 4);

//UI
lv_obj_t *lRange = lv_label_create(lv_scr_act(), nullptr);
lv_obj_t *rRange = lv_label_create(lv_scr_act(), nullptr);
#pragma endregion

#pragma region Helper functions
template <class T>
T limitAbs(T n, T max)
{
	if (n > 0)
	{
		if (n > max)
		{
			return max;
		}
		return n;
	}
	else if (n < 0)
	{
		if (n < -max)
		{
			return -max;
		}
		return n;
	}
	else
	{
		return 0;
	}
}

template <class T>
T lerp(T a, T b, T w)
{
	return a + w * (b - a);
}
#pragma endregion

#pragma region Autonomous helper functions
/** Generalized PID controler for use with more specialized functions.
 * \param SETPOINT Target sensor value used to calculate error
 * \param SENSOR_VALUE Sensor value used to calculate error
 * \param integral Variable used to store integral for future iterations
 * \param prevError Variable used to store the previous error to calculate derivative for future iterations
 * \param KP Proportional multiplier constant
 * \param KI Integral multiplier constant
 * \param KD Derivative multiplier constant
 * \return Value to be passed to motors
 */
double PID(const double SETPOINT, const double SENSOR_VALUE, double &integral, double &prevError, const double KP, const double KI = 0, const double KD = 0)
{
	const double ERROR = SETPOINT - SENSOR_VALUE;
	double derivative = 0;

	if (KD != 0)
	{
		derivative = ERROR - prevError;
	}
	if (KI != 0)
	{
		integral += ERROR;
	}
	prevError = ERROR;
	return (ERROR * KP) + (integral * KI) + (derivative * KP);
}
/** Moves a certain amount of degrees for left and right using a PID controller
 * \param LEFT_AMT Number of degrees to turn the left wheels
 * \param RIGHT_AMT Number of degrees to turn the right wheels
 * \param RPM Max RPM to spin the wheels at
 * \param TOLERANCE Maximum acceptable error in degrees
 * \param TARGET_TIME Amount of time the wheels have to be within the tolerance in ms
 * \param KP_DRIVE kP for drive motors
 * \param KI_DRIVE kI for drive motors
 * \param KD_DRIVE kD for drive motors
 */
void PIDMove(const double LEFT_AMT, const double RIGHT_AMT, const double RPM, const double TOLERANCE, const int TARGET_TIME, const double KP_DRIVE, const double KI_DRIVE = 0, const double KD_DRIVE = 0)
{
	const double FR_TARGET = driveFR.get_position() + RIGHT_AMT;
	const double FL_TARGET = driveFL.get_position() + LEFT_AMT;
	const double BR_TARGET = driveBR.get_position() + RIGHT_AMT;
	const double BL_TARGET = driveBL.get_position() + LEFT_AMT;

	double FRIntegral = 0;
	double FLIntegral = 0;
	double BRIntegral = 0;
	double FLIntegral = 0;

	double FRPrevError = 0;
	double FLPrevError = 0;
	double BRPrevError = 0;
	double BLPrevError = 0;

	bool atTarget = false;
	int timeAtTarget = 0;
	while (!atTarget && timeAtTarget <= TARGET_TIME)
	{
		driveFR.move_velocity(limitAbs(PID(FR_TARGET, driveFR.get_position(), FRIntegral, FRPrevError, KP_DRIVE), RPM));
		driveFL.move_velocity(limitAbs(PID(FL_TARGET, driveFL.get_position(), FLIntegral, FLPrevError, KP_DRIVE), RPM));
		driveBR.move_velocity(limitAbs(PID(BR_TARGET, driveBR.get_position(), BRIntegral, BRPrevError, KP_DRIVE), RPM));
		driveBL.move_velocity(limitAbs(PID(BL_TARGET, driveBL.get_position(), FLIntegral, BLPrevError, KP_DRIVE), RPM));

		atTarget = (std::abs(driveFR.get_position() - FR_TARGET) < TOLERANCE) && (std::abs(driveFL.get_position() - FL_TARGET) < TOLERANCE) && (std::abs(driveBR.get_position() - BR_TARGET) < TOLERANCE) && (std::abs(driveBL.get_position() - BL_TARGET) < TOLERANCE);
		if (atTarget)
		{
			timeAtTarget += POLL_RATE;
		}
		else
		{
			timeAtTarget = 0;
		}
		delay(POLL_RATE);
	}
	driveFR.move(0);
	driveFL.move(0);
	driveBR.move(0);
	driveBL.move(0);
}

/** Moves a certain amount forward using ultrasonic rangefinders and a PID controller
 * \param AMT Ultrasonic sensor units to move
 * \param RPM Max RPM to spin the wheels
 * \param TOLERANCE Maximum acceptable rangefinder error
 * \param TARGET_TICKS Amount of time the measurement have to be within the error in ms
 * \param KP_RANGE kP for drive motors
 * \param KI_RANGE kI for drive motors
 * \param KD_RANGE kD for drive motors
 */
void PIDDriveForward(const double AMT, const double RPM, const double TOLERANCE, const double TARGET_TIME, const double KP_RANGE, const double KI_RANGE = 0, const double KD_RANGE = 0, const bool ABSOLUTE = false)
{
	const double L_TARGET = AMT + (ABSOLUTE ? 0 : rangeL.get_value());
	const double R_TARGET = AMT + (ABSOLUTE ? 0 : rangeR.get_value());

	double lIntegral = 0;
	double rIntegral = 0;

	double lPrevError = 0;
	double rPrevError = 0;


	double lPower = 0;
	double rPower = 0;

	int timeAtTarget = 0;

	bool atTarget = false;

	while (!atTarget && timeAtTarget <= TARGET_TIME)
	{
		// lv_label_set_text(lRange, (std::to_string(lPrevError)).c_str());
		// lv_label_set_text(rRange, (std::to_string(rPrevError)).c_str());
		lPower = limitAbs(PID(L_TARGET, rangeL.get_value(), lIntegral, lPrevError, KP_RANGE, KI_RANGE, KD_RANGE), RPM);
		rPower = limitAbs(PID(R_TARGET, rangeR.get_value(), rIntegral, rPrevError, KP_RANGE, KI_RANGE, KD_RANGE), RPM);
		
		driveFR.move_velocity(rPower);
		driveFL.move_velocity(lPower);
		driveBR.move_velocity(rPower);
		driveBL.move_velocity(lPower);

		atTarget = (std::abs(rangeR.get_value() - R_TARGET) < TOLERANCE) && (std::abs(rangeL.get_value() - L_TARGET) < TOLERANCE);
		if (atTarget)
		{
			timeAtTarget += POLL_RATE;
		}
		else
		{
			timeAtTarget = 0;
		}
		delay(POLL_RATE);
	}
	driveFR.move(0);
	driveFL.move(0);
	driveBR.move(0);
	driveBL.move(0);
}

void intakeStack(const double EFF_THRESHOLD, const double ADJUST_TIME, const double MAX_TIME = -1, const double DR4B_TARGET = -40, const double DR4B_TOLERANCE = 1)
{
	double timeWasted = 0;

	while ((timeWasted <= MAX_TIME) && (MAX_TIME > 0))
	{
		lv_label_set_text(lRange, (std::to_string(DR4BR.get_position())).c_str());

		if (Intake.get_efficiency() < EFF_THRESHOLD && timeWasted > 1000)
		{
			Intake.move_velocity(-100);
			delay(ADJUST_TIME);
			timeWasted += ADJUST_TIME;
		}
		else
		{
			Intake.move_velocity(100);
		}
		timeWasted += POLL_RATE;
		delay(POLL_RATE);
	}
	Intake.move_velocity(0);
}

void PIDDriveH(const double AMT, const double RPM, const double TOLERANCE, const double TARGET_TIME, const double KP_RANGE, const double KI_RANGE = 0, const double KD_RANGE = 0)
{
	const double L_TARGET = rangeL.get_value();
	const double R_TARGET = rangeR.get_value();

	const double H_TARGET = driveH.get_position() + AMT;

	double lIntegral = 0;
	double rIntegral = 0;

	double lPrevError = 0;
	double rPrevError = 0;

	int timeAtTarget = 0;

	bool atTarget = false;
	while (!atTarget && timeAtTarget <= TARGET_TIME)
	{

		driveH.move_absolute(H_TARGET, RPM);
		driveFR.move_velocity(limitAbs(PID(R_TARGET, rangeR.get_value(), lIntegral, lPrevError, KP_RANGE, KI_RANGE, KD_RANGE), RPM));
		driveFL.move_velocity(limitAbs(PID(L_TARGET, rangeL.get_value(), rIntegral, rPrevError, KP_RANGE, KI_RANGE, KD_RANGE), RPM));
		driveBR.move_velocity(limitAbs(PID(R_TARGET, rangeR.get_value(), lIntegral, lPrevError, KP_RANGE, KI_RANGE, KD_RANGE), RPM));
		driveBL.move_velocity(limitAbs(PID(L_TARGET, rangeL.get_value(), rIntegral, rPrevError, KP_RANGE, KI_RANGE, KD_RANGE), RPM));

		atTarget = abs(driveH.get_position() - H_TARGET) < TOLERANCE;
		if (atTarget)
		{
			timeAtTarget += POLL_RATE;
		}
		else
		{
			timeAtTarget = 0;
		}
		delay(POLL_RATE);
	}
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

	//Intake set to hold - prevent cubes from being forced out
	Intake.set_brake_mode(MOTOR_BRAKE_HOLD);

	//Separate UI elements
	lv_obj_set_pos(rRange, 0, 100);
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
	// One-point auto
	driveBL.move_velocity(-200);
	driveBR.move_velocity(-200);
	driveFL.move_velocity(-200);
	driveFR.move_velocity(-200);
	delay(4000);
	driveBL.move_velocity(200);
	driveBR.move_velocity(200);
	driveFL.move_velocity(200);
	driveFR.move_velocity(200);
	delay(1000);
	driveBL.move_velocity(0);
	driveBR.move_velocity(0);
	driveFL.move_velocity(0);
	driveFR.move_velocity(0);
	// lv_label_set_text(lRange, (std::to_string(rangeL.get_value())).c_str());
	// lv_label_set_text(rRange, (std::to_string(rangeR.get_value())).c_str());
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
	double timeStart = millis();
	bool rumbled15s = false;
	bool rumbled30s = false;

	while (true)
	{
		lv_label_set_text(lRange, (std::to_string(rangeL.get_value())).c_str());
		lv_label_set_text(rRange, (std::to_string(rangeR.get_value())).c_str());

		// lv_label_set_text(lRange, (std::to_string(rangeL.get_value())).c_str());
		// lv_label_set_text(rRange, (std::to_string(rangeR.get_value())).c_str());
		//Intake/outtake with L/R triggers
		if (puppeteer.get_digital(E_CONTROLLER_DIGITAL_L1))
		{
			Intake.move(127);
		}
		else if (puppeteer.get_digital(E_CONTROLLER_DIGITAL_L2))
		{
			Intake.move(-127);
		}
		else
		{
			Intake.move(0);
		}
		//DR4B: move w/ up/down directional buttons
		//If the two sides become offset, the side that is ahead slows down to compensate.
		DR4BOffset = DR4BL.get_position() - DR4BR.get_position();
		if (abs(DR4BOffset) < 30 && puppeteer.get_digital(E_CONTROLLER_DIGITAL_R2) && DR4BL.get_position() < DR4B_MAX && DR4BR.get_position() < DR4B_MAX)
		{
			// DR4BVelocity = lerp(DR4BVelocity, 127, (POLL_RATE * DR4B_ACCEL));

			DR4BL.move(limitAbs((100 + DR4BOffset), 127.0) * (slowedMovement ? 0.5 : 1));
			DR4BR.move(limitAbs((100 - DR4BOffset), 127.0) * (slowedMovement ? 0.5 : 1));
		}
		else if (abs(DR4BOffset) < 30 && puppeteer.get_digital(E_CONTROLLER_DIGITAL_R1))
		{
			// DR4BVelocity = lerp(DR4BVelocity, -127, (POLL_RATE * DR4B_ACCEL));

			DR4BL.move(limitAbs((-64 - DR4BOffset), 64.0) * (slowedMovement ? 0.5 : 1));
			DR4BR.move(limitAbs((-64 + DR4BOffset), 64.0) * (slowedMovement ? 0.5 : 1));
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
		if (!slowedMovement)
		{
			leftPower = sigmoid_map[puppeteer.get_analog(E_CONTROLLER_ANALOG_LEFT_Y) + 127] + sigmoid_map[puppeteer.get_analog(E_CONTROLLER_ANALOG_RIGHT_X) + 127];
			rightPower = sigmoid_map[puppeteer.get_analog(E_CONTROLLER_ANALOG_LEFT_Y) + 127] - sigmoid_map[puppeteer.get_analog(E_CONTROLLER_ANALOG_RIGHT_X) + 127];
		}
		else
		{
			leftPower = lerp(float(leftPower), float(sigmoid_map[puppeteer.get_analog(E_CONTROLLER_ANALOG_LEFT_Y) + 127] + sigmoid_map[puppeteer.get_analog(E_CONTROLLER_ANALOG_RIGHT_X) + 127]), 1 - (1 / POLL_RATE));
			rightPower = lerp(float(rightPower), float(sigmoid_map[puppeteer.get_analog(E_CONTROLLER_ANALOG_LEFT_Y) + 127] - sigmoid_map[puppeteer.get_analog(E_CONTROLLER_ANALOG_RIGHT_X) + 127]), 1 - (1 / POLL_RATE));
		}
		driveFR.move(rightPower * (slowedMovement ? 0.5 : 1));
		driveBR.move(rightPower * (slowedMovement ? 0.5 : 1));

		driveFL.move(leftPower * (slowedMovement ? 0.5 : 1));
		driveBL.move(leftPower * (slowedMovement ? 0.5 : 1));

		//H-drive on horizontal axis of left control stick
		driveH.move(puppeteer.get_analog(E_CONTROLLER_ANALOG_LEFT_X) * (slowedMovement ? 0.5 : 1));
		//Rumble at 30s remaining
		if (millis() - timeStart > 75 * 1000 && !rumbled30s)
		{
			puppeteer.rumble("- - -");
			rumbled30s = true;
		}

		//Rumble at 15s remaining
		if (millis() - timeStart > 90 * 1000 && !rumbled15s)
		{
			puppeteer.rumble(".. .. ..");
			rumbled15s = true;
		}
		slowedMovement = puppeteer.get_digital(E_CONTROLLER_DIGITAL_A);
		delay(POLL_RATE);
	}
}
