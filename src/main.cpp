#include "main.h"

void disabled() {}
void competition_initialize() {}
void autonomous() {}

const float drivetrainWheelSize = 2.75;
const float distanceToTravelInches = 48;
const float errorRange = 0.5;
const float movementSpeed = 0.5;

pros::Controller controller(pros::E_CONTROLLER_MASTER);
pros::MotorGroup leftMotorGroup({1, -2, 3}, pros::MotorGears::blue);
pros::MotorGroup rightMotorGroup({-4, 5, -8}, pros::MotorGears::blue);
pros::Imu imu(11);

struct PID {
	double kP, kI, kD;
	double target;
	double previousError = 0;
	double integral = 0;
	double antiWindup = 3;
};

double getAveragePosition() {
	std::vector<double> leftPositions = leftMotorGroup.get_position_all();
	std::vector<double> rightPositions = rightMotorGroup.get_position_all();
	double averagePosition = 0;
	for (int i = 0; i < leftPositions.size(); i++) {
		averagePosition += leftPositions[i] + rightPositions[i];
	}
	averagePosition /= leftPositions.size() * 2;
	return averagePosition;
}

double updateDrivePID(PID* pid) {
	// Calculate error
	double error = getAveragePosition() - pid->target;

	// Calculate proportional term
	double proportional = pid->kP * error;

	// Calculate integral term
	pid->integral += pid->kI * error;

	// Calculate derivative term
	double derivative = pid->kD * (error - pid->previousError);

	// Calculate PID output
	double output = proportional + pid->integral + derivative;

	// Update previous error
	pid->previousError = error;

	return output;
}

double updateTurnPID(PID* pid) {
	double angle = imu.get_heading();
	double error = pid->target - angle;

	// Calculate proportional term
	double proportional = pid->kP * error;

	// Calculate integral term
	pid->integral += pid->kI * error;

	// Calculate derivative term
	double derivative = pid->kD * (error - pid->previousError);

	// Calculate PID output
	double output = proportional + pid->integral + derivative;

	// Update previous error
	pid->previousError = error;

	return std::clamp(output, -1.0, 1.0);
}

void opcontrol() {
	leftMotorGroup.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
	rightMotorGroup.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);

	imu.reset();
	leftMotorGroup.tare_position();
	rightMotorGroup.tare_position();

	// Kp, Ki, Kd, target
	PID drivePID = {0.1, 0.1, 0.1, 360 * (distanceToTravelInches / drivetrainWheelSize)};
	PID turnPID = {0.1, 0.1, 0.1, 0};

	while (true) {
		pros::lcd::print(0, "Started");

		double drive = updateDrivePID(&drivePID) * 12000 * movementSpeed;
		double turn = updateTurnPID(&turnPID) * 12000 * movementSpeed;

		if (getAveragePosition() <= drivePID.target + errorRange &&
			getAveragePosition() >= drivePID.target - errorRange) {
			drive = 0;
			turn = 0;
		}

		pros::lcd::print(1, "Position: %f", getAveragePosition());
		pros::lcd::print(2, "D: %f, T: %f", drive, turn);

		// Drive
		leftMotorGroup.move_voltage(drive + turn);
		rightMotorGroup.move_voltage(drive - turn);

		pros::delay(20);
	}
}
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");
}
