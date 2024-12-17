#include "autonomous/autonomous.hpp"
#include "main.h"
#include "utils/configuration.hpp"
#include "utils/devices.hpp"
#include "utils/screen-displays.hpp"

// https://lemlib.readthedocs.io/en/stable/tutorials/4_pid_tuning.html
void tuneAngularPID() {
	Devices::chassis.turnToHeading(90, 100000);
}

void tuneLateralPID() {
	Devices::chassis.moveToPoint(48, 0, 100000);
	// Devices::chassis.moveToPoint(0, 48, 100000);
}

void runAuton(const AutonTypes autonType) {
	using Devices::chassis;
	using Devices::holderPiston;
	using Devices::intakeMotorGroup;

	// ReSharper disable once CppExpressionWithoutSideEffects
	intakeMotorGroup.move(0);
	holderPiston.extend();

	chassis.setPose(0, 0, 0);
	if (autonType == TL) {
		tuneLateralPID();
	} else if (autonType == TA) {
		tuneAngularPID();
	}
}

void autonomousRunner() {
	pros::Task task{[=] {
		clearScreen();

		short lineNumber = 0;
		batteryDisplay(lineNumber);
		chassisPositionDisplay(lineNumber);
		drivetrainPositionDisplay(lineNumber);
		drivetrainVelocityDisplay(lineNumber);
		pros::delay(50);
	}};

	constexpr AutonTypes autonType = TL;

	short lineNumber = 5;

	pros::lcd::print(lineNumber++, "AUTON RUNNING %s", autonTypeString.at(autonType));
	// Config::controllerStrings[1] = fmt::format("Running Auton");
	// Config::controllerStrings[2] = fmt::format("AUTON {}", autonTypeString.at(autonType));

	runAuton(autonType);
}
