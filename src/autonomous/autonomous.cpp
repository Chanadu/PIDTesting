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
	Devices::chassis.moveToPoint(0, 48, 100000);
}

void autonomousRunner() {
	clearScreen();

	constexpr AutonTypes autonType = BP;

	short lineNumber = 0;

	pros::lcd::print(lineNumber++, "AUTON RUNNING %s", autonTypeString.at(autonType));
	Config::controllerStrings[1] = fmt::format("AUTON {}", autonTypeString.at(autonType));

	batteryDisplay(lineNumber);
	chassisPositionDisplay(lineNumber);

	runAuton(autonType);
}
void runAuton(const AutonTypes autonType) {
	using Devices::chassis;
	using Devices::holderPiston;
	using Devices::intakeMotorGroup;

	Config::controllerStrings[2] = fmt::format("Started {}", autonTypeString.at(autonType));
	pros::lcd::print(2, "Started %s", autonTypeString.at(autonType));

	// const std::map<AutonTypes, std::vector<asset>> autonAssets = {
	// 	{BP, {}},
	// 	{BN, {}},
	// 	{RP, {RPPH1B_txt, RP2_txt, RPPH3_txt, RP4_txt}},
	// 	{RN, {RNPH1B_txt, RN2_txt, RN3_txt, RN4NC_txt}},
	// };

	// chassis.setPose(0, 0, 0);
	intakeMotorGroup.move(0);
	holderPiston.extend();

	switch (autonType) {
		case BP:
			Devices::leftMotorGroup.move(-75);
			Devices::rightMotorGroup.move(-75);
			pros::delay(750);
			Devices::leftMotorGroup.move(0);
			Devices::rightMotorGroup.move(0);
			holderPiston.retract();
			intakeMotorGroup.move(-127);

			// // chassis.moveToPoint(24, 0, 5000, {.forwards = false}, true);
			// pros::delay(500);
			// holderPiston.retract();
			// intakeMotorGroup.move(127);
			// chassis.follow(autonAssets.at(BP)[0], 15, 2000, false, false);

			// holderPiston.extend();
			// intakeMotorGroup.move(127);

			// chassis.follow(autonAssets.at(BP)[1], 15, 2000, true, false);

			// intakeMotorGroup.move(0);
			// chassis.follow(autonAssets.at(BP)[2], 15, 2000, true, false);

			// holderPiston.extend();
			// intakeMotorGroup.move(127);

			// chassis.follow(autonAssets.at(BP)[3], 15, 2000, true, false);
			// intakeMotorGroup.move(0);
			// // chassis.follow(autonAssets.at(RP)[1], 15, 2000, false, false);
			// break;
			// case BN:
			// 	chassis.follow(autonAssets.at(BN)[0], 15, 2000, false, false);
			// 	holderPiston.extend();
			// 	chassis.follow(autonAssets.at(BN)[1], 15, 2000, true, false);
			// 	intakeMotorGroup.move(127);
			// 	chassis.follow(autonAssets.at(BN)[2], 15, 2000, true, false);
			// 	chassis.follow(autonAssets.at(BN)[3], 15, 2000, true, false);
			// 	intakeMotorGroup.move(0);
			// 	// chassis.follow(autonAssets.at(RN)[1], 15, 2000, false, false);
			// 	break;
			// case RP:
			// 	chassis.follow(autonAssets.at(RP)[0], 15, 2000, false, false);

			// 	holderPiston.extend();
			// 	intakeMotorGroup.move(127);

			// 	chassis.follow(autonAssets.at(RP)[1], 15, 2000, true, false);

			// 	intakeMotorGroup.move(0);
			// 	chassis.follow(autonAssets.at(RP)[2], 15, 2000, true, false);

			// 	holderPiston.extend();
			// 	intakeMotorGroup.move(127);

			// 	chassis.follow(autonAssets.at(RP)[3], 15, 2000, true, false);
			// 	intakeMotorGroup.move(0);
			// 	// chassis.follow(autonAssets.at(RP)[1], 15, 2000, false, false);
			// 	break;
			// case RN:
			// 	chassis.follow(autonAssets.at(RN)[0], 15, 2000, false, false);
			// 	holderPiston.extend();
			// 	chassis.follow(autonAssets.at(RN)[1], 15, 2000, true, false);
			// 	intakeMotorGroup.move(127);
			// 	chassis.follow(autonAssets.at(RN)[2], 15, 2000, true, false);
			// 	chassis.follow(autonAssets.at(RN)[3], 15, 2000, true, false);
			// 	intakeMotorGroup.move(0);
			// 	// chassis.follow(autonAssets.at(RN)[1], 15, 2000, false, false);
			// 	break;
			// case TA:
			// 	tuneAngularPID();
			// 	break;
			// case TL:
			// 	tuneLateralPID();
			// 	break;
	}
}
