#include "utils/configuration.hpp"
#include "main.h"

namespace Config {
const float drivetrainTrackWidth = 14.975;
const float drivetrainWheelbaseWidth = 9.0;
const float drivetrainWheelRPM = 600;
const float drivetrainHorizontalDrift = 2.0;

double intakeVelocity = 1.0;

const float drivetrainWheelType = lemlib::Omniwheel::NEW_275;

const float maxControllerValue = 127.0;

double movementVelocityPercentage = 1.0;
double turningVelocityPercentage = 1.0;

// https://www.desmos.com/calculator/umicbymbnl
float joystickThresholdPercentage = 3.0;
float outputMovementThresholdPercentage = 10.0;

float lateralCurveGain = 1.019;
float angularCurveGain = 1.019;

float angularKu = 0.7;
float angularTuSeconds = 0.75;

float lateralKu = 0;
float lateralTuSeconds = 0;

PIDTypes currentlyUsingPIDType = PIDTypes::P;
/*
 * Ziegler–Nichols method
 * Control Type:		Kp		Ti		Td		Ki		Kd
 * P					0.5Ku	-		-		-		-
 * PI					0.45Ku	0.85Tu	-		0.27Ku	-
 */

// lateral PID controller
lemlib::ControllerSettings lateralMovementController(0.1,  // ProportionalGain
													 0,	   // IntegralGain
													 0,	   // DerivativeGain
													 3,	   // AntiWindup
													 1,	   // SmallErrorRange
													 100,  // SmallErrorRangeTimeout
													 3,	   // LargeErrorRange
													 500,  // LargeErrorRangeTimeout
													 20	   // MaximumAcceleration
);

lemlib::ControllerSettings angularMovementController(0.1,  // ProportionalGain
													 0,	   // IntegralGain
													 0,	   // DerivativeGain
													 3,	   // AntiWindup
													 1,	   // SmallErrorRange
													 100,  // SmallErrorRangeTimeout
													 3,	   // LargeErrorRange
													 500,  // LargeErrorRangeTimeout
													 20	   // MaximumAcceleration
);

DrivetrainMovement drivetrainMovement = DrivetrainMovement::DoubleStickArcade;
std::string controllerStrings[3] = {
	"Line 1",
	"Line 2",
	"Line 3",
};
std::string controllerRumblePattern;

lemlib::ControllerSettings createControllerSettings(const PIDTypes pidType,
													const double Ku,
													const double Tu) {
	constexpr float windup = 3;
	constexpr float smallErrorRange = 1;
	constexpr float smallErrorRangeTimeout = 100;
	constexpr float largeErrorRange = 3;
	constexpr float largeErrorRangeTimeout = 500;
	constexpr float maxAcceleration = 20;

	double Kp;
	double Ti;
	double Td;

	switch (pidType) {
		case PIDTypes::PID:
			Kp = 0.6 * Ku;
			Ti = 0.5 * Tu;
			Td = 0.125 * Tu;
			break;
		case PIDTypes::PI:
			Kp = 0.45 * Ku;
			Ti = (1.0 / 1.2) * Tu;
			Td = 0;
			break;
		case PIDTypes::PD:
			Kp = 0.8 * Ku;
			Ti = 0;
			Td = 0.125 * Tu;
			break;
		case PIDTypes::P:
			Kp = 0.5 * Ku;
			Ti = 0;
			Td = 0;
			break;
	}
	const double Ki = Ti == 0 ? 0 : Kp / Ti;
	const double Kd = Kp * Td;

	return lemlib::ControllerSettings(	//
		Kp, Ki, Kd, windup, smallErrorRange, smallErrorRangeTimeout, largeErrorRange,
		largeErrorRangeTimeout, maxAcceleration	 //
	);
}
};	// namespace Config
