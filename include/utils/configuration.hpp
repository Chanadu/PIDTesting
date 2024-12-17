#ifndef CONFIGURATION_HPP
#define CONFIGURATION_HPP

#include "main.h"

namespace Config {
enum class PIDTypes;

extern const float drivetrainTrackWidth;
extern const float drivetrainWheelbaseWidth;
extern const float drivetrainWheelRPM;
extern const float drivetrainHorizontalDrift;

extern double intakeVelocity;  // 0 - 1

extern const float drivetrainWheelType;

extern const float maxControllerValue;

extern double movementVelocityPercentage;  // 0 - 1
extern double turningVelocityPercentage;   // 0 - 1

extern float joystickThresholdPercentage;		 // 0 - 1
extern float outputMovementThresholdPercentage;	 // 0 - 1

extern float lateralCurveGain;
extern float angularCurveGain;

extern lemlib::ControllerSettings lateralMovementController;
extern lemlib::ControllerSettings angularMovementController;

extern float angularKu;
extern float angularTuSeconds;

extern float lateralKu;
extern float lateralTuSeconds;

extern PIDTypes currentlyUsingPIDType;

enum class DrivetrainMovement {
	Tank,
	SingleStickArcade,
	DoubleStickArcade,
	SingleStickCurvature,
	DoubleStickCurvature
};

enum class PIDTypes {
	P,
	PI,
	PD,
	PID,
};

inline const std::vector drivetrainMovements{
	DrivetrainMovement::Tank,
	DrivetrainMovement::SingleStickArcade,
	DrivetrainMovement::DoubleStickArcade,
	DrivetrainMovement::SingleStickCurvature,
	DrivetrainMovement::DoubleStickCurvature,
};

extern DrivetrainMovement drivetrainMovement;
extern std::string controllerStrings[3];
extern std::string controllerRumblePattern;

lemlib::ControllerSettings createControllerSettings(PIDTypes pidType, double Ku, double Tu);
}  // namespace Config

#endif	// CONFIGURATION_HPP
