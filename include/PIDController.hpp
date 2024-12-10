//
// Created by chanadu on 12/9/24.
//
// Pragma once
#pragma once

class PIDController {
public:
	double kp, ki, kd, prevError, integral;
	PIDController(double newKp, double newKi, double newKd) {}
    double update(double setpoint, double processVariable);
};
