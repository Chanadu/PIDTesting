//
// Created by chanadu on 12/9/24.
//

class PIDController {

public:
	double kp, ki, kd, prevError, integral;
	PIDController(double newKp, double newKi, double newKd) : kp(newKp), ki(newKi), kd(newKd), prevError(0), integral(0) {}

    double update(double setpoint, double processVariable) {
    	// Calculate error
    double error = setpoint - processVariable;

        // Calculate proportional term
    double proportional = kp * error;

        // Calculate integral term
        integral += ki * error;

        // Calculate derivative term
        double derivative = kd * (error - prevError);

        // Calculate PID output
        double output = proportional + integral + derivative;

        // Update previous error
        prevError = error;

        return output;
    }
};
