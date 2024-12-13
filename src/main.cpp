#include "main.h"

void disabled() {}
void competition_initialize() {}


const float drivetrainWheelSize = 2.75;
const float distanceToTravelInches = 48;
const float errorRange = 0.5;
const float movementSpeed = 0.5;

pros::Controller controller(pros::E_CONTROLLER_MASTER);
pros::MotorGroup leftMotorGroup({1, -12, 3}, pros::MotorGears::blue);
pros::MotorGroup rightMotorGroup({-4, 5, -8}, pros::MotorGears::blue);
pros::Imu imu(11);

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

class PIDImpl;
class PID {
    public:
        // Kp -  proportional gain
        // Ki -  Integral gain
        // Kd -  derivative gain
        // dt -  loop interval time
        // max - maximum value of manipulated variable
        // min - minimum value of manipulated variable
        PID( double dt, double max, double min, double Kp, double Kd, double Ki );

        // Returns the manipulated variable given a setpoint and current process value
        double calculate( double setpoint, double pv );
        ~PID();

    private:
        PIDImpl *pimpl;
};

class PIDImpl
{
    public:
        PIDImpl( double dt, double max, double min, double Kp, double Kd, double Ki );
        ~PIDImpl();
        double calculate( double setpoint, double pv );

    private:
        double _dt;
        double _max;
        double _min;
        double _Kp;
        double _Kd;
        double _Ki;
        double _pre_error;
        double _integral;
};


PID::PID(double dt, double max, double min, double Kp, double Kd, double Ki )
{
    pimpl = new PIDImpl(dt,max,min,Kp,Kd,Ki);
}
double PID::calculate( double setpoint, double pv )
{
    return pimpl->calculate(setpoint,pv);
}
PID::~PID() 
{
    delete pimpl;
}


/**
 * Implementation
 */
PIDImpl::PIDImpl( double dt, double max, double min, double Kp, double Kd, double Ki ) :
    _dt(dt),
    _max(max),
    _min(min),
    _Kp(Kp),
    _Kd(Kd),
    _Ki(Ki),
    _pre_error(0),
    _integral(0)
{
}

double PIDImpl::calculate( double setpoint, double pv )
{
    
    // Calculate error
    double error = setpoint - pv;

    // Proportional term
    double Pout = _Kp * error;

    // Integral term
    _integral += error * _dt;
    double Iout = _Ki * _integral;

    // Derivative term
    double derivative = (error - _pre_error) / _dt;
    double Dout = _Kd * derivative;

    // Calculate total output
    double output = Pout + Iout + Dout;

    // Restrict to max/min
    if( output > _max )
        output = _max;
    else if( output < _min )
        output = _min;

    // Save error to previous error
    _pre_error = error;

    return output;
}

PIDImpl::~PIDImpl()
{
}

void pre_auton(void) {
// Initializing Robot Configuration. DO NOT REMOVE!
vexcodeInit();

	leftMotorGroup.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
	rightMotorGroup.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
	leftMotorGroup.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	rightMotorGroup.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

void drivePID() {
    PID lPID = PID(0.1, 100, -100, 0.1, 0.01, 0.5);
    PID tPID = PID(0.1, 100, -100, 0.1, 0.01, 0.5);
	int lTarget = 2;
	int tTarget = 0;

	double lateralMotorPower = lPID.calculate(lTarget, getAveragePosition());
	double turnMotorPower = tPID.calculate(tTarget, imu.get_heading());
	leftMotorGroup.move(lateralMotorPower + turnMotorPower);
    rightMotorGroup.move(lateralMotorPower - turnMotorPower);


	pros::delay(10);
}

void autonomous() {
	pros::Task a(drivePID);
}

void opcontrol() {

	while (true) {
		
		pros::delay(20);
	}
}
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");
}
