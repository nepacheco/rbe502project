#include "PIDMotor.h"

PIDMotor::PIDMotor(double kp, double ki, double kd, double ff_plus, double ff_minus, Motor *motor)
{
    /* creates a PIDMotor object by receiving a motor and initializing constants*/
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
    this->ff_plus = ff_plus;
    this->ff_minus = ff_minus;
    this->motor = motor;

    this->motor->setDeadzones(ff_plus, ff_minus);
    timer.start();
}

PIDMotor::PIDMotor(PinName enablePin, PinName in1, PinName in2, PinName ss, encoderType type, SPI* spi)
{ /* creates a PIDMotor object by initializing the motor*/
    this->motor = new Motor(enablePin, in1, in2, ss, type,spi);
    timer.start();
}

void PIDMotor::setSetpoint(double setpoint)
{
    /* Assumes the setpoint passed in is in encoder ticks
    */
    this->setpoint = setpoint;
    this->sum_error = 0;
    this->last_error = 0;
    timer.reset();
   // this->calculate();
}

double PIDMotor::getSetpoint(){
    return this->setpoint;
}

void PIDMotor::calculate()
{
    /* Figure out the desired acceleration for the motor and then call drive motor*/
    uint64_t time_diff = timer.read_high_resolution_us();
    double time_step = time_diff/pow(10,6); //time is in nano seconds so this converts it to seconds
    timer.reset(); //update new current time
    
    double current_pos = this->readAngle();
    double curr_error = this->setpoint - current_pos;
    
    //creating integral term
    if (curr_error*sum_error >= 0 && abs(curr_error) < 1000)
    { //have not crossed the setpoint
        this->sum_error += curr_error * time_step; // multiply by the dt (make sure its in terms of seconds)
    } else
    { // have crossed the setpoint so we want to reset the summation
        this->sum_error = 0;
    }
    // pc.printf("time diff,: %ld\n",time_diff);
    //creating derivative term
    double change_error = (curr_error - this->last_error)/time_step;
    this->last_error = curr_error;

    //Currently only using integral and proportional control
    double desired_accel = this->ki * this->sum_error + this->kp * curr_error + this->kd * change_error;
    this->runMotor(desired_accel);
}

void PIDMotor::runMotor(double desired_accel)
{
    /* Drive the motor */
    double torque = desired_accel; //change if we find a dynamic model of system
    this->motor->drive(torque);
}

bool PIDMotor::setpointReached()
{
    return false;
}

void PIDMotor::setConstants(double kp, double ki, double kd)
{
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
}

double PIDMotor::readAngle(){
    return this->motor->readAngle();
}

void PIDMotor::stop(){
    this->motor->stop();
}

void PIDMotor::setHomePosition(){
    this->motor->enc->setHomePosition(this->readAngle());
}
