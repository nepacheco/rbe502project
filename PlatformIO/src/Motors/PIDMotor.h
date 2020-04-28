/* 
    PIDMotor.h
 */

#ifndef PIDMOTOR_H
#define PIDMOTOR_H

#include "Motor.h"
extern Serial pc;
class PIDMotor{
private:
    //Timers
    Timer timer;
    // PID gains
    double kp = 1;
    double ki = 0;
    double kd = 0;
    double ff_plus = 0;
    double ff_minus = 0;

    // PID other shit
    double setpoint = 0;
    double last_error = 0;
    double sum_error = 0;
    Motor* motor;


public:
    // set the gains for the PID and the feedforward
    PIDMotor(double kp, double ki, double kd, double ff_plus, double ff_minus, Motor* motor);
    // initializing a PIDMotor with just motor and encoder pins
    PIDMotor(PinName enablePin, PinName in1, PinName in2, PinName ss, encoderType type, SPI* spi);

    // set the setpoint and drive motors
    void calculate();

    // send the desired acceleration to motor
    void runMotor(double desired_accel);

    //change the setpoint for PID
    void setSetpoint(double setpoint);

    double getSetpoint();

    // check if pid is done interpolating
    bool setpointReached();

    void setConstants(double kp, double ki, double kd);

    double readAngle();

    void setHomePosition();

    void stop();
};

#endif // PIDMOTOR_H