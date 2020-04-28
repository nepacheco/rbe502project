/* 
    Motor.h
 */

#ifndef MOTOR_H
#define MOTOR_H

#include "mbed.h"
#include "config.h"
#include "Encoder.h"

class Motor
{

private:
    // pins
    PwmOut * enablePin;
    DigitalOut * in1;
    DigitalOut * in2;

    // const
    int const maxSpeed = 1;

    //deadzones
    double for_deadzone = 0;
    double rev_deadzone = 0;

public:
    Encoder *enc;

    // sets the pinouts for the h-bridge
    Motor(PinName enablePin, PinName in1, PinName in2, Encoder *enc);
    // this constructor initializes the encoder within itself
    Motor(PinName enablePin, PinName in1, PinName in2, PinName ss, encoderType type, SPI* spi);

    // drive set speed with negative being reverse
    void drive(double speed);

    // Stops the motor
    void stop();

    double readAngle();

    // TODO: use pid to drive set distance (might go to separate class)
    void driveDistance(int distance);

    DigitalOut * getEncoderPin();

    void setDeadzones(double for_deadzone, double rev_deadzone);
};

#endif // MOTOR_H