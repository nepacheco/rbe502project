/* 
    Joystick.h
 */

#ifndef JOYSTICK_H
#define JOYSTICK_H

#include "config.h"

#define MAX_TILT 0.6
#define MIN_TILT 0.4
#define OUTLIER 2000
#define MAX_READ 1

#define NORTH 1
#define NORTHEAST 2
#define EAST 3
#define SOUTHEAST 4
#define SOUTH 5
#define SOUTHWEST 6
#define WEST 7
#define NORTHWEST 8

class Joystick
{
private:
    // pins
    AnalogIn* xPin;
    AnalogIn* yPin; 
    DigitalIn* bPin;
    double currentXVal = 0.0;
    double currentYVal = 0.0;
    int buttonPressed = 0;

public:
    // Set the pins of an Joystick
    Joystick(PinName xPin, PinName yPin, PinName bPin);
    int getPin();
    double getX();
    double getY();
    int isPressed();
    int getDirection();
    void updateValues();
    void getAnalogDirection(double * direction);
};

#endif // Joystick_H