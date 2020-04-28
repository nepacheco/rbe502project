/* 
    Encoder.h
 */

#ifndef ENCODER_H
#define ENDODER_H

#include "config.h"
#include "SPI.h"
//Enums
enum encoderType
{
    absolute,
    relative
};
extern Serial pc;

class Encoder
{
    /* 
    Connected via SPI
    Mode 1
    MSB
 */
public:
    // Set the pins of an encoder
    Encoder(PinName ss, encoderType type, SPI* spi);

    DigitalOut * getPin();

    // read from SPI to update the angle
    double readAngle();

    void setHomePosition(int home_position);
private:
    // pins
    DigitalOut * cs;
    double currentAngle = 0;
    double previousAngle = 0;
    double home_position = 0;
    encoderType type;
    int const bits14 = pow(2, 14);
    SPI * spi;
};

#endif // ENCODER_H