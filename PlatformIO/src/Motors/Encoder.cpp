#include "Encoder.h"

Encoder::Encoder(PinName ss, encoderType type, SPI* spi)
{
  this->cs = new DigitalOut(ss);
  this->spi = spi;
  this->cs->write(1);
  this->type = type;
  
}

double Encoder::readAngle()
{
  /* Reading from spi */
  // when attached to interrupt to be made volatile?
  unsigned short byte1;
  unsigned short result;

  // get transferred bytes
  // assuming 16 bit data transfer
  this->cs->write(0); // select slave
  this->spi->write(0x3FFF); //send command to receive angle
  this->cs->write(1); // deselect slave
  
  wait_us(1);

  this->cs->write(0); // select slave
  byte1 = this->spi->write(0x0000); //send dummy command but actually receive angle
  this->cs->write(1); // deselect slave

  byte1 &= 0x3FFF; // unmask first byte

  result = byte1;
  /* Interpreting SPI data*/
  if (this->type == relative){ // relative encoder requires a constant count of the current 'distance'
    int upper_ticks = this->bits14*0.9;
    int lower_ticks = this->bits14*0.1;
    int change = 0;
    
    if (this->previousAngle < lower_ticks && result > upper_ticks)
    { // the change will be negative in this case
      change = -this->bits14 + (result - this->previousAngle); 

    } else if (this->previousAngle > upper_ticks && result < lower_ticks)
    { // change will be positive but its wrapping around
      change = this->bits14 + (result - this->previousAngle);

    } else
    { //normal change can be positive or negative without wrapping
      change = result - this->previousAngle;
    }
    // Serial.print("Change: ");
    // Serial.println(change);
    this->currentAngle += change;
    // Serial.print("Current Angle: ");
    // Serial.println(currentAngle);
    this->previousAngle = result; // update the previous angle for next loop
    return (this->currentAngle - this->home_position);
  } else
  { // absolute encoder only cares about current angle
    this->currentAngle = result;
    // Serial.print(" Current Angle: ");
    // Serial.println(result);
    return fmod((this->currentAngle - this->home_position),this->bits14);
  }
  return (this->currentAngle - this->home_position);
}

void Encoder::setHomePosition(int home_position)
{
  this->home_position = home_position;
}

DigitalOut* Encoder::getPin()
{
  return this->cs;
}
