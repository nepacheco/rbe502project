#include "Joystick.h"

Joystick::Joystick(PinName xPin, PinName yPin, PinName bPin)
{
  this->xPin = new AnalogIn(xPin);
  this->yPin = new AnalogIn(yPin);
  this->bPin = new DigitalIn(bPin);
  this->currentXVal = 0;
  this->currentYVal = 0;
  this->buttonPressed = 0;

  //pinMode(xPin, INPUT_PULLDOWN);
  //pinMode(yPin, INPUT_PULLDOWN);
  //pinMode(bPin, INPUT);
}

double Joystick::getX()
{
  return this->currentXVal;
}
double Joystick::getY()
{
  return this->currentYVal;
}
int Joystick::isPressed()
{
  return this->buttonPressed;
}
 
 /*
 DIRECTION KEY:

  8  1  2
   \ | /
 7---|--- 3
   / | \ 
  6  5  4

  */
int Joystick::getDirection()
{
  int x = this->getX();
  int y = this->getY();
  if(x < OUTLIER && y < OUTLIER){
    if(x < MIN_TILT && (y > MIN_TILT && y < MAX_TILT)) return WEST;
    if(x > MAX_TILT && (y > MIN_TILT && y < MAX_TILT)) return EAST;
    if(y < MIN_TILT && (x > MIN_TILT && x < MAX_TILT)) return NORTH;
    if(y > MAX_TILT && (x > MIN_TILT && x < MAX_TILT)) return SOUTH;
    if(x < MIN_TILT && y < MIN_TILT) return NORTHWEST;
    if(x < MIN_TILT && y > MAX_TILT) return SOUTHWEST;
    if(x > MAX_TILT && y < MIN_TILT) return NORTHEAST;
    if(x > MAX_TILT && y > MAX_TILT) return SOUTHEAST;
  }
  return 0;
}

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
 return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void Joystick::getAnalogDirection(double * xy_vals)
{
  double x = this->getX();
  double y = this->getY();
  xy_vals[0] = 0;
  xy_vals[1] = 0;
  if(x > MAX_TILT || x < MIN_TILT){
    if(x > MAX_TILT){
      xy_vals[0] = mapfloat(x, MAX_TILT, MAX_READ, 0.0, 1.0);
    }
    else if(x < MIN_TILT){
      xy_vals[0] = mapfloat(x, 0, MIN_TILT, -1.0, 0.0);

    }
  }
  if(y > MAX_TILT || y < MIN_TILT){
    if(y > MAX_TILT){
      xy_vals[1] =  mapfloat(y, MAX_TILT, MAX_READ, 0.0, 1.0);
    }
    else if(y < MIN_TILT){
      xy_vals[1] = mapfloat(y, 0, MIN_TILT, -1.0, 0.0);
    }
  }
}

void Joystick::updateValues(){
    this->currentXVal = xPin->read();
    this->currentYVal = yPin->read();
    this->buttonPressed = bPin->read();
}

