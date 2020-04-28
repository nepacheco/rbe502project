#include "Motor.h"

Motor::Motor(PinName enablePin, PinName _in1, PinName _in2, Encoder* enc){
    this->enablePin = new PwmOut(enablePin);
    this->enablePin->period_ms(1.5);
    this->in1 = new DigitalOut(_in1);
    this->in2 = new DigitalOut(_in2);
    this->enc = enc;
}

Motor::Motor(PinName enablePin, PinName in1, PinName in2, PinName ss, encoderType type, SPI* spi){
    this->enablePin = new PwmOut(enablePin);
    this->enablePin->period_ms(1.5);
    this->in1 = new DigitalOut(in1);
    this->in2 = new DigitalOut(in2);
    this->enc = new Encoder(ss,type, spi);
}

void Motor::drive(double speed){
  // takes a value between 1 and -1

  // check if motor should be stopped
  if (speed == 0){
    this->enablePin->write(0);
    return;
  }

  // Check direction
  bool isForward = true;
  if (speed < 0){
    isForward = false;
  }

  // cap speed at max speed
  speed = abs(speed);
  if (speed >= 1){
    speed = 1;
  }
 

  // Set direction of motor
  int writeSpeed;
  if (isForward){
    this->in1->write(1);
    this->in2->write(0);
    writeSpeed = speed*(maxSpeed-this->for_deadzone) + this->for_deadzone;
  }
  else {
    this->in1->write(0);
    this->in2->write(1);
    writeSpeed = speed*(maxSpeed-this->rev_deadzone) + this->rev_deadzone;
  }
  
  // Set speed of motor
  this->enablePin->write(writeSpeed);;
}

void Motor::stop(){
  this->drive(0);
}

double Motor::readAngle(){
  return this->enc->readAngle();
}

DigitalOut * Motor::getEncoderPin(){
  return this->enc->getPin();
}

void Motor::setDeadzones(double for_deadzone, double rev_deadzone){
  //takes in a voltage that the motors start moving at 
  this->for_deadzone = (for_deadzone / 12) * maxSpeed;
  this->rev_deadzone = (rev_deadzone / 12) * maxSpeed;
}