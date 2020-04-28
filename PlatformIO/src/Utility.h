/* 
    Utility.h
    Holds basic scripts to make motors move and for testing
 */

#ifndef UTILITY_H
#define UTILITY_H
#define MESSAGE_LENGTH 60
#define MAX_ENCODER_VALUE 16383
#define _USE_MATH_DEFINES
//unfortunate brute force
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include "config.h"
#include "mbed.h"
#include "Motors/PIDMotor.h"
#include <string>
#include <cmath>
extern Serial pc;

// Utility Functions
void stopAllMotors(Motor *motors[3])
{
  for (int i = 0; i < 3; i++)
  {
    motors[i]->stop();
  }
}

void stopAllMotors(PIDMotor *motors[3])
{
  for (int i = 0; i < 3; i++)
  {
    motors[i]->stop();
  }
}

void home(PIDMotor *pid_motors[3])
{
  /* 
    Resets the home position for each of the translational motors

    Disables the interrupt and drives the motor back
     until limit switch is reached.
    Optionally will move forward a little the slowly back up 
     to hit the switch again

    Blocking for now
   */

  stopAllMotors(pid_motors);
  DigitalIn *switches[2] = {new DigitalIn(BACK_SWTICH, PullUp), new DigitalIn(FRONT_SWITCH, PullUp)};

  for (int i = 0; i < 2; i++)
  {
    pc.printf("Homing for Motor: %d\n", i);
    // drive until the switch is hit
    while (switches[i]->read())
    {
      // pid_motors[i]->readAngle();
      pid_motors[i]->runMotor(-1);
    }

    pid_motors[i]->stop();

    wait_us(500000);

    pc.printf("Home Position Reached for Motor: %d\n", i);
    pid_motors[i]->setHomePosition(); // motors set home position
  }

  pid_motors[2]->setHomePosition();
}

void readAllAngles(PIDMotor *pid_motors[3], bool print = false)
{
  /* 
    For each encoder, goes through and prints out current angle
   */
  for (int i = 0; i < 3; i++)
  {
    pc.printf(" Encoder %d: ", i);
    pc.printf("%f", pid_motors[i]->readAngle());
  }
  pc.printf("\n");
}

/*
  Asyncrounous serial call to Matlab, send array of 3 doubles (X Y Z)

  USAGE:
  sendPacket(pos);
  recievePacket(act);
 */
void sendPacket(double *pos)
{
  char packet[MESSAGE_LENGTH];
  // string message = std::to_string(pos[0]) + ";" + std::to_string(pos[1]) + ";" + std::to_string(pos[2]);
  // std::strcpy(packet,message.c_str());
  pc.printf("%f;%f;%f\n", pos[0], pos[1], pos[2]);
}

// Converts a roation in rad to revolutions, and then to encoder ticks
double radToTicks(double rad)
{

  return (rad / (2 * M_PI) * MAX_ENCODER_VALUE);
}

// Converts a distance in mm to revolutions, and then to encoder ticks
double mmToTicks(double mm)
{
  // Lead screw is .2in per rotation
  return (mm / (2.54) * MAX_ENCODER_VALUE);
}

// Converts l, theta, d to encoder values for [FRONTPLATE, BACKPLATE, ROATATION]
void actuatorVariablesToEncoderValues(double *actuator)
{
  long frontMotor = 0;
  long backMotor = 0;
  long rotMotor = 0;

  if (mmToTicks(actuator[0]) < 0)
  {
    frontMotor = mmToTicks(actuator[2]);
    backMotor = mmToTicks(actuator[2]) - mmToTicks(actuator[0]);
  }
  else
  {
    frontMotor = mmToTicks(actuator[2]) + mmToTicks(actuator[0]);
    backMotor = mmToTicks(actuator[2]);
  }
  rotMotor = radToTicks(actuator[1]);

  actuator[0] = frontMotor;
  actuator[1] = backMotor;
  actuator[2] = rotMotor;
}

/*
  Asyncrounous serial call to Matlab, retrieves array of 3 doubles (L Theta D)

  USAGE:
  sendPacket(pos);
  recievePacket(act);
 */
unsigned char rcv[MESSAGE_LENGTH + 1];
uint8_t buf[MESSAGE_LENGTH];
char *tokens[3];
char *temp = NULL;
int val = 0;
int counter = 0;
enum PacketType
{
  noPacket,
  newPacket,
  readError
};
PacketType packetAvailable = noPacket;
void messageCallback(int events)
{
  if (events & SERIAL_EVENT_RX_CHARACTER_MATCH)
  {
    packetAvailable = newPacket;
    // pc.printf("char match\n");
  }
  else
  {
    pc.printf("error reading packet\n");
    packetAvailable = readError;
  }
}

bool recievePacket(double *act)
{
  if (packetAvailable == newPacket)
  {
    int index = 0;
    temp = strtok((char *)buf, ","); // takes a list of delimiters
    while (temp != NULL)
    {
      tokens[index] = temp;
      index++;
      temp = strtok(NULL, ","); // takes a list of delimiters
    }
    act[0] = atof(tokens[0]);
    act[1] = atof(tokens[1]);
    act[2] = atof(tokens[2]);
    actuatorVariablesToEncoderValues(act);
    packetAvailable = noPacket;
    for (int i = 0; i < MESSAGE_LENGTH; i++)
    {
      buf[i] = '\0';
    }
    return true;
  }
  else if (packetAvailable == readError)
  {// this will say we received a packet, but will not
  // update the actuator variables because of an error
    packetAvailable = noPacket;
    return true;
  }
  else
  {
    return false;
  }
}

bool compareArray(double array1[], double array2[], int size){
  for (int i = 0; i < size; i++){
    if (array1[i] != array2[i]){
      return false;
    }
  }
  return true;
}

void copyArray(double array1[], double array2[], int size){
  //copies the contents of array2 into array1
  for (int i = 0; i < size; i++){
    array1[i] = array2[i];
  }
}


double lasterror = 0;
double tolerance = 2;
bool setSS = true;
int countt = 0;

void printPID(PIDMotor* motors[3], int motorIndex){
  PIDMotor* m = motors[motorIndex];
  countt = countt + 1;

  double error = m->getSetpoint() - m->readAngle();

  if ((abs(error - lasterror)< tolerance) && setSS){
    // steadystate = t.read();
    setSS = false;
    pc.printf("count %d \t", countt);
  }

  lasterror = error;
  pc.printf("Errors %d: %.3f \t ", motorIndex, error);

}

#endif // UTILITY_H