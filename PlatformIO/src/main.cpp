#include "config.h"
#include "mbed.h"
#include <math.h>
#include "Motors/PIDMotor.h"
#include "Motors/Joystick.h"
#include "Utility.h"
// #include "variant.h"   //PINOUT for Nucleo

// Create pointers to the 3 motors
Motor *backMotor;
Motor *frontMotor;
Motor *rotMotor;
int num_of_motors = 3;

// Encoder
Encoder *backEnc;
Encoder *frontEnc;
Encoder *rotEnc;

//PIDMotor
PIDMotor *pid_backMotor;
PIDMotor *pid_frontMotor;
PIDMotor *pid_rotMotor;
PIDMotor *pid_motors[3];

//Joystick
Joystick *joystick1;
Joystick *joystick2;

// Timer
Timer timer1;
Timer timer2;
Timer interrupt_test;
Ticker timer_up;
//SPI communication
SPI *spi = new SPI(SPI1_MOSI, SPI1_MISO, SPI1_SCK);
extern Serial pc(USBTX, USBRX);
//Enumeration of states
enum State
{
  updateJoystick,
  sendingPacket,
  receivingPacket,
  other
};
State state;
// Button ISR
void estop()
{
  stopAllMotors(pid_motors);
  pc.printf("----ESTOP----\n");
  pc.printf("Please restart the board\n");
  while (1)
  {
  }
}

//Timer interrupt for updating PID
int counter1 = 0;
void updatePID()
{
  // counter1++;
  // interrupt_test.reset();
  for (int i = 0; i < 3; i++)
  {
    pid_motors[i]->calculate();
  }
  // if (counter1 >= 1000){
  //   pc.printf("Time diff in interrupt: %f\n",interrupt_test.read());
  //   counter1 = 0;
  // }
}

event_callback_t readCallback;

bool runStateMachine = true;

int main()
{
  if (runStateMachine){
    readCallback.attach(messageCallback);
    pc.baud(57600);
  }
  else {
    pc.baud(9600);
  }

  // Need to read from serial on the interrupt

  // Start SPI
  spi->format(16, 1);
  spi->frequency(1000000);
  // SPI.setDataMode(SPI_MODE1);
  // SPI.setBitOrder(MSBFIRST);

  // Create encoder objects
  backEnc = new Encoder(SS_ENCBack, relative, spi);
  frontEnc = new Encoder(SS_ENCFront, relative, spi);
  rotEnc = new Encoder(SS_ENCRot, relative, spi);

  // attach each motor and set the pins for their h-bridges
  backMotor = new Motor(MOTOR1_ENABLE, MOTOR1_IN1, MOTOR1_IN2, backEnc);
  frontMotor = new Motor(MOTOR2_ENABLE, MOTOR2_IN1, MOTOR2_IN2, frontEnc);
  rotMotor = new Motor(MOTOR3_ENABLE, MOTOR3_IN1, MOTOR3_IN2, rotEnc);

  // create the PIDMotor objects with the created motors
  pid_backMotor = new PIDMotor(0.0008, 0.01, 0.0003, 1.75, 1.75, backMotor);
  pid_frontMotor = new PIDMotor(0.0008, 0.01, 0.0003, 1.75, 1.75, frontMotor);
  pid_rotMotor = new PIDMotor(0.0008, 0.01, 0.0003, 1.9, 1.8, rotMotor);

  // add to pid motor array
  pid_motors[0] = pid_backMotor;
  pid_motors[1] = pid_frontMotor;
  pid_motors[2] = pid_rotMotor;

  //Joystick
  joystick1 = new Joystick(JOYSTICK_1_X, JOYSTICK_1_Y, JOYSTICK_BUTT);
  joystick2 = new Joystick(JOYSTICK_2_X, JOYSTICK_2_Y, JOYSTICK_BUTT);

  // Buttons
  DigitalIn usrButton(USER_BUTT);          // blue button on the nucleo board
  InterruptIn estopButt(BIG_BUTT, PullUp); // estop
  estopButt.fall(&estop);

  // wait for user input to run
  pc.printf("Press any button to calibrate home position\n");
  while (!pc.readable() && !usrButton.read()){}
  wait_us(100000);

  home(pid_motors);

  pc.printf("Press any button to begin\n");
  while (!pc.readable() && !usrButton.read()){}
  //waiting for button press for start
  pc.printf("Begin\n");
  wait_us(100000);

  //initialize motor setpoints
  for (int i = 0; i < 3; i++)
  {
    pid_motors[i]->setSetpoint(0);
  }

  //Initial Conditions
  state = updateJoystick;
  double pos[3] = {0.0, 0.0, 61.0};
  double prevPos[3] = {0.0, 0.0, 61.0};   // inital position
  double returnPacket[3] = {0, 0, 0};
  double prevPacket[3] = {0, 0, 0};
  double diff = 0;
  double j1_direction[2];
  double j2_direction[2];
  bool packetReceived = false;
  timer1.start();
  timer2.start();

  //PID timer interrupt set to 1 ms
  timer_up.attach(&updatePID, 0.002);
  for (int i = 0; i < MESSAGE_LENGTH; i++)
  {
    buf[i] = '\0';
  }

  int index = 0;
  /* 
  original values 0.00015, 0.0001, 0.00001
  working values 0.0008, 0.01, 0.0003
  */

 // TODO: try decreasing the deadband for less stutter, which will allow higher kp to move faster
  if(!runStateMachine){
    pc.printf("Not running state machine \n");
    pid_motors[index]->setConstants(0.0008, 0.01, 0.0003);
  }
  
  // --------------------------------------- LOOOP ---------------------------------------

  while (1)
  { 	
    if (runStateMachine){
      switch (state){

      case updateJoystick:
        // pc.printf("updateJoystick\n");
        joystick1->updateValues();
        joystick2->updateValues();
        joystick1->getAnalogDirection(j1_direction);
        joystick2->getAnalogDirection(j2_direction);
        pos[0] = prevPos[0] + j1_direction[0] * INCREMENT;
        pos[1] = prevPos[1] + j1_direction[1] * -1 * INCREMENT;
        pos[2] = prevPos[2] + j2_direction[1] * INCREMENT;
        // wait_us(250000);
        state = sendingPacket;
        break;
      case sendingPacket:
        // pc.printf("sendingPacket\n");
        sendPacket(pos);
        timer1.reset();
        state = receivingPacket;
        timer2.reset();
        pc.read(buf, MESSAGE_LENGTH - 1, readCallback, SERIAL_EVENT_RX_ALL, 0);
        break;
      case receivingPacket:
        // pc.printf("receivingPacket\n");
        packetReceived = recievePacket(returnPacket);
        if (packetReceived)
        {
          if (!compareArray(prevPacket, returnPacket, 3))
          {
            pid_motors[0]->setSetpoint(returnPacket[1]);
            pid_motors[1]->setSetpoint(returnPacket[0]);
            pid_motors[2]->setSetpoint(returnPacket[2]);   
            copyArray(prevPacket, returnPacket, 3);
            copyArray(prevPos, pos, 3);
          }
          diff = timer2.read(); //timer in seconds
          packetReceived = false;
          pc.printf("Front Plate: %f Back Plate: %f Rotation: %f tDiff: %f\r\n", returnPacket[0], returnPacket[1], returnPacket[2], diff);
          state = updateJoystick;
        }
      }
   }else{
      pid_motors[index]->setSetpoint(10000);
      printPID(pid_motors, 0);
      // printPID(pid_motors, 1);
      // printPID(pid_motors, 2);
      pc.printf("\n");
    }
  }
}
