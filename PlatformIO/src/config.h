#ifndef CONFIG_H
#define CONFIG_H

#include "mbed.h"


//Universal Serial object
// Serial communication
/* 
    Defines all pins being used in one easy location
 */
 
// Back Translation Motor
#define MOTOR1_ENABLE PC_8   // PA_3
#define MOTOR1_IN1 PF_14        // D5
#define MOTOR1_IN2 PE_11        // D4

// Front Translation Motor
#define MOTOR2_ENABLE PC_9   // PC_0
#define MOTOR2_IN1 PF_15        // D3
#define MOTOR2_IN2 PE_13        // D2

// Rotation Motor
#define MOTOR3_ENABLE A0    // PC_3
#define MOTOR3_IN1 PG_9        // D1
#define MOTOR3_IN2 PG_14       // D0

// Encoder Pins
#define SPI1_SCK PA_5         // PA_5
#define SPI1_MISO PA_6        // PA_6
#define SPI1_MOSI PA_7        // PA_7
#define SS_ENCFront PF_12       // PF_12
#define SS_ENCBack PD_15        // PD_15
#define SS_ENCRot PD_14        // PD_14

// Big Button
#define BIG_BUTT PE_9          // PE_9
#define USER_BUTT PC_13        // PC13

// Limit Switches
#define FRONT_SWITCH PG_3
#define BACK_SWTICH PG_2

// Joystick
#define JOYSTICK_1_X A1       
#define JOYSTICK_1_Y A2 
#define JOYSTICK_2_X A3       
#define JOYSTICK_2_Y A4       
#define JOYSTICK_BUTT PE_7   

#define INCREMENT .25

#endif // CONFIG_H

