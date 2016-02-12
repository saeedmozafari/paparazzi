/*
############################## AUTMAV ##############################
# PROJECT: AUTMAV MODIFIED PAPARAZZI							   #
# DEVELOPED BY: ALI JAMEEI										   #
# Copyright (C) 2015-2016										   #	
####################################################################
*/


/* DESCRIPTION: 
    SBUS radio commander for guidance pack 
*/ 

#ifndef SBUS_COMMAND_H
#define SBUS_COMMAND_H

// header files
#include <std.h>
#include <string.h>
#include "mcu_periph/uart.h"
    
//definitions
#define SBUS_SIGNAL_OK          0x00
#define SBUS_SIGNAL_LOST        0x01
#define SBUS_SIGNAL_FAILSAFE    0x03

#define SBUS_COMMAND_Dev (&(SBUS_COMMAND_PORT).device)
#define SBUS_COMMAND_Transmit(c) SBUS_COMMAND_Dev->put_byte(SBUS_COMMAND_Dev->periph, c)
#define SBUS_COMMAND_ChAvailable() SBUS_COMMAND_Dev->char_available(SBUS_COMMAND_Dev->periph)
#define SBUS_COMMAND_Getch() SBUS_COMMAND_Dev->get_byte(SBUS_COMMAND_Dev->periph)
#define SBUS_COMMAND_SendMessage() SBUS_COMMAND_Dev->send_message(SBUS_COMMAND_Dev->periph)

// variables 
extern uint8_t sbusData[25];
extern int16_t channels[18];
extern int16_t servos[18];
extern uint8_t  failsafe_status;
extern int sbus_passthrough;
extern int toChannels;
extern uint8_t byte_in_sbus;
extern uint8_t bit_in_sbus;
extern uint8_t bit_in_channel;
extern uint8_t bit_in_servo;
extern uint8_t inBuffer[25];
extern int bufferIndex;
extern uint8_t inData;
extern int feedState;

//functions
extern void FUTABA_SBUS_begin(void);
extern int16_t FUTABA_SBUS_Channel(uint8_t ch);
extern uint8_t FUTABA_SBUS_DigiChannel(uint8_t ch);
extern void FUTABA_SBUS_Servo(uint8_t ch, int16_t position);
extern void FUTABA_SBUS_DigiServo(uint8_t ch, uint8_t position);
extern uint8_t FUTABA_SBUS_Failsafe(void);
extern void FUTABA_SBUS_PassthroughSet(int mode);
extern int FUTABA_SBUS_PassthroughRet(void);
extern void FUTABA_SBUS_UpdateServos(void);
extern void FUTABA_SBUS_UpdateChannels(void);
extern void FUTABA_SBUS_FeedLine(void);

#endif