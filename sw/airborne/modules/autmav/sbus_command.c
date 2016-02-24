/*
############################## AUTMAV ##############################
# PROJECT: AUTMAV MODIFIED PAPARAZZI				      		   #
# DEVELOPED BY: ALI JAMEEI							      		   #
# Copyright (C) 2015-2016								    	   #	
####################################################################
*/


/* DESCRIPTION: 
  SBUS radio commander for guidance pack
*/ 

//header files
#include "modules/autmav/sbus_command.h"
#include "subsystems/radio_control/sbus.h"

// variables 
uint8_t sbusData[25];
int16_t channels[18];
int16_t servos[18];
uint8_t  failsafe_status;
int sbus_passthrough;
int toChannels;
uint8_t byte_in_sbus;
uint8_t bit_in_sbus;
uint8_t bit_in_channel;
uint8_t bit_in_servo;
uint8_t inBuffer[25];
int bufferIndex;
uint8_t inData;
int feedState;

//functions
void FUTABA_SBUS_begin(){
	uint8_t loc_sbusData[25] = {
	  0x0f,0x01,0x04,0x20,0x00,0xff,0x07,0x40,0x00,0x02,0x10,0x80,0x2c,0x64,0x21,0x0b,0x59,0x08,0x40,0x00,0x02,0x10,0x80,0x00,0x00};
	int16_t loc_channels[18]  = {
	  		1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,0,0};
	int16_t loc_servos[18]    = {
  			1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,0,0};
  	memcpy(sbusData,loc_sbusData,25);
	memcpy(channels,loc_channels,18);
	memcpy(servos,loc_servos,18);
	failsafe_status = SBUS_SIGNAL_OK;
	sbus_passthrough = 1;
	toChannels = 0;
	bufferIndex=0;
	feedState = 0;
}

int16_t FUTABA_SBUS_Channel(uint8_t ch) {
  // Read channel data
  if ((ch>0)&&(ch<=16)){
    return channels[ch-1];
  }
  else{
    return 1023;
  }
}

uint8_t FUTABA_SBUS_DigiChannel(uint8_t ch) {
  // Read digital channel data
  if ((ch>0) && (ch<=2)) {
    return channels[15+ch];
  }
  else{
    return 0;
  }
}
void FUTABA_SBUS_Servo(uint8_t ch, int16_t position) {
  // Set servo position
  if ((ch>0)&&(ch<=16)) {
    if (position>2048) {
      position=2048;
    }
    servos[ch-1] = position;
  }
}
void FUTABA_SBUS_DigiServo(uint8_t ch, uint8_t position) {
  // Set digital servo position
  if ((ch>0) && (ch<=2)) {
    if (position>1) {
      position=1;
    }
    servos[15+ch] = position;
  }
}
uint8_t FUTABA_SBUS_Failsafe(void) {
  return failsafe_status;
}

void FUTABA_SBUS_PassthroughSet(int mode) {
  // Set passtrough mode, if true, received channel data is send to servos
  sbus_passthrough = mode;
}

int FUTABA_SBUS_PassthroughRet(void) {
  // Return current passthrough mode
  return sbus_passthrough;
}
void FUTABA_SBUS_UpdateServos(void) {
  // Send data to servos
  // Passtrough mode = 0 >> send own servo data
  // Passtrough mode = 1 >> send received channel data from the same UART channel
  // Passtrough mode = 2 >> send received channel data from the default autopilot channel
  uint8_t i,ch;

  //sending radio commands to external autopilot
  if (sbus_passthrough == 2) {
    for (i=1; i<17; i++) {
        FUTABA_SBUS_Servo(i,RC_PPM_TICKS_OF_USEC(sbus.ppm[i-1]) + 8);
    }
  }
  
  if ((sbus_passthrough == 0) || (sbus_passthrough == 2)) {
    // clear received channel data
    for (i=1; i<24; i++) {
      sbusData[i] = 0;
    }

    // reset counters
    ch = 0;
    bit_in_servo = 0;
    byte_in_sbus = 1;
    bit_in_sbus = 0;

    // store servo data
    for (i=0; i<176; i++) {
      if (servos[ch] & (1<<bit_in_servo)) {
        sbusData[byte_in_sbus] |= (1<<bit_in_sbus);
      }
      bit_in_sbus++;
      bit_in_servo++;

      if (bit_in_sbus == 8) {
        bit_in_sbus =0;
        byte_in_sbus++;
      }
      if (bit_in_servo == 11) {
        bit_in_servo =0;
        ch++;
      }
    }

    // DigiChannel 1
    if (channels[16] == 1) {
      sbusData[23] |= (1<<0);
    }
    // DigiChannel 2
    if (channels[17] == 1) {
      sbusData[23] |= (1<<1);
    }

    // Failsafe
    if (failsafe_status == SBUS_SIGNAL_LOST) {
      sbusData[23] |= (1<<2);
    }

    if (failsafe_status == SBUS_SIGNAL_FAILSAFE) {
      sbusData[23] |= (1<<2);
      sbusData[23] |= (1<<3);
    }
  }
  // send data out
  for (i=0;i<25;i++) {
   SBUS_COMMAND_Transmit(sbusData[i]);
  }
}
void FUTABA_SBUS_UpdateChannels(void) {
  
  channels[0]  = ((sbusData[1]|sbusData[2]<< 8) & 0x07FF);
  channels[1]  = ((sbusData[2]>>3|sbusData[3]<<5) & 0x07FF);
  channels[2]  = ((sbusData[3]>>6|sbusData[4]<<2|sbusData[5]<<10) & 0x07FF);
  channels[3]  = ((sbusData[5]>>1|sbusData[6]<<7) & 0x07FF);
  channels[4]  = ((sbusData[6]>>4|sbusData[7]<<4) & 0x07FF);
  channels[5]  = ((sbusData[7]>>7|sbusData[8]<<1|sbusData[9]<<9) & 0x07FF);
  channels[6]  = ((sbusData[9]>>2|sbusData[10]<<6) & 0x07FF);
  channels[7]  = ((sbusData[10]>>5|sbusData[11]<<3) & 0x07FF); // & the other 8 + 2 channels if you need them
  channels[8]  = ((sbusData[12]|sbusData[13]<< 8) & 0x07FF);
  channels[9]  = ((sbusData[13]>>3|sbusData[14]<<5) & 0x07FF);
  channels[10] = ((sbusData[14]>>6|sbusData[15]<<2|sbusData[16]<<10) & 0x07FF);
  channels[11] = ((sbusData[16]>>1|sbusData[17]<<7) & 0x07FF);
  channels[12] = ((sbusData[17]>>4|sbusData[18]<<4) & 0x07FF);
  channels[13] = ((sbusData[18]>>7|sbusData[19]<<1|sbusData[20]<<9) & 0x07FF);
  channels[14] = ((sbusData[20]>>2|sbusData[21]<<6) & 0x07FF);
  channels[15] = ((sbusData[21]>>5|sbusData[22]<<3) & 0x07FF);
 
  
  // Failsafe
  failsafe_status = SBUS_SIGNAL_OK;
  if (sbusData[23] & (1<<2)) {
    failsafe_status = SBUS_SIGNAL_LOST;
  }
  if (sbusData[23] & (1<<3)) {
    failsafe_status = SBUS_SIGNAL_FAILSAFE;
  }

}
void FUTABA_SBUS_FeedLine(void){
  if (SBUS_COMMAND_ChAvailable() > 24){
    while(SBUS_COMMAND_ChAvailable() > 0){
      inData = SBUS_COMMAND_Getch();
      switch (feedState){
      case 0:
        if (inData != 0x0f){
          while(SBUS_COMMAND_ChAvailable() > 0){//read the contents of in buffer this should resync the transmission
            inData = SBUS_COMMAND_Getch();
          }
          return;
        }
        else{
          bufferIndex = 0;
          inBuffer[bufferIndex] = inData;
          inBuffer[24] = 0xff;
          feedState = 1;
        }
        break;
      case 1:
        bufferIndex ++;
        inBuffer[bufferIndex] = inData;
        if (bufferIndex < 24 && SBUS_COMMAND_ChAvailable() == 0){
          feedState = 0;
        }
        if (bufferIndex == 24){
          feedState = 0;
          if (inBuffer[0]==0x0f && inBuffer[24] == 0x00){
            memcpy(sbusData,inBuffer,25);
            toChannels = 1;
          }
        }
        break;
        default:
        break;
      }
    }
  }
}
