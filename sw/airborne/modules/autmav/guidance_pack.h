/*
############################## AUTMAV ##############################
# PROJECT: AUTMAV MODIFIED PAPARAZZI							   #
# DEVELOPED BY: ALI JAMEEI										   #
# Copyright (C) 2015-2016										   #	
####################################################################
*/


/* DESCRIPTION: 
    Guidance Pack to be used along with another autopilot 
*/ 
#ifndef GUIDANCE_PACK_H
#define GUIDANCE_PACK_H

#define FUTABA_SBUS_CH1_MID (FUTABA_SBUS_CH1_MIN + FUTABA_SBUS_CH1_MAX)/2
#define FUTABA_SBUS_CH2_MID (FUTABA_SBUS_CH2_MIN + FUTABA_SBUS_CH2_MAX)/2
#define FUTABA_SBUS_CH3_MID (FUTABA_SBUS_CH3_MIN + FUTABA_SBUS_CH3_MAX)/2
#define FUTABA_SBUS_CH4_MID (FUTABA_SBUS_CH4_MIN + FUTABA_SBUS_CH4_MAX)/2

#define FUTABA_SBUS_CH1_COARSE (FUTABA_SBUS_CH1_MAX - FUTABA_SBUS_CH1_MIN)/2
#define FUTABA_SBUS_CH2_COARSE (FUTABA_SBUS_CH2_MAX - FUTABA_SBUS_CH2_MIN)/2
#define FUTABA_SBUS_CH3_COARSE (FUTABA_SBUS_CH3_MAX - FUTABA_SBUS_CH3_MIN)/2
#define FUTABA_SBUS_CH4_COARSE (FUTABA_SBUS_CH4_MAX - FUTABA_SBUS_CH4_MIN)/2

// variables 

//functions
extern void guidance_pack_init( void );
extern void guidance_pack_event( void );
extern void guidance_pack_periodic( void );
extern void guidance_pack_downlink( void );
extern void guidance_pack_rc_handler( void );
extern void guidance_pack_v_loop( void );
extern void guidance_pack_h_loop( void );
extern void guidance_pack_calc_rc_out( void );
#endif