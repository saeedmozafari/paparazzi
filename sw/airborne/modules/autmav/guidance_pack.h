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

// variables 


//functions
extern void guidance_pack_init( void );
extern void guidance_pack_event( void );
extern void guidance_pack_periodic( void );
#endif