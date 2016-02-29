/*
############################## AUTMAV ##############################
# PROJECT: AUTMAV MODIFIED PAPARAZZI				               #
# DEVELOPED BY: ALI JAMEEI							               #
# Copyright (C) 2015-2016								           #	
####################################################################
*/


/* DESCRIPTION: 
  Guidance Pack using paparazzi guidance commands to be used along with another autopilot  
*/ 


#include "modules/autmav/guidance_pack.h"
#include "modules/autmav/sbus_command.h"

#ifndef SITL
#include "subsystems/radio_control/sbus.h"
#endif

#include "subsystems/radio_control.h"
#include "subsystems/datalink/downlink.h"
#include "firmwares/rotorcraft/stabilization.h"
#include "firmwares/rotorcraft/navigation.h"
#include "paparazzi.h"

// variables
int gp_horizontal_ok; 
int gp_vertical_ok;

float gp_roll_desired;
float gp_pitch_desired;
float gp_yaw_desired;
float gp_throttle_desired;


// initialization function
void guidance_pack_init( void )
{	
	gp_horizontal_ok = 0;
	gp_vertical_ok = 0;
#ifndef SITL
	FUTABA_SBUS_begin();
	FUTABA_SBUS_PassthroughSet(2);
#endif
}

// setting command mode and sending rc commands to external autopilot
void guidance_pack_rc_handler( void )
{	
#ifndef SITL
	if ((sbus.ppm[SBUS_PASSTHROUGH_CH-1] < 1500) && (radio_control.status == RC_OK)) {
		FUTABA_SBUS_PassthroughSet(2); // rc commands set to full manual
		guidance_pack_calc_rc_out();
	} 
	
	else if ((sbus.ppm[SBUS_PASSTHROUGH_CH-1] >= 1500) && (radio_control.status == RC_OK)) {
		FUTABA_SBUS_PassthroughSet(0); // rc commands set to guidance pack
		FUTABA_SBUS_Servo(ZEROUAV_MANUAL_CH,RC_PPM_TICKS_OF_USEC(ZEROUAV_AUTO_USEC)); // put zero on auto mode
		FUTABA_SBUS_Servo(ZEROUAV_AUTO_MODE_CH,RC_PPM_TICKS_OF_USEC(ZEROUAV_AUTOHOVER_MODE_USEC)); // put zero on auto-hover mode
		guidance_pack_calc_rc_out();
		FUTABA_SBUS_Servo(3,RC_PPM_TICKS_OF_USEC(sbus.ppm[2])); // vertical mode still unavailable so it should be set manually
		FUTABA_SBUS_Servo(4,RC_PPM_TICKS_OF_USEC(sbus.ppm[3])); // yaw axis is controlled by user
	} 
	
	else if (radio_control.status == RC_REALLY_LOST) {
		FUTABA_SBUS_PassthroughSet(0); // rc commands set zero uav failsafe back landing
		FUTABA_SBUS_Servo(ZEROUAV_MANUAL_CH,RC_PPM_TICKS_OF_USEC(ZEROUAV_AUTO_USEC)); // put zero on auto mode
		FUTABA_SBUS_Servo(ZEROUAV_AUTO_MODE_CH,RC_PPM_TICKS_OF_USEC(ZEROUAV_BACKLANDING_MODE_USEC)); // put zero on auto-hover mode
		FUTABA_SBUS_Servo(1,RC_PPM_TICKS_OF_USEC(1500)); // set all commands to position hold for zero uav  
		FUTABA_SBUS_Servo(2,RC_PPM_TICKS_OF_USEC(1500));
		FUTABA_SBUS_Servo(3,RC_PPM_TICKS_OF_USEC(1500));
		FUTABA_SBUS_Servo(4,RC_PPM_TICKS_OF_USEC(1500));
	}
#endif
}

// vertical loop
void guidance_pack_v_loop( void ) {

	gp_throttle_desired = stabilization_cmd[COMMAND_THRUST] / 9600.0;
	BoundAbs(gp_throttle_desired, 1.0);
}

// horizontal loop
void guidance_pack_h_loop( void ) {
	
	gp_roll_desired = stabilization_cmd[COMMAND_ROLL] / 9600.0;
	gp_pitch_desired = stabilization_cmd[COMMAND_PITCH] / 9600.0;
	gp_yaw_desired = stabilization_cmd[COMMAND_YAW] / 9600.0;
	Bound(gp_roll_desired, -1.0, 1.0);
	Bound(gp_pitch_desired, -1.0, 1.0);
	Bound(gp_yaw_desired, -1.0, 1.0);
}

// calculating rc values from raw commands
void guidance_pack_calc_rc_out ( void ) {
#ifndef SITL
	FUTABA_SBUS_Servo(ZEROUAV_ROLL_CH,RC_PPM_TICKS_OF_USEC(FUTABA_SBUS_CH1_MID + FUTABA_SBUS_CH1_COARSE * gp_roll_desired * SBUS_COMMAND_SCALER));
	FUTABA_SBUS_Servo(ZEROUAV_PITCH_CH,RC_PPM_TICKS_OF_USEC(FUTABA_SBUS_CH2_MID + FUTABA_SBUS_CH2_COARSE * gp_pitch_desired * SBUS_COMMAND_SCALER));
	FUTABA_SBUS_Servo(ZEROUAV_THROTTLE_CH,RC_PPM_TICKS_OF_USEC(FUTABA_SBUS_CH3_MAX - FUTABA_SBUS_CH2_COARSE * gp_throttle_desired * 2)); //throttle is controlled by user
	FUTABA_SBUS_Servo(ZEROUAV_YAW_CH,RC_PPM_TICKS_OF_USEC(FUTABA_SBUS_CH4_MID + FUTABA_SBUS_CH4_COARSE * gp_yaw_desired * SBUS_COMMAND_SCALER)); //yaw is controlled by user
#endif
}

//event function
void guidance_pack_event( void )
{	
	//FUTABA_SBUS_FeedLine();
}

//periodic function
void guidance_pack_periodic( void )
{	
	guidance_pack_v_loop();
	guidance_pack_h_loop();
#ifndef SITL	
	guidance_pack_rc_handler();
	FUTABA_SBUS_UpdateServos();
#endif
}

//downlink function
void guidance_pack_downlink ( void ) {
	DOWNLINK_SEND_GUIDANCE_PACK(DefaultChannel, DefaultDevice,
								&gp_roll_desired,
								&gp_pitch_desired,
								&gp_yaw_desired,
								&gp_throttle_desired); 
}
