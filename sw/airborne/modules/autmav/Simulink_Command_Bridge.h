#ifndef SIMULINK_COMMAND_BRIDGE_H
#define SIMULINK_COMMAND_BRIDGE_H

#include "std.h"
#include "paparazzi.h"
#include <stdio.h>
#include <stdlib.h>
#include "math/pprz_algebra_int.h"

struct scb_data
{
	float roll_rad;
	float pitch_rad;
	float yaw_rad;
	int16_t throttle;
	int16_t roll_deg;
	int16_t pitch_deg;
	int16_t yaw_deg;
};

union float_to_byte
{
	uint8_t bytes[4];
  	float value;
};

extern uint8_t scb_buffer[512];
extern int write_idx;
extern int msg_length;
extern int ack_fail;
extern bool use_scb;

extern union float_to_byte b2f;
extern struct scb_data scb;

extern void simulink_command_bridge_init(void);
extern void simulink_command_bridge_periodic(void);
extern void send_sensor_data(void);
extern void calc_checksums(uint8_t *, uint8_t *, uint8_t *, int);
extern void parse(void);

// Implement own Horizontal loops
extern void guidance_h_module_init(void);
extern void guidance_h_module_enter(void);
extern void guidance_h_module_run(bool in_flight);



// Implement own Vertical loops
extern void guidance_v_module_init(void);
extern void guidance_v_module_enter(void);
extern void guidance_h_module_read_rc(void);
extern void guidance_v_module_run(UNUSED bool in_flight);

#endif