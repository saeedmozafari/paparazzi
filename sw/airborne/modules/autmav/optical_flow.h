#ifndef OPTICAL_FLOW_H
#define OPTICAL_FLOW_H

#include "std.h"
#include "paparazzi.h"
#include <stdio.h>
#include <stdlib.h>

struct optical_flow_data
{
	float vx;
	float vy;
};

union float_to_byte
{
	uint8_t bytes[4];
  	float value;
};


extern union float_to_byte b2f;
extern struct optical_flow_data of_raw_data;
extern struct optical_flow_data of_metric_data;
extern struct optical_flow_data of_corrected_metric_data;

extern uint8_t of_buffer[512];
extern float sonar_range;
extern float focal_length;
extern int write_idx;
extern int msg_length;
extern int ack_fail;
extern int counter;

extern void optical_flow_init(void);
extern void optical_flow_periodic(void);
extern void calc_checksums(uint8_t *, uint8_t *);
extern void parse(void);
extern struct optical_flow_data pixel_per_sec_to_meter_per_sec(struct optical_flow_data);
extern struct optical_flow_data correct_velocity(struct optical_flow_data);

#endif