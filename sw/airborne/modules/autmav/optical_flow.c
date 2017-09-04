#include "optical_flow.h"
#include "modules/autmav/sonar_i2c.h"
#include "mcu_periph/uart.h"
#include "state.h"
#include "subsystems/datalink/telemetry.h"
#include <stdio.h>
#include <stdlib.h>

struct link_device *of_dev;

#ifndef OF_PORT
#define OF_PORT uart4
#endif

#ifndef OF_BAUDRATE
#define OF_BAUDRATE B57600
#endif

union float_to_byte b2f;
struct optical_flow_data of_raw_data;
struct optical_flow_data of_metric_data;
struct optical_flow_data of_corrected_metric_data;

uint8_t of_buffer[512];
float sonar_range = 1.0;
float focal_length = 150.0;
int write_idx = 0;
int msg_length = 0;
int ack_fail = 0;


static void send_optical_flow(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_OPTICAL_FLOW(trans, dev, AC_ID,
                                &of_raw_data.vx,
                                &of_raw_data.vy,
                                &of_metric_data.vx,
                                &of_metric_data.vy,
                                &of_corrected_metric_data.vx,
                                &of_corrected_metric_data.vy);
}

void optical_flow_init(void){
	of_dev = &((OF_PORT).device);
  	uart_periph_set_bits_stop_parity(&OF_PORT, UBITS_8, USTOP_1, UPARITY_NO);
  	uart_periph_set_baudrate(&OF_PORT, OF_BAUDRATE);

  	register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_OPTICAL_FLOW, send_optical_flow);
}

void optical_flow_periodic(void){

	sonar_range = sonar_i2c.distance;

	of_metric_data = pixel_per_sec_to_meter_per_sec(of_raw_data);
	of_corrected_metric_data = correct_velocity(of_metric_data);

	if (of_dev->char_available(of_dev->periph)){
		uint8_t response = of_dev->get_byte(of_dev->periph);

		if(((int)response == 153) && (write_idx == 0)){ 
			of_buffer[write_idx] = response;
			write_idx++;
			return;
		}
		if(write_idx == 1){
			msg_length = (int)response;
			of_buffer[write_idx] = response;
			write_idx++;
			return;	
		}
		if(write_idx == msg_length - 1){ 
			of_buffer[write_idx] = response;
			write_idx = 0;
					
			uint8_t a, b;
					
			calc_checksums(&a, &b);
			if(a == of_buffer[msg_length - 2] && b == of_buffer[msg_length - 1]){
				parse();
				return;
			}
			else{
				ack_fail ++;
				return;
			}
		}
		if(write_idx != 0){
			of_buffer[write_idx] = response;
			write_idx++;
		}
	}
}

struct optical_flow_data pixel_per_sec_to_meter_per_sec(struct optical_flow_data pixel_per_sec){
	struct optical_flow_data meter_per_sec;

	meter_per_sec.vx = pixel_per_sec.vx * sonar_range / focal_length;
	meter_per_sec.vy = pixel_per_sec.vy * sonar_range / focal_length;

	return meter_per_sec;
}

struct optical_flow_data correct_velocity(struct optical_flow_data not_corrected){
	struct optical_flow_data corrected;

	corrected.vx = not_corrected.vx - state.body_rates_f.q * sonar_range;
	corrected.vy = not_corrected.vy + state.body_rates_f.p * sonar_range;

	return corrected;
}

void calc_checksums(uint8_t *a, uint8_t *b){
	*a = msg_length;
	*b = msg_length;
	
	int i;
	for(i=2; i<msg_length - 2; i++){
		*a = *a + of_buffer[i];
		*b = *b + *a;
	}
}

void parse(void){
	
	switch(of_buffer[3]){
		case 1:
			b2f.bytes[0] = of_buffer[4];
			b2f.bytes[1] = of_buffer[5];
			b2f.bytes[2] = of_buffer[6];
			b2f.bytes[3] = of_buffer[7];

			of_raw_data.vy = -1 * b2f.value;

			b2f.bytes[0] = of_buffer[8];
			b2f.bytes[1] = of_buffer[9];
			b2f.bytes[2] = of_buffer[10];
			b2f.bytes[3] = of_buffer[11];

			of_raw_data.vx = b2f.value;

			break;
		case 2:

			break;
		default:
			
			break;
	}
	
}