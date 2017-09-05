#include "optical_flow.h"
#include "modules/autmav/sonar_i2c.h"
#include "mcu_periph/uart.h"
#include "state.h"
#include "subsystems/datalink/telemetry.h"
#include "subsystems/abi.h"
#include "filters/median_filter.h"
#include <stdio.h>
#include <stdlib.h>

struct link_device *of_dev;
struct MedianFilterInt optical_flow_vx_median_filter;
struct MedianFilterInt optical_flow_vy_median_filter;

#ifndef OF_PORT
#define OF_PORT uart4
#endif

#ifndef OF_BAUDRATE
#define OF_BAUDRATE B57600
#endif

#ifndef USE_OF_STAB
#define USE_OF_STAB FALSE
#endif

union float_to_byte b2f;
struct optical_flow_data of_raw_data;
struct optical_flow_data of_metric_data;
struct optical_flow_data of_corrected_metric_data;

float ma_buffer[10];
float ma_buffer1[10];
uint8_t of_buffer[512];
float sonar_range = 1.0;
float focal_length = 150.0;
float current_p;
float current_q;
float current_r;
int write_idx = 0;
int msg_length = 0;
int ack_fail = 0;
int ma_idx = 0;
bool use_of;


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
	use_of = USE_OF_STAB;

	of_dev = &((OF_PORT).device);
  	uart_periph_set_bits_stop_parity(&OF_PORT, UBITS_8, USTOP_1, UPARITY_NO);
  	uart_periph_set_baudrate(&OF_PORT, OF_BAUDRATE);

  	register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_OPTICAL_FLOW, send_optical_flow);

  	init_median_filter(&optical_flow_vx_median_filter);
  	init_median_filter(&optical_flow_vy_median_filter);
}

void send_rates(void){
	struct FloatRates *angular_rates = stateGetBodyRates_f();

	uint8_t send_buffer[128];
	union float_to_byte send_f2b;

	send_buffer[0] = 153;
	send_buffer[1] = 18;
	send_buffer[2] = 0;
	send_buffer[3] = 1;

	send_f2b.value = angular_rates->p;

	send_buffer[4] = send_f2b.bytes[0];
	send_buffer[5] = send_f2b.bytes[1];
	send_buffer[6] = send_f2b.bytes[2];
	send_buffer[7] = send_f2b.bytes[3];

	send_f2b.value = angular_rates->q;

	send_buffer[8] = send_f2b.bytes[0];
	send_buffer[9] = send_f2b.bytes[1];
	send_buffer[10] = send_f2b.bytes[2];
	send_buffer[11] = send_f2b.bytes[3];

	send_f2b.value = angular_rates->r;

	send_buffer[12] = send_f2b.bytes[0];
	send_buffer[13] = send_f2b.bytes[1];
	send_buffer[14] = send_f2b.bytes[2];
	send_buffer[15] = send_f2b.bytes[3];

	uint8_t checksum_a, checksum_b;

	calc_checksums(&checksum_a, &checksum_b, send_buffer, 18);

	send_buffer[16] = checksum_a;
	send_buffer[17] = checksum_b;

	for(int i=0; i<18; i++){
		of_dev->put_byte(of_dev->periph, 0, (uint8_t)send_buffer[i]);
	}

}

void optical_flow_periodic(void){

	while (of_dev->char_available(of_dev->periph)){
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
					
			calc_checksums(&a, &b, of_buffer, msg_length);
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

	sonar_range = sonar_i2c.distance;

	of_metric_data = pixel_per_sec_to_meter_per_sec(of_raw_data);
	of_corrected_metric_data = correct_velocity(of_metric_data);

	if(use_of)
		AbiSendMsgVELOCITY_ESTIMATE(PX4FLOW_VELOCITY_ID,
                                0,
                                of_metric_data.vx,
                                of_metric_data.vy,
                                0.0f,
                                0.0f);
}

struct optical_flow_data pixel_per_sec_to_meter_per_sec(struct optical_flow_data pixel_per_sec){
	struct optical_flow_data meter_per_sec;

	meter_per_sec.vx = pixel_per_sec.vx * sonar_range / focal_length;
	meter_per_sec.vy = pixel_per_sec.vy * sonar_range / focal_length;

	return meter_per_sec;
}

struct optical_flow_data correct_velocity(struct optical_flow_data not_corrected){
	struct optical_flow_data corrected;

	int32_t res_vx, res_vy;

	res_vx = (not_corrected.vx - current_q * sonar_range) * 10000.0;
	res_vy = (not_corrected.vy + current_p * sonar_range) * 10000.0;

	corrected.vx = update_median_filter(&optical_flow_vx_median_filter, res_vx) / 10000.0;
	corrected.vy = update_median_filter(&optical_flow_vy_median_filter, res_vy) / 10000.0;

	return corrected;
}

void calc_checksums(uint8_t *a, uint8_t *b, uint8_t *send_buffer, int length){
	*a = length;
	*b = length;
	
	int i;
	for(i=2; i<length - 2; i++){
		*a = *a + send_buffer[i];
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

			of_raw_data.vy = b2f.value;

			b2f.bytes[0] = of_buffer[8];
			b2f.bytes[1] = of_buffer[9];
			b2f.bytes[2] = of_buffer[10];
			b2f.bytes[3] = of_buffer[11];

			of_raw_data.vx = -1 * b2f.value;

			b2f.bytes[0] = of_buffer[12];
			b2f.bytes[1] = of_buffer[13];
			b2f.bytes[2] = of_buffer[14];
			b2f.bytes[3] = of_buffer[15];

			current_p = b2f.value;

			b2f.bytes[0] = of_buffer[16];
			b2f.bytes[1] = of_buffer[17];
			b2f.bytes[2] = of_buffer[18];
			b2f.bytes[3] = of_buffer[19];

			current_q = b2f.value;

			b2f.bytes[0] = of_buffer[20];
			b2f.bytes[1] = of_buffer[21];
			b2f.bytes[2] = of_buffer[22];
			b2f.bytes[3] = of_buffer[23];

			current_r = b2f.value;

			break;
		case 2:

			break;
		default:
			
			break;
	}
	
}