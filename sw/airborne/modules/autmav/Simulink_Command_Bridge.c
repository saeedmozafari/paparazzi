#include "Simulink_Command_Bridge.h"
#include "modules/autmav/sonar_i2c.h"
#include "mcu_periph/uart.h"
#include "state.h"
#include "subsystems/datalink/telemetry.h"
#include "subsystems/abi.h"
#include "filters/median_filter.h"
#include <stdio.h>
#include <stdlib.h>

#include "firmwares/rotorcraft/stabilization/stabilization_attitude.h"
#include "firmwares/rotorcraft/guidance/guidance_v.h"
#include "autopilot.h"

struct link_device *scb_dev;

#ifndef SCB_PORT
#define SCB_PORT uart3
#endif

#ifndef SCB_BAUDRATE
#define SCB_BAUDRATE B57600
#endif

#ifndef USE_SCB
#define USE_SCB FALSE
#endif

uint8_t scb_buffer[512];
int write_idx = 0;
int msg_length = 0;
int ack_fail = 0;
bool use_scb = false;

union float_to_byte b2f;
struct scb_data scb;


static void send_simulink_commands(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_SIMULINK_COMMANDS(trans, dev, AC_ID,
                                &scb.roll_deg,
                                &scb.pitch_deg,
                                &scb.yaw_deg,
                                &scb.throttle);
}

void simulink_command_bridge_init(void){
	use_scb = USE_SCB;

	scb_dev = &((SCB_PORT).device);
  	uart_periph_set_bits_stop_parity(&SCB_PORT, UBITS_8, USTOP_1, UPARITY_NO);
  	uart_periph_set_baudrate(&SCB_PORT, SCB_BAUDRATE);

  	register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_SIMULINK_COMMANDS, send_simulink_commands);
}

void send_sensor_data(void){
	struct FloatRates *angular_rates = stateGetBodyRates_f();

	uint8_t send_buffer[128];
	union float_to_byte send_f2b;

	send_buffer[0] = 153;
	send_buffer[1] = 34;
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

	send_f2b.value = stateGetNedToBodyEulers_f()->phi;

	send_buffer[16] = send_f2b.bytes[0];
	send_buffer[17] = send_f2b.bytes[1];
	send_buffer[18] = send_f2b.bytes[2];
	send_buffer[19] = send_f2b.bytes[3];

	send_f2b.value = stateGetNedToBodyEulers_f()->theta;

	send_buffer[20] = send_f2b.bytes[0];
	send_buffer[21] = send_f2b.bytes[1];
	send_buffer[22] = send_f2b.bytes[2];
	send_buffer[23] = send_f2b.bytes[3];

	send_f2b.value = stateGetNedToBodyEulers_f()->psi;

	send_buffer[24] = send_f2b.bytes[0];
	send_buffer[25] = send_f2b.bytes[1];
	send_buffer[26] = send_f2b.bytes[2];
	send_buffer[27] = send_f2b.bytes[3];

	send_f2b.value = 0;

	send_buffer[28] = send_f2b.bytes[0];
	send_buffer[29] = send_f2b.bytes[1];
	send_buffer[30] = send_f2b.bytes[2];
	send_buffer[31] = send_f2b.bytes[3];

	uint8_t checksum_a, checksum_b;

	calc_checksums(&checksum_a, &checksum_b, send_buffer, 34);

	send_buffer[32] = checksum_a;
	send_buffer[33] = checksum_b;

	for(int i=0; i<34; i++){
		scb_dev->put_byte(scb_dev->periph, 0, (uint8_t)send_buffer[i]);
	}

}

void simulink_command_bridge_periodic(void){

	while (scb_dev->char_available(scb_dev->periph)){

		if(write_idx == 512)
			write_idx = 0;

		uint8_t response = scb_dev->get_byte(scb_dev->periph);

		if(((int)response == 153) && (write_idx == 0)){ 
			scb_buffer[write_idx] = response;
			write_idx++;
			return;
		}
		if(write_idx == 1){
			msg_length = (int)response;
			scb_buffer[write_idx] = response;
			write_idx++;
			return;	
		}
		if(write_idx == msg_length - 1){ 
			scb_buffer[write_idx] = response;
			write_idx = 0;
					
			uint8_t a, b;
					
			calc_checksums(&a, &b, scb_buffer, msg_length);
			if(a == scb_buffer[msg_length - 2] && b == scb_buffer[msg_length - 1]){
				parse();
				return;
			}
			else{
				ack_fail ++;
				return;
			}
		}
		if(write_idx != 0){
			scb_buffer[write_idx] = response;
			write_idx++;
		}
	}

	if(use_scb){
		
	}
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
	
	switch(scb_buffer[3]){
		case 1:
			b2f.bytes[0] = scb_buffer[4];
			b2f.bytes[1] = scb_buffer[5];
			b2f.bytes[2] = scb_buffer[6];
			b2f.bytes[3] = scb_buffer[7];

			scb.roll_rad = b2f.value;
			scb.roll_deg = scb.roll_rad;

			b2f.bytes[0] = scb_buffer[8];
			b2f.bytes[1] = scb_buffer[9];
			b2f.bytes[2] = scb_buffer[10];
			b2f.bytes[3] = scb_buffer[11];

			scb.pitch_rad = b2f.value;
			scb.pitch_deg = scb.pitch_rad;

			b2f.bytes[0] = scb_buffer[12];
			b2f.bytes[1] = scb_buffer[13];
			b2f.bytes[2] = scb_buffer[14];
			b2f.bytes[3] = scb_buffer[15];

			scb.yaw_rad = b2f.value;
			scb.yaw_deg = scb.yaw_rad;

			b2f.bytes[0] = scb_buffer[16];
			b2f.bytes[1] = scb_buffer[17];
			b2f.bytes[2] = scb_buffer[18];
			b2f.bytes[3] = scb_buffer[19];

			scb.throttle = (int16_t)(b2f.value);

			break;
		case 2:

			break;
		default:
			
			break;
	}
	
}

void guidance_h_module_init(void)
{
 
}

void guidance_h_module_enter(void)
{
  /* Set rool/pitch to 0 degrees and psi to current heading */
}

void guidance_h_module_read_rc(void){
	
}

void guidance_h_module_run(bool in_flight)
{

  struct Int32Eulers sc_cmd;
  sc_cmd.phi = scb.roll_rad;
  sc_cmd.theta = scb.pitch_rad;
  sc_cmd.psi = scb.yaw_rad;

  /* Update the setpoint */
  stabilization_attitude_set_rpy_setpoint_i(&sc_cmd);

  /* Run the default attitude stabilization */
  stabilization_attitude_run(in_flight);
}

void guidance_v_module_init(void)
{
  // initialization of your custom vertical controller goes here
}

// Implement own Vertical loops
void guidance_v_module_enter(void)
{
  // your code that should be executed when entering this vertical mode goes here
}

void guidance_v_module_run(UNUSED bool in_flight)
{
  stabilization_cmd[COMMAND_THRUST] = scb.throttle;
}