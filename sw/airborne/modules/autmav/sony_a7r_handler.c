#include "sony_a7r_handler.h"
#include "RTK_receive.h"
#include "pprzlink/pprzlink_device.h"
#include "mcu_periph/uart.h"
#include "subsystems/datalink/datalink.h" 
#include "generated/airframe.h"
#include "subsystems/datalink/telemetry.h"
#include "led.h"
#include "mcu_periph/uart.h"
#include "mcu_periph/sys_time.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <math.h>
#include <string.h>

struct link_device *wifi_command;

#ifndef ESP_01_BAUD
#define ESP_01_BAUD B115200
#endif

#ifndef ESP_01_UART_PORT 
#define ESP_01_UART_PORT uart6
#endif

#ifndef ISO_SPEED_RATE
#define ISO_SPEED_RATE 400
#endif

#ifndef SHUTTER_SPEED
#define SHUTTER_SPEED 0.008
#endif

#ifndef FNUMBER
#define FNUMBER 2.8
#endif

int result_counter = 0;
int name_counter = 0;
int delay_counter = 0;
uint8_t tcp_connection_errors = 0;
int udp_connection_errors = 0;
int time_counter = 0;
int last_recieved_setting = 0;
uint8_t last_set_setting = 0;
uint8_t connection_counter = 0;
int shutter_speed_response_counter = 0;
int iso_response_counter = 0;
int fnumber_response_counter = 0;
uint16_t sival = 160;
bool result_read = false;
bool mode_set = false;
bool tcp_connected = false;
bool word_processing = false;
bool already_connected = false;
bool delay_mode = false;
bool chars_recieved = false;
bool image_name_started = false;
bool image_name_finished = false;
bool available_settings_received = false;
bool settings_set = false;
bool enable_camera = false;
char image_name[100];
char final_name[100];
char shutter_speed_response[1000];
char iso_response[1000];
char fnumber_response[1000];
char ready_test[4] = "AT\r\n";
float curr_time = 0.0;
float start_time = 0.0;
float sfval = 2.8;
float ssvalue = 0.000625;
FILE *image_names;

enum cam_state sony_a7r_state;
enum parser_status camera_parser_status;
enum cam_order camera_order;
enum camera_model cam_model;

static void send_camera_state(struct transport_tx *trans, struct link_device *dev)
 {
   	uint8_t cam_state_for_debug = sony_a7r_state;
   	//uint8_t parser_state_for_debug = camera_parser_status;
	pprz_msg_send_SONY_CAMERA_STATUS(trans, dev, AC_ID,
                         &cam_state_for_debug,
                         &tcp_connection_errors);
}

void sony_a7r_handler_setup(void){
	wifi_command = &((ESP_01_UART_PORT).device);
	uart_periph_set_bits_stop_parity(&ESP_01_UART_PORT, UBITS_8, USTOP_1, UPARITY_NO);
	uart_periph_set_baudrate(&ESP_01_UART_PORT, ESP_01_BAUD);
	register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_SONY_CAMERA_STATUS, send_camera_state);
	clear_image_name();
}

void esp_01_jap(void){
	LED_ON(3);
	char jap_msg[45];

	if(cam_model == SONY_A7R){
		char temp_msg[45] = "AT+CWJAP=\"DIRECT-PXE0:ILCE-7R\",\"wLmAKDZS\"\r\n";
		strncpy(jap_msg, temp_msg, 45);
	}
	if(cam_model == SONY_A6000){
		char temp_msg[45] = "AT+CWJAP=\"DIRECT-pAE0:ILCE-6000\",\"6ioy7sQ3\"\r\n";
		strncpy(jap_msg, temp_msg, 45);
	}
	if(cam_model == SONY_QX1){
		char temp_msg[45] = "AT+CWJAP=\"DIRECT-HuQ1:ILCE-QX1\",\"GtTFG9ij\"\r\n";
		strncpy(jap_msg, temp_msg, 45);
	}

	for(int i=0; i<45; i++){
		wifi_command->put_byte(wifi_command->periph, 0, (uint8_t)jap_msg[i]);
	}
	
}

void esp_01_cipmux(void){
	char multi_con_msg[13] = "AT+CIPMUX=1\r\n";
	for(int i=0; i<13; i++){
		wifi_command->put_byte(wifi_command->periph, 0, (uint8_t)multi_con_msg[i]);
	}
}

void esp_01_cipstart(void){
	char start_con_msg[45] = "AT+CIPSTART=0,\"UDP\",\"239.255.255.250\",1900\r\n";
	for(int i=0; i<45; i++){
		wifi_command->put_byte(wifi_command->periph, 0, (uint8_t)start_con_msg[i]);
	}
}

void esp_01_cipsend(void){
	char send_command_msg[19] = "AT+CIPSEND=0,125\r\n";
	for(int i=0; i<18; i++){
		wifi_command->put_byte(wifi_command->periph, 0, (uint8_t)send_command_msg[i]);
	}
}

void esp_01_msearch(void){
	char request_msg[125] = "M-SEARCH * HTTP/1.1\r\nHOST:239.255.255.250:1900\r\nMAN:\"ssdp:discover\"\r\nMX:1\r\nST:urn:schemas-sony-com:service:ScalarWebAPI:1\r\n\r\n";
	for(int i=0; i<125; i++){
		wifi_command->put_byte(wifi_command->periph, 0, (uint8_t)request_msg[i]);
	}
}

void esp_01_cipclose(void){
	char close_msg[15] = "AT+CIPCLOSE=0\r\n";
	for(int i=0; i<15; i++){
		wifi_command->put_byte(wifi_command->periph, 0, (uint8_t)close_msg[i]);
	}
}

void esp_01_cipstart_tcp(void){
	char start_con_msg_tcp[44] = "AT+CIPSTART=0,\"TCP\",\"192.168.122.1\",8080\r\n";
	for(int i=0; i<44; i++){
		wifi_command->put_byte(wifi_command->periph, 0, (uint8_t)start_con_msg_tcp[i]);
	}
}

void esp_01_cipsend_tcp(void){
	char send_command_msg_tcp[18] = "AT+CIPSEND=0,264\r\n";
	for(int i=0; i<18; i++){
		wifi_command->put_byte(wifi_command->periph, 0, (uint8_t)send_command_msg_tcp[i]);
	}
}

void esp_01_setmode(void){
	char prepare_to_shoot_msg[264] = "POST /sony/camera HTTP/1.1\r\nContent-Type: application/json; charset=utf-8\r\nAccept: Accept-application/json\r\nHost: 192.168.122.1:8080\r\nContent-Length: 60\r\nExpect: 100-continue\r\nConnection: Keep-Alive\r\n\r\n{\"method\":\"startRecMode\",\"params\":[],\"id\":1,\"version\":\"1.0\"}\r\n";
	for(int i=0; i<264; i++){
		wifi_command->put_byte(wifi_command->periph, 0, (uint8_t)prepare_to_shoot_msg[i]);
	}
}

void esp_01_cipsend_shoot(void){
	char send_command_msg[18] = "AT+CIPSEND=0,266\r\n";
	for(int i=0; i<18; i++){
		wifi_command->put_byte(wifi_command->periph, 0, (uint8_t)send_command_msg[i]);
	}
}

void sony_a7r_shoot(void){
	char shoot_command_msg[266] = "POST /sony/camera HTTP/1.1\r\nContent-Type: application/json; charset=utf-8\r\nAccept: Accept-application/json\r\nHost: 192.168.122.1:8080\r\nContent-Length: 62\r\nExpect: 100-continue\r\nConnection: Keep-Alive\r\n\r\n{\"method\":\"actTakePicture\",\"params\":[],\"id\":1,\"version\":\"1.0\"}\r\n";
	for(int i=0; i<266; i++){
		wifi_command->put_byte(wifi_command->periph, 0, (uint8_t)shoot_command_msg[i]);
	}
}

void check_esp_01_connection_status(void){
	char check_connection_msg[14] = "AT+CIPSTATUS\r\n";
	for(int i=0; i<14; i++){
		wifi_command->put_byte(wifi_command->periph, 0, (uint8_t)check_connection_msg[i]);
	}
}

void esp_01_cipsend_setting_as(void){
	char send_command_msg[18] = "AT+CIPSEND=0,276\r\n";
	for(int i=0; i<18; i++){
		wifi_command->put_byte(wifi_command->periph, 0, (uint8_t)send_command_msg[i]);
	}
}

void get_available_shutter_speed(void){
	char available_shutter_speeds_msg[276] = "POST /sony/camera HTTP/1.1\r\nContent-Type: application/json; charset=utf-8\r\nAccept: Accept-application/json\r\nHost: 192.168.122.1:8080\r\nContent-Length: 72\r\nExpect: 100-continue\r\nConnection: Keep-Alive\r\n\r\n{\"method\":\"getAvailableShutterSpeed\",\"params\":[],\"id\":1,\"version\":\"1.0\"}\r\n";
	for(int i=0; i<276; i++){
		wifi_command->put_byte(wifi_command->periph, 0, (uint8_t)available_shutter_speeds_msg[i]);
	}
}

void esp_01_cipsend_setting_gs(void){
	char send_command_msg[18] = "AT+CIPSEND=0,267\r\n";
	for(int i=0; i<18; i++){
		wifi_command->put_byte(wifi_command->periph, 0, (uint8_t)send_command_msg[i]);
	}
}

void get_current_shutter_speed(void){
	char get_shutter_speeds_msg[267] = "POST /sony/camera HTTP/1.1\r\nContent-Type: application/json; charset=utf-8\r\nAccept: Accept-application/json\r\nHost: 192.168.122.1:8080\r\nContent-Length: 63\r\nExpect: 100-continue\r\nConnection: Keep-Alive\r\n\r\n{\"method\":\"getShutterSpeed\",\"params\":[],\"id\":1,\"version\":\"1.0\"}\r\n";
	for(int i=0; i<267; i++){
		wifi_command->put_byte(wifi_command->periph, 0, (uint8_t)get_shutter_speeds_msg[i]);
	}
}

void esp_01_cipsend_setting_ss(void){
	char send_command_msg[18] = "AT+CIPSEND=0,278\r\n";
	for(int i=0; i<18; i++){
		wifi_command->put_byte(wifi_command->periph, 0, (uint8_t)send_command_msg[i]);
	}
}

void set_shutter_speed(float val1){
	char value[9];
	char *char_ptr;

	if(val1 == 30 ){
		char temp_value[9] = "30";
		strncpy(value, temp_value, 9);
		char_ptr = temp_value;
	}
	if(val1 == 25){
		char temp_value[9] = "25";
		strncpy(value, temp_value, 9);
		char_ptr = temp_value;
	}
	if(val1 == 20){
		char temp_value[9] = "20";
		strncpy(value, temp_value, 9);
		char_ptr = temp_value;
	}
	if(val1 == 15){
		char temp_value[9] = "15";
		strncpy(value, temp_value, 9);
		char_ptr = temp_value;
	}
	if(val1 == 13){
		char temp_value[9] = "13";
		strncpy(value, temp_value, 9);
		char_ptr = temp_value;
	}
	if(val1 == 10){
		char temp_value[9] = "10";
		strncpy(value, temp_value, 9);
		char_ptr = temp_value;
	}
	if(val1 == 8){
		char temp_value[9] = "8";
		strncpy(value, temp_value, 9);
		char_ptr = temp_value;
	}
	if(val1 == 6){
		char temp_value[9] = "6";
		strncpy(value, temp_value, 9);
		char_ptr = temp_value;
	}
	if(val1 == 5){
		char temp_value[9] = "5";
		strncpy(value, temp_value, 9);
		char_ptr = temp_value;
	}
	if(val1 == 4){
		char temp_value[9] = "4";
		strncpy(value, temp_value, 9);
		char_ptr = temp_value;
	}
	if(val1 == 3.2){
		char temp_value[9] = "3.2";
		strncpy(value, temp_value, 9);
		char_ptr = temp_value;
	}
	if(val1 == 2.5){
		char temp_value[9] = "2.5";
		strncpy(value, temp_value, 9);
		char_ptr = temp_value;
	}
	if(val1 == 2){
		char temp_value[9] = "2";
		strncpy(value, temp_value, 9);
		char_ptr = temp_value;
	}
	if(val1 == 1.6){
		char temp_value[9] = "1.6";
		strncpy(value, temp_value, 9);
		char_ptr = temp_value;
	}
	if(val1 == 1.3){
		char temp_value[9] = "1.3";
		strncpy(value, temp_value, 9);
		char_ptr = temp_value;
	}
	if(val1 == 1){
		char temp_value[9] = "1";
		strncpy(value, temp_value, 9);
		char_ptr = temp_value;
	}
	if(val1 == 0.8){
		char temp_value[9] = "0.8";
		strncpy(value, temp_value, 9);
		char_ptr = temp_value;
	}
	if(val1 == 0.6){
		char temp_value[9] = "0.6";
		strncpy(value, temp_value, 9);
		char_ptr = temp_value;
	}
	if(val1 == 0.5){
		char temp_value[9] = "0.5";
		strncpy(value, temp_value, 9);
		char_ptr = temp_value;
	}
	if(val1 == 0.4){
		char temp_value[9] = "0.4";
		strncpy(value, temp_value, 9);
		char_ptr = temp_value;
	}
	if(val1 == 0.25){
		char temp_value[9] = "0.25";
		strncpy(value, temp_value, 9);
		char_ptr = temp_value;
	}
	if(val1 == 0.2){
		char temp_value[9] = "0.2";
		strncpy(value, temp_value, 9);
		char_ptr = temp_value;
	}
	if(val1 == 0.125){
		char temp_value[9] = "0.125";
		strncpy(value, temp_value, 9);
		char_ptr = temp_value;
	}
	if(val1 == 0.1){
		char temp_value[9] = "0.1";
		strncpy(value, temp_value, 9);
		char_ptr = temp_value;
	}
	if(val1 == 0.05){
		char temp_value[9] = "0.05";
		strncpy(value, temp_value, 9);
		char_ptr = temp_value;
	}
	if(val1 == 0.04){
		char temp_value[9] = "0.04";
		strncpy(value, temp_value, 9);
		char_ptr = temp_value;
	}
	if(val1 == 0.025){
		char temp_value[9] = "0.025";
		strncpy(value, temp_value, 9);
		char_ptr = temp_value;
	}
	if(val1 == 0.02){
		char temp_value[9] = "0.02";
		strncpy(value, temp_value, 9);
		char_ptr = temp_value;
	}
	if(val1 == 0.0125){
		char temp_value[9] = "0.0125";
		strncpy(value, temp_value, 9);
		char_ptr = temp_value;
	}
	if(val1 == 0.01){
		char temp_value[9] = "0.01";
		strncpy(value, temp_value, 9);
		char_ptr = temp_value;
	}
	if(val1 == 0.008){
		char temp_value[9] = "0.008";
		strncpy(value, temp_value, 9);
		char_ptr = temp_value;
	}
	if(val1 == 0.005){
		char temp_value[9] = "0.005";
		strncpy(value, temp_value, 9);
		char_ptr = temp_value;
	}
	if(val1 == 0.004){
		char temp_value[9] = "0.004";
		strncpy(value, temp_value, 9);
		char_ptr = temp_value;
	}
	if(val1 == 0.003125){
		char temp_value[9] = "0.003125";
		strncpy(value, temp_value, 9);
		char_ptr = temp_value;
	}
	if(val1 == 0.0025){
		char temp_value[9] = "0.0025";
		strncpy(value, temp_value, 9);
		char_ptr = temp_value;
	}
	if(val1 == 0.002){
		char temp_value[9] = "0.002";
		strncpy(value, temp_value, 9);
		char_ptr = temp_value;
	}
	if(val1 == 0.0015625){
		char temp_value[9] = "0.0015625";
		strncpy(value, temp_value, 9);
		char_ptr = temp_value;
	}
	if(val1 == 0.00125){
		char temp_value[9] = "0.00125";
		strncpy(value, temp_value, 9);
		char_ptr = temp_value;
	}
	if(val1 == 0.001){
		char temp_value[9] = "0.001";
		strncpy(value, temp_value, 9);
		char_ptr = temp_value;
	}
	if(val1 == 0.0008){
		char temp_value[9] = "0.0008";
		strncpy(value, temp_value, 9);
		char_ptr = temp_value;
	}
	if(val1 == 0.000625){
		char temp_value[9] = "0.000625";
		strncpy(value, temp_value, 9);
		char_ptr = temp_value;
	}
	if(val1 == 0.0005){
		char temp_value[9] = "0.0005";
		strncpy(value, temp_value, 9);
		char_ptr = temp_value;
	}
	if(val1 == 0.0004){
		char temp_value[9] = "0.0004";
		strncpy(value, temp_value, 9);
		char_ptr = temp_value;
	}
	if(val1 == 0.0003125){
		char temp_value[9] = "0.0003125";
		strncpy(value, temp_value, 9);
		char_ptr = temp_value;
	}
	if(val1 == 0.00025){
		char temp_value[9] = "0.00025";
		strncpy(value, temp_value, 9);
		char_ptr = temp_value;
	}

	char set_shutter_speeds_msg[278] = "POST /sony/camera HTTP/1.1\r\nContent-Type: application/json; charset=utf-8\r\nAccept: Accept-application/json\r\nHost: 192.168.122.1:8080\r\nContent-Length: 74\r\nExpect: 100-continue\r\nConnection: Keep-Alive\r\n\r\n{\"method\":\"setShutterSpeed\",\"params\":[\"         \"],\"id\":1,\"version\":\"1.0\"}\r\n";
	
	for(int i=0; i<9; i++){
		set_shutter_speeds_msg[i + 241] = value[i];
	}

	for(int i=0; i<278; i++){
		wifi_command->put_byte(wifi_command->periph, 0, (uint8_t)set_shutter_speeds_msg[i]);
	}
}

void esp_01_cipsend_setting_ai(void){
	char send_command_msg[18] = "AT+CIPSEND=0,276\r\n";
	for(int i=0; i<18; i++){
		wifi_command->put_byte(wifi_command->periph, 0, (uint8_t)send_command_msg[i]);
	}
}

void get_available_isos(void){
	char available_isos_msg[276] = "POST /sony/camera HTTP/1.1\r\nContent-Type: application/json; charset=utf-8\r\nAccept: Accept-application/json\r\nHost: 192.168.122.1:8080\r\nContent-Length: 72\r\nExpect: 100-continue\r\nConnection: Keep-Alive\r\n\r\n{\"method\":\"getAvailableIsoSpeedRate\",\"params\":[],\"id\":1,\"version\":\"1.0\"}\r\n";
	for(int i=0; i<276; i++){
		wifi_command->put_byte(wifi_command->periph, 0, (uint8_t)available_isos_msg[i]);
	}
}

void esp_01_cipsend_setting_gi(void){
	char send_command_msg[18] = "AT+CIPSEND=0,267\r\n";
	for(int i=0; i<18; i++){
		wifi_command->put_byte(wifi_command->periph, 0, (uint8_t)send_command_msg[i]);
	}
}

void get_current_iso(void){
	char get_iso_msg[267] = "POST /sony/camera HTTP/1.1\r\nContent-Type: application/json; charset=utf-8\r\nAccept: Accept-application/json\r\nHost: 192.168.122.1:8080\r\nContent-Length: 63\r\nExpect: 100-continue\r\nConnection: Keep-Alive\r\n\r\n{\"method\":\"getIsoSpeedRate\",\"params\":[],\"id\":1,\"version\":\"1.0\"}\r\n";
	for(int i=0; i<267; i++){
		wifi_command->put_byte(wifi_command->periph, 0, (uint8_t)get_iso_msg[i]);
	}
}

void esp_01_cipsend_setting_si(void){
	char send_command_msg[18] = "AT+CIPSEND=0,274\r\n";
	for(int i=0; i<18; i++){
		wifi_command->put_byte(wifi_command->periph, 0, (uint8_t)send_command_msg[i]);
	}
}

void set_iso(uint16_t val){
	char value[5];
	char *char_ptr;

	if(val == 0){
		char temp_value[5] = "AUTO";
		strncpy(value, temp_value, 5);
		char_ptr = temp_value;
	}
	if(val == 100){
		char temp_value[5] = "100";
		strncpy(value, temp_value, 5);
		char_ptr = temp_value;
	}
	if(val == 125){
		char temp_value[5] = "125";
		strncpy(value, temp_value, 5);
		char_ptr = temp_value;
	}
	if(val == 160){
		char temp_value[5] = "160";
		strncpy(value, temp_value, 5);
		char_ptr = temp_value;
	}
	if(val == 200){
		char temp_value[5] = "200";
		strncpy(value, temp_value, 5);
		char_ptr = temp_value;
	}
	if(val == 250){
		char temp_value[5] = "250";
		strncpy(value, temp_value, 5);
		char_ptr = temp_value;
	}
	if(val == 320){
		char temp_value[5] = "320";
		strncpy(value, temp_value, 5);
		char_ptr = temp_value;
	}
	if(val == 400){
		char temp_value[5] = "400";
		strncpy(value, temp_value, 5);
		char_ptr = temp_value;
	}
	if(val == 500){
		char temp_value[5] = "500";
		strncpy(value, temp_value, 5);
		char_ptr = temp_value;
	}
	if(val == 640){
		char temp_value[5] = "640";
		strncpy(value, temp_value, 5);
		char_ptr = temp_value;
	}
	if(val == 800){
		char temp_value[5] = "800";
		strncpy(value, temp_value, 5);
		char_ptr = temp_value;
	}
	if(val == 1000){
		char temp_value[5] = "1000";
		strncpy(value, temp_value, 5);
		char_ptr = temp_value;
	}
	if(val == 1250){
		char temp_value[5] = "1250";
		strncpy(value, temp_value, 5);
		char_ptr = temp_value;
	}
	if(val == 1600){
		char temp_value[5] = "1600";
		strncpy(value, temp_value, 5);
		char_ptr = temp_value;
	}
	if(val == 2000){
		char temp_value[5] = "2000";
		strncpy(value, temp_value, 5);
		char_ptr = temp_value;
	}
	if(val == 2500){
		char temp_value[5] = "2500";
		strncpy(value, temp_value, 5);
		char_ptr = temp_value;
	}
	if(val == 3200){
		char temp_value[5] = "3200";
		strncpy(value, temp_value, 5);
		char_ptr = temp_value;
	}
	if(val == 4000){
		char temp_value[5] = "4000";
		strncpy(value, temp_value, 5);
		char_ptr = temp_value;
	}
	if(val == 5000){
		char temp_value[5] = "5000";
		strncpy(value, temp_value, 5);
		char_ptr = temp_value;
	}
	if(val == 6400){
		char temp_value[5] = "6400";
		strncpy(value, temp_value, 5);
		char_ptr = temp_value;
	}
	if(val == 8000){
		char temp_value[5] = "8000";
		strncpy(value, temp_value, 5);
		char_ptr = temp_value;
	}
	if(val == 10000){
		char temp_value[5] = "10000";
		strncpy(value, temp_value, 5);
		char_ptr = temp_value;
	}
	if(val == 12800){
		char temp_value[5] = "12800";
		strncpy(value, temp_value, 5);
		char_ptr = temp_value;
	}
	if(val == 16000){
		char temp_value[5] = "16000";
		strncpy(value, temp_value, 5);
		char_ptr = temp_value;
	}

	char set_iso_msg[274] = "POST /sony/camera HTTP/1.1\r\nContent-Type: application/json; charset=utf-8\r\nAccept: Accept-application/json\r\nHost: 192.168.122.1:8080\r\nContent-Length: 70\r\nExpect: 100-continue\r\nConnection: Keep-Alive\r\n\r\n{\"method\":\"setIsoSpeedRate\",\"params\":[\"     \"],\"id\":1,\"version\":\"1.0\"}\r\n";
	for(int i=0; i<5; i++){
		set_iso_msg[241 + i] = value[i];
	}

	for(int i=0; i<274; i++){
		wifi_command->put_byte(wifi_command->periph, 0, (uint8_t)set_iso_msg[i]);
	}
}

void esp_01_cipsend_setting_af(void){
	char send_command_msg[18] = "AT+CIPSEND=0,271\r\n";
	for(int i=0; i<18; i++){
		wifi_command->put_byte(wifi_command->periph, 0, (uint8_t)send_command_msg[i]);
	}
}

void get_available_fnumbers(void){
	char available_fnumbers_msg[271] = "POST /sony/camera HTTP/1.1\r\nContent-Type: application/json; charset=utf-8\r\nAccept: Accept-application/json\r\nHost: 192.168.122.1:8080\r\nContent-Length: 67\r\nExpect: 100-continue\r\nConnection: Keep-Alive\r\n\r\n{\"method\":\"getAvailableFNumber\",\"params\":[],\"id\":1,\"version\":\"1.0\"}\r\n";
	for(int i=0; i<271; i++){
		wifi_command->put_byte(wifi_command->periph, 0, (uint8_t)available_fnumbers_msg[i]);
	}
}

void esp_01_cipsend_setting_gf(void){
	char send_command_msg[18] = "AT+CIPSEND=0,262\r\n";
	for(int i=0; i<18; i++){
		wifi_command->put_byte(wifi_command->periph, 0, (uint8_t)send_command_msg[i]);
	}
}

void get_current_fnumber(void){
	char get_fnumber_msg[262] = "POST /sony/camera HTTP/1.1\r\nContent-Type: application/json; charset=utf-8\r\nAccept: Accept-application/json\r\nHost: 192.168.122.1:8080\r\nContent-Length: 58\r\nExpect: 100-continue\r\nConnection: Keep-Alive\r\n\r\n{\"method\":\"getFNumber\",\"params\":[],\"id\":1,\"version\":\"1.0\"}\r\n";
	for(int i=0; i<262; i++){
		wifi_command->put_byte(wifi_command->periph, 0, (uint8_t)get_fnumber_msg[i]);
	}
}

void esp_01_cipsend_setting_sf(void){
	char send_command_msg[18] = "AT+CIPSEND=0,267\r\n";
	for(int i=0; i<18; i++){
		wifi_command->put_byte(wifi_command->periph, 0, (uint8_t)send_command_msg[i]);
	}
}

void set_fnumber(float val){
	char value[3];
	char *char_ptr;

	if(val == 2.8){
		char temp_value[3] = "2.8";
		strncpy(value, temp_value, 3);
		char_ptr = temp_value; 
	}
	if(val == 3.5){
		char temp_value[3] = "3.5";
		strncpy(value, temp_value, 3);
		char_ptr = temp_value; 
	}
	if(val == 4.0){
		char temp_value[3] = "4.0";
		strncpy(value, temp_value, 3);
		char_ptr = temp_value; 
	}
	if(val == 4.5){
		char temp_value[3] = "4.5";
		strncpy(value, temp_value, 3);
		char_ptr = temp_value; 
	}
	if(val == 5.0){
		char temp_value[3] = "5.0";
		strncpy(value, temp_value, 3);
		char_ptr = temp_value; 
	}
	if(val == 5.6){
		char temp_value[3] = "5.6";
		strncpy(value, temp_value, 3);
		char_ptr = temp_value; 
	}
	if(val == 6.3){
		char temp_value[3] = "6.3";
		strncpy(value, temp_value, 3);
		char_ptr = temp_value; 
	}
	if(val == 7.1){
		char temp_value[3] = "7.1";
		strncpy(value, temp_value, 3);
		char_ptr = temp_value; 
	}
	if(val == 8.0){
		char temp_value[3] = "8.0";
		strncpy(value, temp_value, 3);
		char_ptr = temp_value; 
	}
	if(val == 9.0){
		char temp_value[3] = "9.0";
		strncpy(value, temp_value, 3);
		char_ptr = temp_value; 
	}
	if(val == 10){
		char temp_value[3] = "10";
		strncpy(value, temp_value, 3);
		char_ptr = temp_value; 
	}
	if(val == 11){
		char temp_value[3] = "11";
		strncpy(value, temp_value, 3);
		char_ptr = temp_value; 
	}
	if(val == 13){
		char temp_value[3] = "13";
		strncpy(value, temp_value, 3);
		char_ptr = temp_value; 
	}
	if(val == 14){
		char temp_value[3] = "14";
		strncpy(value, temp_value, 3);
		char_ptr = temp_value; 
	}
	if(val == 16){
		char temp_value[3] = "16";
		strncpy(value, temp_value, 3);
		char_ptr = temp_value; 
	}
	if(val == 18){
		char temp_value[3] = "18";
		strncpy(value, temp_value, 3);
		char_ptr = temp_value; 
	}
	if(val == 20){
		char temp_value[3] = "20";
		strncpy(value, temp_value, 3);
		char_ptr = temp_value; 
	}
	if(val == 22){
		char temp_value[3] = "22";
		strncpy(value, temp_value, 3);
		char_ptr = temp_value; 
	}

	char set_fnumber_msg[267] = "POST /sony/camera HTTP/1.1\r\nContent-Type: application/json; charset=utf-8\r\nAccept: Accept-application/json\r\nHost: 192.168.122.1:8080\r\nContent-Length: 63\r\nExpect: 100-continue\r\nConnection: Keep-Alive\r\n\r\n{\"method\":\"setFNumber\",\"params\":[\"   \"],\"id\":1,\"version\":\"1.0\"}\r\n";
	for(int i=0; i<3; i++){
		set_fnumber_msg[236 + i] = value[i];
	}

	for(int i=0; i<267; i++){
		wifi_command->put_byte(wifi_command->periph, 0, (uint8_t)set_fnumber_msg[i]);
	}
}

void endline(void){
	char shoot_command_msg[2] = "\r\n";
	for(int i=0; i<2; i++){
		wifi_command->put_byte(wifi_command->periph, 0, (uint8_t)shoot_command_msg[i]);
	}
}

void sony_a7r_handler_periodic(void){
	char curr_byte;
if(enable_camera){
	switch(sony_a7r_state){
		case READY:
			
			for(int i=0; i<4; i++){
				wifi_command->put_byte(wifi_command->periph, 0, (uint8_t)ready_test[i]);
			}
			sony_a7r_state = WAIT_FOR_READY;

		break;
		case WAIT_FOR_READY:
			while(wifi_command->char_available(wifi_command->periph)){
				wifi_response_parser(wifi_command->get_byte(wifi_command->periph));
				if(camera_parser_status == GOT_OK){
					sony_a7r_state = CONNECTING_TO_CAM_AP;
				}
				if(camera_parser_status == GOT_ERROR){
					sony_a7r_state = READY;
				}
			}
		break;
		case CONNECTING_TO_CAM_AP:
			esp_01_jap();
			sony_a7r_state = WAIT_FOR_CONNECTING_TO_CAM_AP;
		break;
		case WAIT_FOR_CONNECTING_TO_CAM_AP:
			while(wifi_command->char_available(wifi_command->periph)){
				wifi_response_parser(wifi_command->get_byte(wifi_command->periph));
				if(camera_parser_status == GOT_OK){
					sony_a7r_state = SETTING_CIPMUX;
				}
				if(camera_parser_status == GOT_FAIL || camera_parser_status == GOT_ERROR){
					if(connection_counter < 3){
						settings_set = true;
						sony_a7r_state = CONNECTING_TO_CAM_AP;
					}
					else{
						settings_set = false;
						uint8_t two = 2;
						DOWNLINK_SEND_CAMERA_SETTINGS(DefaultChannel, DefaultDevice,
														&ssvalue,
														&sival,
														&sfval,
														&last_set_setting,
														&two);
						enable_camera = false;
						sony_a7r_state = READY;
						break;
					}

					connection_counter++;
				}
			}
		break;
		case SETTING_CIPMUX:
			esp_01_cipmux();
			sony_a7r_state = WAIT_FOR_SETTING_CIPMUX;
		break;
		case WAIT_FOR_SETTING_CIPMUX:
			while(wifi_command->char_available(wifi_command->periph)){
				wifi_response_parser(wifi_command->get_byte(wifi_command->periph));
				if(camera_parser_status == GOT_OK){
					if(cam_model != SONY_QX1){
						sony_a7r_state = STARTING_TCP_CONNECTION;
					}
					else{
						sony_a7r_state = CAM_IDLE_MODE;
						tcp_connected = true;
					}
				}
				if(camera_parser_status == GOT_FAIL || camera_parser_status == GOT_ERROR){
					sony_a7r_state = SETTING_CIPMUX;
				}
			}
		break;
		case STARTING_TCP_CONNECTION:
			esp_01_cipstart_tcp();
			sony_a7r_state = WAIT_FOR_STARTING_TCP_CONNECTION;
			already_connected = false;
			delay_mode = false;

		break;
		case WAIT_FOR_STARTING_TCP_CONNECTION:
			while(wifi_command->char_available(wifi_command->periph)){
				wifi_response_parser(wifi_command->get_byte(wifi_command->periph));
				if(camera_parser_status == GOT_OK){
					delay_mode = true;
					delay_counter = 0;
				}
				if(camera_parser_status == GOT_ALREADY){
					already_connected = true;
					sony_a7r_state = REQUESTING_SEND_TCP;
				}
				if(camera_parser_status == GOT_ERROR && !already_connected){
					if(tcp_connection_errors < 3){
						sony_a7r_state = STARTING_TCP_CONNECTION;
						tcp_connection_errors++;
					}
					else{
						tcp_connection_errors = 0;
						sony_a7r_state = CONNECTING_TO_CAM_AP;
					}
				}
			}
			if(delay_mode){
				if(delay_counter < 20){
					delay_counter++;
				}
				else{
					sony_a7r_state = REQUESTING_SEND_TCP;
					delay_mode = false;
				}
			}
		break;
		case REQUESTING_SEND_TCP:
			if(!mode_set){
				esp_01_cipsend_tcp();
				sony_a7r_state = WAIT_FOR_REQUESTING_SEND_TCP;
			}
			else{
				
				if(last_set_setting == 1){
					sony_a7r_state = SENDING_REQUEST_SS;
				}
				else if(last_set_setting == 2){
					sony_a7r_state = SENDING_REQUEST_SI;
				}
				else if(last_set_setting == 3){
					sony_a7r_state = SENDING_REQUEST_SF;
				}
				else if(last_set_setting == 0){
					sony_a7r_state = CAM_IDLE_MODE;
					tcp_connected = true;
				}
				
			}
		break;
		case WAIT_FOR_REQUESTING_SEND_TCP:
			while(wifi_command->char_available(wifi_command->periph)){
				wifi_response_parser(wifi_command->get_byte(wifi_command->periph));
				if(camera_parser_status == GOT_OK){
					sony_a7r_state = SENDING_SHOOT_MODE_MSG;
				}
				if(camera_parser_status == GOT_ERROR){
					sony_a7r_state = REQUESTING_SEND_TCP;
				}
			}
		break;
		case SENDING_SHOOT_MODE_MSG:
			esp_01_setmode();
			sony_a7r_state = WAIT_FOR_SENDING_SHOOT_MODE_MSG;
			result_read = false;
		break;
		case WAIT_FOR_SENDING_SHOOT_MODE_MSG:
			while(wifi_command->char_available(wifi_command->periph)){
				curr_byte = wifi_command->get_byte(wifi_command->periph);
				wifi_response_parser(curr_byte);
				if(camera_parser_status == GOT_result){
					result_read = true;
					result_counter = 0;
				}
				if(result_read){
					if(result_counter == 4){
						if(curr_byte == '0'){
							settings_set = true;
						}
						else{
							settings_set = false;
						}
					}
					result_counter++;
				}
				if(camera_parser_status == GOT_FAIL){
					settings_set = false;
				}
				if(camera_parser_status == GOT_CLOSE){
					delay_mode = true;
					delay_counter = 0;
				}
			}
			if(delay_mode){
				if(delay_counter < 1000){
					delay_counter++;
				}
				else{
					if(settings_set){
						sony_a7r_state = MODE_MSG_SENT;
						delay_mode = false;
					}
					else{
						sony_a7r_state = STARTING_TCP_CONNECTION;
						delay_mode = false;
					}
				}
			}
		break;
		case MODE_MSG_SENT:
			mode_set = true;
			sony_a7r_state = CAM_IDLE_MODE;	
		break;


		case CAM_IDLE_MODE:
			
				switch(camera_order){
					case CAMERA_IDLE:

					break;
					case CAMERA_SHOOT:
						if(!tcp_connected){
							sony_a7r_state = STARTING_TCP_CONNECTION;
						}
						else{
							esp_01_cipsend_shoot();
							sony_a7r_state = WAIT_FOR_REQUESTING_SEND_SHOOT_TCP;
						}
					break;
					case SET_CAMERA_PARAMETERS:
						
						sony_a7r_state = STARTING_TCP_CONNECTION;
						last_set_setting = 1;
					
					break;
				}
			
		break;


		case SENDING_REQUEST_SS:
			esp_01_cipsend_setting_ss();
			sony_a7r_state = WAIT_FOR_SS_REQUEST_TO_BE_SENT;
			delay_counter = 0;
			delay_mode = false;
		break;
		case WAIT_FOR_SS_REQUEST_TO_BE_SENT:
			while(wifi_command->char_available(wifi_command->periph)){
				wifi_response_parser(wifi_command->get_byte(wifi_command->periph));
				if(camera_parser_status == GOT_OK){
					delay_mode = true;
				}
				if(camera_parser_status == GOT_ERROR){
					sony_a7r_state = SENDING_REQUEST_SS;
				}
			}
			if(delay_mode){

				if(delay_counter < 20){
					delay_counter++;
				}
				else{
					sony_a7r_state = SETTING_SHUTTER_SPEED;
					delay_mode = false;
				}
			}
		break;
		case SETTING_SHUTTER_SPEED:
			set_shutter_speed(ssvalue);
			sony_a7r_state = WAIT_FOR_SHUTTER_SPEED_TO_BE_SET;
			result_counter = 0;
			result_read = false;
		break;
		case WAIT_FOR_SHUTTER_SPEED_TO_BE_SET:
			while(wifi_command->char_available(wifi_command->periph)){
				curr_byte = wifi_command->get_byte(wifi_command->periph);
				wifi_response_parser(curr_byte);
				if(camera_parser_status == GOT_result){
					result_read = true;
				}
				if(result_read){
					if(result_counter == 4){
						if(curr_byte == '0'){
							settings_set = true;
						}
						else{
							settings_set = false;
						}
					}
					result_counter++;
				}
				if(camera_parser_status == GOT_CLOSE){
					delay_mode = true;
					delay_counter = 0;
				}
			}
			if(delay_mode){

				if(delay_counter < 20){
					delay_counter++;
				}
				else{
					last_set_setting = 2;
					if(settings_set){
						sony_a7r_state = STARTING_TCP_CONNECTION;
					}
					else{
						tcp_connected = false;
						sony_a7r_state = CAM_IDLE_MODE;
						camera_order = CAMERA_IDLE;
						enable_camera = false;
						uint8_t zero = 0;
						DOWNLINK_SEND_CAMERA_SETTINGS(DefaultChannel, DefaultDevice,
														&ssvalue,
														&sival,
														&sfval,
														&last_set_setting,
														&zero);
					}
					delay_mode = false;
				}

			}
		break;
		case SENDING_REQUEST_SI:
			esp_01_cipsend_setting_si();
			sony_a7r_state = WAIT_FOR_SI_REQUEST_TO_BE_SENT;
			delay_counter = 0;
			delay_mode = false;
		break;
		case WAIT_FOR_SI_REQUEST_TO_BE_SENT:
			while(wifi_command->char_available(wifi_command->periph)){
				wifi_response_parser(wifi_command->get_byte(wifi_command->periph));
				if(camera_parser_status == GOT_OK){
					delay_mode = true;
				}
				if(camera_parser_status == GOT_ERROR){
					sony_a7r_state = SENDING_REQUEST_SI;
				}
			}
			if(delay_mode){

				if(delay_counter < 20){
					delay_counter++;
				}
				else{
					sony_a7r_state = SETTING_ISO;
					delay_mode = false;
				}
			}
		break;
		case SETTING_ISO:
			set_iso(sival);
			sony_a7r_state = WAIT_FOR_ISO_TO_BE_SET;
			result_counter = 0;
			result_read = false;
		break;
		case WAIT_FOR_ISO_TO_BE_SET:
			while(wifi_command->char_available(wifi_command->periph)){
				curr_byte = wifi_command->get_byte(wifi_command->periph);
				wifi_response_parser(curr_byte);
				if(camera_parser_status == GOT_result){
					result_read = true;
				}
				if(result_read){
					if(result_counter == 4){
						if(curr_byte == '0'){
							settings_set = true;
						}
						else{
							settings_set = false;
						}
					}
					result_counter++;
				}
				if(camera_parser_status == GOT_CLOSE){
					delay_mode = true;
					delay_counter = 0;
				}
			}
			if(delay_mode){

				if(delay_counter < 20){
					delay_counter++;
				}
				else{
					if(cam_model == SONY_QX1){
						tcp_connected = false;
						last_set_setting = 0;
						camera_order = CAMERA_IDLE;
						sony_a7r_state = CAM_IDLE_MODE;
						delay_mode = false;
						uint8_t one = 1;
						DOWNLINK_SEND_CAMERA_SETTINGS(DefaultChannel, DefaultDevice,
															&ssvalue,
															&sival,
															&sfval,
															&last_set_setting,
															&one);
					}
					if(cam_model != SONY_QX1){
						last_set_setting = 3;
						if(settings_set){
							sony_a7r_state = STARTING_TCP_CONNECTION;
						}
						else{
							sony_a7r_state = CAM_IDLE_MODE;
							camera_order = CAMERA_IDLE;
							uint8_t zero = 0;
							DOWNLINK_SEND_CAMERA_SETTINGS(DefaultChannel, DefaultDevice,
															&ssvalue,
															&sival,
															&sfval,
															&last_set_setting,
															&zero);
						}
						delay_mode = false;
					}
				}

			}
		break;
		case SENDING_REQUEST_SF:
			esp_01_cipsend_setting_sf();
			sony_a7r_state = WAIT_FOR_SF_REQUEST_TO_BE_SENT;
			delay_counter = 0;
			delay_mode = false;
		break;
		case WAIT_FOR_SF_REQUEST_TO_BE_SENT:
			while(wifi_command->char_available(wifi_command->periph)){
				wifi_response_parser(wifi_command->get_byte(wifi_command->periph));
				if(camera_parser_status == GOT_OK){
					delay_mode = true;
				}
				if(camera_parser_status == GOT_ERROR){
					sony_a7r_state = SENDING_REQUEST_SF;
				}
			}
			if(delay_mode){

				if(delay_counter < 20){
					delay_counter++;
				}
				else{
					sony_a7r_state = SETTING_FNUMBER;
					delay_mode = false;
				}
			}
		break;
		case SETTING_FNUMBER:
			set_fnumber(sfval);
			sony_a7r_state = WAIT_FOR_FNUMBER_TO_BE_SET;
			result_counter = 0;
			result_read = false;
		break;
		case WAIT_FOR_FNUMBER_TO_BE_SET:
			while(wifi_command->char_available(wifi_command->periph)){
				curr_byte = wifi_command->get_byte(wifi_command->periph);
				wifi_response_parser(curr_byte);
				if(camera_parser_status == GOT_result){
					result_read = true;
				}
				if(result_read){
					if(result_counter == 4){
						if(curr_byte == '0'){
							settings_set = true;
						}
						else{
							settings_set = false;
						}
					}
					result_counter++;
				}
				if(camera_parser_status == GOT_CLOSE){
					delay_mode = true;
					delay_counter = 0;
				}
			}
			if(delay_mode){

				if(delay_counter < 20){
					delay_counter++;
				}
				else{
					if(settings_set){
						tcp_connected = false;
						last_set_setting = 0;
						camera_order = CAMERA_IDLE;
						sony_a7r_state = CAM_IDLE_MODE;
						delay_mode = false;
						uint8_t one = 1;
						DOWNLINK_SEND_CAMERA_SETTINGS(DefaultChannel, DefaultDevice,
															&ssvalue,
															&sival,
															&sfval,
															&last_set_setting,
															&one);
					}
					else{
						tcp_connected = false;
						camera_order = CAMERA_IDLE;
						sony_a7r_state = CAM_IDLE_MODE;
						delay_mode = false;
						uint8_t zero = 0;
						DOWNLINK_SEND_CAMERA_SETTINGS(DefaultChannel, DefaultDevice,
															&ssvalue,
															&sival,
															&sfval,
															&last_set_setting,
															&zero);
					}
				}

			}
		break;


		case WAIT_FOR_REQUESTING_SEND_SHOOT_TCP:
			while(wifi_command->char_available(wifi_command->periph)){
				wifi_response_parser(wifi_command->get_byte(wifi_command->periph));
				if(camera_parser_status == GOT_OK){
					sony_a7r_state = WAIT_FOR_SENDING_SHOOT_COMMAND_MSG_MSG;
				}
				if(camera_parser_status == GOT_ERROR){
					sony_a7r_state = CAM_IDLE_MODE;
				}
			}
		break;
		case WAIT_FOR_SENDING_SHOOT_COMMAND_MSG_MSG:
			sony_a7r_shoot();
			sony_a7r_state = WAIT_FOR_SENDING_SHOOT_COMMAND_MSG;
			result_read = false;
			clear_image_name();
			time_counter = 0;
		break;
		case WAIT_FOR_SENDING_SHOOT_COMMAND_MSG:
			if(time_counter > (MODULES_FREQUENCY * 3)){ // module frequency * 3 seconds
				tcp_connected = false;
				camera_order = CAMERA_IDLE;
				sony_a7r_state = CAM_IDLE_MODE;
			}
			while(wifi_command->char_available(wifi_command->periph)){
				curr_byte = wifi_command->get_byte(wifi_command->periph);
				wifi_response_parser(curr_byte);
				if(camera_parser_status == GOT_result){
					result_read = true;
					image_name_finished = false;
					image_name_started = false;
				}
				if(result_read){
					if(cam_model != SONY_QX1){
						if(camera_parser_status == GOT_slashpict){
							image_name[0] = 'p';
							image_name[1] = 'i';
							image_name[2] = 'c';
							image_name[3] = 't';

							name_counter = 4;
							image_name_started = true;
							continue;
						}
					}
					if(cam_model == SONY_QX1){
						if(camera_parser_status == GOT_DSC){
							image_name[0] = 'D';
							image_name[1] = 'S';
							image_name[2] = 'C';
						
							name_counter = 3;
							image_name_started = true;
							continue;
						}
					}
					if(image_name_started){
					
						if(camera_parser_status != GOT_JPG){
							if(!image_name_finished){
								image_name[name_counter] = curr_byte;
								name_counter++;
							}
						}
						else{
							image_name[name_counter] = curr_byte;
							for(int i=0;i<100;i++){
								final_name[i] = image_name[i];
							}
							image_name_finished = true;
							tag_image_log();
							delay_mode = true;
							delay_counter = 0;
						}
					}

				}
				if(camera_parser_status == GOT_FAIL){
					tcp_connected = false;
					camera_order = CAMERA_IDLE;
					sony_a7r_state = CAM_IDLE_MODE;
				}
			}
			if(delay_mode){
				if(delay_counter < 20){
					delay_counter++;
				}
				else{
					tcp_connected = false;
					camera_order = CAMERA_IDLE;
					sony_a7r_state = CAM_IDLE_MODE;
					delay_mode = false;
				}
			}

			time_counter++;
		break;
	}
}
}

void wifi_response_parser(char curr_byte){

	//processing ok response

	lable:

	if(!word_processing && curr_byte == 'O'){
		camera_parser_status = GOT_O;
		word_processing = true;
		return;
	}
	if(camera_parser_status == GOT_O && curr_byte == 'K'){
		camera_parser_status = GOT_OK;
		word_processing = false;
		return;
	}
	else if(camera_parser_status == GOT_O && curr_byte != 'K'){
		camera_parser_status = IDLE;
		word_processing = false;
		goto lable;
	}

	//processing error response

	if(!word_processing && curr_byte == 'E'){
		camera_parser_status = GOT_E;
		word_processing = true;
		return;
	}
	if(camera_parser_status == GOT_E && curr_byte == 'R'){
		camera_parser_status = GOT_ER;
		return;
	}
	else if(camera_parser_status == GOT_E && curr_byte != 'R'){
		camera_parser_status = IDLE;
		word_processing = false;
		goto lable;
	}
	if(camera_parser_status == GOT_ER && curr_byte == 'R'){
		camera_parser_status = GOT_ERR;
		return;
	}
	else if(camera_parser_status == GOT_ER && curr_byte != 'R'){
		camera_parser_status = IDLE;
		word_processing = false;
		goto lable;
	}
	if(camera_parser_status == GOT_ERR && curr_byte == 'O'){
		camera_parser_status = GOT_ERRO;
		return;
	}
	else if(camera_parser_status == GOT_ERR && curr_byte != 'O'){
		camera_parser_status = IDLE;
		word_processing = false;
		goto lable;
	}
	if(camera_parser_status == GOT_ERRO && curr_byte == 'R'){
		camera_parser_status = GOT_ERROR;
		word_processing = false;
		return;
	}
	else if(camera_parser_status == GOT_ERRO && curr_byte != 'R'){
		camera_parser_status = IDLE;
		word_processing = false;
		goto lable;
	}

	if(!word_processing && curr_byte == 'A'){
		camera_parser_status = GOT_A;
		word_processing = true;
		return;
	}
	if(camera_parser_status == GOT_A && curr_byte == 'L'){
		camera_parser_status = GOT_AL;
		return;
	}
	else if(camera_parser_status == GOT_A && curr_byte != 'L'){
		camera_parser_status = IDLE;
		word_processing = false;
		goto lable;
	}
	if(camera_parser_status == GOT_AL && curr_byte == 'R'){
		camera_parser_status = GOT_ALR;
		return;
	}
	else if(camera_parser_status == GOT_AL && curr_byte != 'R'){
		camera_parser_status = IDLE;
		word_processing = false;
		goto lable;
	}
	if(camera_parser_status == GOT_ALR && curr_byte == 'E'){
		camera_parser_status = GOT_ALRE;
		return;
	}
	else if(camera_parser_status == GOT_ALR && curr_byte != 'E'){
		camera_parser_status = IDLE;
		word_processing = false;
		goto lable;
	}
	if(camera_parser_status == GOT_ALRE && curr_byte == 'A'){
		camera_parser_status = GOT_ALREA;
		return;
	}
	else if(camera_parser_status == GOT_ALRE && curr_byte != 'A'){
		camera_parser_status = IDLE;
		word_processing = false;
		goto lable;
	}
	if(camera_parser_status == GOT_ALREA && curr_byte == 'D'){
		camera_parser_status = GOT_ALREAD;
		return;
	}
	else if(camera_parser_status == GOT_ALREA && curr_byte != 'D'){
		camera_parser_status = IDLE;
		word_processing = false;
		goto lable;
	}
	if(camera_parser_status == GOT_ALREAD && curr_byte == 'Y'){
		camera_parser_status = GOT_ALREADY;
		word_processing = false;
		return;
	}
	else if(camera_parser_status == GOT_ALREAD && curr_byte != 'Y'){
		camera_parser_status = IDLE;
		word_processing = false;
		goto lable;
	}

	//processing result response

	if(!word_processing && curr_byte == 'r'){
		camera_parser_status = GOT_r;
		word_processing = true;
		return;
	}
	if(camera_parser_status == GOT_r && curr_byte == 'e'){
		camera_parser_status = GOT_re;
		return;
	}
	else if(camera_parser_status == GOT_r && curr_byte != 'e'){
		camera_parser_status = IDLE;
		word_processing = false;
		goto lable;
	}
	if(camera_parser_status == GOT_re && curr_byte == 's'){
		camera_parser_status = GOT_res;
		return;
	}
	else if(camera_parser_status == GOT_re && curr_byte != 's'){
		camera_parser_status = IDLE;
		word_processing = false;
		goto lable;
	}
	if(camera_parser_status == GOT_res && curr_byte == 'u'){
		camera_parser_status = GOT_resu;
		return;
	}
	else if(camera_parser_status == GOT_res && curr_byte != 'u'){
		camera_parser_status = IDLE;
		word_processing = false;
		goto lable;
	}
	if(camera_parser_status == GOT_resu && curr_byte == 'l'){
		camera_parser_status = GOT_resul;
		return;
	}
	else if(camera_parser_status == GOT_resu && curr_byte != 'l'){
		camera_parser_status = IDLE;
		word_processing = false;
		goto lable;
	}
	if(camera_parser_status == GOT_resul && curr_byte == 't'){
		camera_parser_status = GOT_result;
		word_processing = false;
		return;
	}
	else if(camera_parser_status == GOT_resul && curr_byte != 't'){
		camera_parser_status = IDLE;
		word_processing = false;
		goto lable;
	}

	//achieving image name

	if(!word_processing && curr_byte == '/'){
		camera_parser_status = GOT_slash;
		word_processing = true;
		return;
	}
	if(camera_parser_status == GOT_slash && curr_byte == 'p'){
		camera_parser_status = GOT_slashp;
		return;
	}
	else if(camera_parser_status == GOT_slash && curr_byte != 'p'){
		camera_parser_status = IDLE;
		word_processing = false;
		goto lable;
	}
	if(camera_parser_status == GOT_slashp && curr_byte == 'i'){
		camera_parser_status = GOT_slashpi;
		return;
	}
	else if(camera_parser_status == GOT_slashp && curr_byte != 'i'){
		camera_parser_status = IDLE;
		word_processing = false;
		goto lable;
	}
	if(camera_parser_status == GOT_slashpi && curr_byte == 'c'){
		camera_parser_status = GOT_slashpic;
		return;
	}
	else if(camera_parser_status == GOT_slashpi && curr_byte != 'c'){
		camera_parser_status = IDLE;
		word_processing = false;
		goto lable;
	}
	if(camera_parser_status == GOT_slashpic && curr_byte == 't'){
		camera_parser_status = GOT_slashpict;
		word_processing = false;
		return;
	}
	else if(camera_parser_status == GOT_slashpic && curr_byte != 't'){
		camera_parser_status = IDLE;
		word_processing = false;
		goto lable;
	}

	//processing close connection status

	if(!word_processing && curr_byte == 'C'){
		camera_parser_status = GOT_C;
		word_processing = true;
		return;
	}
	if(camera_parser_status == GOT_C && curr_byte == 'L'){
		camera_parser_status = GOT_CL;
		return;
	}
	else if(camera_parser_status == GOT_C && curr_byte != 'L'){
		camera_parser_status = IDLE;
		word_processing = false;
		goto lable;
	}
	if(camera_parser_status == GOT_CL && curr_byte == 'O'){
		camera_parser_status = GOT_CLO;
		return;
	}
	else if(camera_parser_status == GOT_CL && curr_byte != 'O'){
		camera_parser_status = IDLE;
		word_processing = false;
		goto lable;
	}
	if(camera_parser_status == GOT_CLO && curr_byte == 'S'){
		camera_parser_status = GOT_CLOS;
		return;
	}
	else if(camera_parser_status == GOT_CLO && curr_byte != 'S'){
		camera_parser_status = IDLE;
		word_processing = false;
		goto lable;
	}
	if(camera_parser_status == GOT_CLOS && curr_byte == 'E'){
		camera_parser_status = GOT_CLOSE;
		word_processing = false;
		return;
	}
	else if(camera_parser_status == GOT_CLOS && curr_byte != 'E'){
		camera_parser_status = IDLE;
		word_processing = false;
		goto lable;
	}

	//processing end of image name

	if(!word_processing && curr_byte == 'D'){
		camera_parser_status = GOT_D;
		word_processing = true;
		return;
	}
	if(camera_parser_status == GOT_D && curr_byte == 'S'){
		camera_parser_status = GOT_DS;
		return;
	}
	else if(camera_parser_status == GOT_D && curr_byte != 'S'){
		camera_parser_status = IDLE;
		word_processing = false;
		goto lable;
	}
	if(camera_parser_status == GOT_DS && curr_byte == 'C'){
		camera_parser_status = GOT_DSC;
		word_processing = false;
		return;
	}
	else if(camera_parser_status == GOT_DS && curr_byte != 'C'){
		camera_parser_status = IDLE;
		word_processing = false;
		goto lable;
	}

	if(!word_processing && curr_byte == 'J'){
		camera_parser_status = GOT_J;
		word_processing = true;
		return;
	}
	if(camera_parser_status == GOT_J && curr_byte == 'P'){
		camera_parser_status = GOT_JP;
		return;
	}
	else if(camera_parser_status == GOT_J && curr_byte != 'P'){
		camera_parser_status = IDLE;
		word_processing = false;
		goto lable;
	}
	if(camera_parser_status == GOT_JP && curr_byte == 'G'){
		camera_parser_status = GOT_JPG;
		word_processing = false;
		return;
	}
	else if(camera_parser_status == GOT_JP && curr_byte != 'G'){
		camera_parser_status = IDLE;
		word_processing = false;
		goto lable;
	}

	//processing SEND FAIL response

	if(!word_processing && curr_byte == 'F'){
		camera_parser_status = GOT_F;
		word_processing = true;
		return;
	}
	if(camera_parser_status == GOT_F && curr_byte == 'A'){
		camera_parser_status = GOT_FA;
		return;
	}
	else if(camera_parser_status == GOT_F && curr_byte != 'A'){
		camera_parser_status = IDLE;
		word_processing = false;
		goto lable;
	}
	if(camera_parser_status == GOT_FA && curr_byte == 'I'){
		camera_parser_status = GOT_FAI;
		return;
	}
	else if(camera_parser_status == GOT_FA && curr_byte != 'I'){
		camera_parser_status = IDLE;
		word_processing = false;
		goto lable;
	}
	if(camera_parser_status == GOT_FAI && curr_byte == 'L'){
		camera_parser_status = GOT_FAIL;
		word_processing = false;
		return;
	}
	else if(camera_parser_status == GOT_FAI && curr_byte != 'L'){
		camera_parser_status = IDLE;
		word_processing = false;
		goto lable;
	}

	if(!word_processing && curr_byte == '4'){
		camera_parser_status = GOT_4;
		return;
	}

	if(!word_processing && curr_byte == '3'){
		camera_parser_status = GOT_3;
		return;
	}

	camera_parser_status = IDLE;
}

void clear_image_name(void){
	for(int i=0; i<100; i++){
		image_name[i] = '&';
	}
}

void shoot(void){
	camera_order = CAMERA_SHOOT;
}

void send_settings(void){
	camera_order = SET_CAMERA_PARAMETERS;
}
