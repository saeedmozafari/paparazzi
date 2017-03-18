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

struct link_device *wifi_command;

#ifndef ESP_01_BAUD
#define ESP_01_BAUD B115200
#endif

#ifndef ESP_01_UART_PORT 
#define ESP_01_UART_PORT uart6
#endif

int result_counter = 0;
int name_counter = 0;
int delay_counter = 0;
int tcp_connection_errors = 0;
int udp_connection_errors = 0;
int time_counter = 0;
bool result_read = false;
bool mode_set = false;
bool tcp_connected = false;
bool word_processing = false;
bool already_connected = false;
bool delay_mode = false;
bool chars_recieved = false;
bool image_name_started = false;
bool image_name_finished = false;
char image_name[100];
char final_name[100];
char ready_test[4] = "AT\r\n";
float curr_time = 0.0;
float start_time = 0.0;
FILE *image_names;

enum cam_state sony_a7r_state;
enum parser_status camera_parser_status;
enum cam_order camera_order;

static void send_camera_state(struct transport_tx *trans, struct link_device *dev)
 {
   	uint8_t cam_state_for_debug = sony_a7r_state;
   	uint8_t parser_state_for_debug = camera_parser_status;
	pprz_msg_send_SONY_CAMERA_STATUS(trans, dev, AC_ID,
                         &cam_state_for_debug,
                         &tcp_connection_errors);
}

void sony_a7r_handler_setup(void){
	wifi_command = &((ESP_01_UART_PORT).device);
	uart_periph_set_bits_stop_parity(&ESP_01_UART_PORT, UBITS_8, USTOP_1, UPARITY_NO);
	uart_periph_set_baudrate(&ESP_01_UART_PORT, ESP_01_BAUD);
	mode_set = false;
	register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_SONY_CAMERA_STATUS, send_camera_state);
	clear_image_name();
}

void esp_01_jap(void){
	LED_ON(3);
	//char jap_msg[43] = "AT+CWJAP=\"DIRECT-PXE0:ILCE-7R\",\"wLmAKDZS\"\r\n";
	char jap_msg[45] = "AT+CWJAP=\"DIRECT-oXE0:ILCE-6000\",\"a5u7LVJY\"\r\n";
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

void endline(void){
	char shoot_command_msg[2] = "\r\n";
	for(int i=0; i<2; i++){
		wifi_command->put_byte(wifi_command->periph, 0, (uint8_t)shoot_command_msg[i]);
	}
}

void sony_a7r_handler_periodic(void){
	char curr_byte;

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
					sony_a7r_state = CONNECTING_TO_CAM_AP;
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
					sony_a7r_state = STARTING_UDP_CONNECTION;
				}
				if(camera_parser_status == GOT_FAIL || camera_parser_status == GOT_ERROR){
					sony_a7r_state = SETTING_CIPMUX;
				}
			}
		break;
		case STARTING_UDP_CONNECTION:
			esp_01_cipstart();
			sony_a7r_state = WAIT_FOR_STARTING_UDP_CONNECTION;
			already_connected = false;
		break;
		case WAIT_FOR_STARTING_UDP_CONNECTION:
			while(wifi_command->char_available(wifi_command->periph)){
				wifi_response_parser(wifi_command->get_byte(wifi_command->periph));
				if(camera_parser_status == GOT_OK){
					sony_a7r_state = REQUESTING_SEND;
				}
				if(camera_parser_status == GOT_ALREADY){
					already_connected = true;
					sony_a7r_state = REQUESTING_SEND;
				}
				if(camera_parser_status == GOT_ERROR && !already_connected){
					if(udp_connection_errors < 3){
						sony_a7r_state = STARTING_UDP_CONNECTION;
						udp_connection_errors++;
					}
					else{
						udp_connection_errors = 0;
						sony_a7r_state = CONNECTING_TO_CAM_AP;
					}
				}
			}
		break;
		case REQUESTING_SEND:
			esp_01_cipsend();
			sony_a7r_state = WAIT_FOR_REQUESTING_SEND;
		break;
		case WAIT_FOR_REQUESTING_SEND:
			while(wifi_command->char_available(wifi_command->periph)){
				wifi_response_parser(wifi_command->get_byte(wifi_command->periph));
				if(camera_parser_status == GOT_OK){
					sony_a7r_state = SENDING_DISCOVERY_MSG;
				}
				if(camera_parser_status == GOT_ERROR){
					sony_a7r_state = REQUESTING_SEND;
				}
			}
		break;
		case SENDING_DISCOVERY_MSG:
			esp_01_msearch();
			sony_a7r_state = WAIT_FOR_SENDING_DISCOVERY_MSG;
		break;
		case WAIT_FOR_SENDING_DISCOVERY_MSG:
			while(wifi_command->char_available(wifi_command->periph)){
				wifi_response_parser(wifi_command->get_byte(wifi_command->periph));
				if(camera_parser_status == GOT_OK){
					sony_a7r_state = CLOSING_UDP_CONNECTION;
				}
				if(camera_parser_status == GOT_FAIL || camera_parser_status == GOT_ERROR){
					sony_a7r_state = REQUESTING_SEND;
				}
			}
		break;
		case CLOSING_UDP_CONNECTION:
			esp_01_cipclose();
			sony_a7r_state = WAIT_FOR_CLOSING_UDP_CONNECTION;
		break;
		case WAIT_FOR_CLOSING_UDP_CONNECTION:
			while(wifi_command->char_available(wifi_command->periph)){
				wifi_response_parser(wifi_command->get_byte(wifi_command->periph));
				if(camera_parser_status == GOT_OK){
					sony_a7r_state = STARTING_TCP_CONNECTION;
				}
				if(camera_parser_status == GOT_ERROR){
					sony_a7r_state = CLOSING_UDP_CONNECTION;
				}
			}
		break;
		case STARTING_TCP_CONNECTION:
			esp_01_cipstart_tcp();
			sony_a7r_state = WAIT_FOR_STARTING_TCP_CONNECTION;
			already_connected = false;
		break;
		case WAIT_FOR_STARTING_TCP_CONNECTION:
			while(wifi_command->char_available(wifi_command->periph)){
				wifi_response_parser(wifi_command->get_byte(wifi_command->periph));
				if(camera_parser_status == GOT_OK){
					sony_a7r_state = REQUESTING_SEND_TCP;
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
		break;
		case REQUESTING_SEND_TCP:
			if(!mode_set){
				esp_01_cipsend_tcp();
				sony_a7r_state = WAIT_FOR_REQUESTING_SEND_TCP;
			}
			else{
				sony_a7r_state = CAM_IDLE_MODE;
				tcp_connected = true;
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
							delay_mode = true;
							delay_counter = 0;
							
						}
						else{
							sony_a7r_state = STARTING_TCP_CONNECTION;
						}
					}
					result_counter++;
				}
				if(camera_parser_status == GOT_FAIL){
					sony_a7r_state = STARTING_TCP_CONNECTION;
				}
			}
			if(delay_mode){
				if(delay_counter < 3){
					delay_counter++;
				}
				else{
					sony_a7r_state = MODE_MSG_SENT;
					delay_mode = false;
				}
			}
		break;
		case MODE_MSG_SENT:
			sony_a7r_state = CAM_IDLE_MODE;
			mode_set = true;
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
		break;
		case WAIT_FOR_SENDING_SHOOT_COMMAND_MSG:
			while(wifi_command->char_available(wifi_command->periph)){
				curr_byte = wifi_command->get_byte(wifi_command->periph);
				wifi_response_parser(curr_byte);
				if(camera_parser_status == GOT_result){
					result_read = true;
					name_counter = 4;
					image_name_finished = false;
					image_name_started = false;
				}
				if(result_read){
					if(camera_parser_status == GOT_slashpict){
						image_name[0] = 'p';
						image_name[1] = 'i';
						image_name[2] = 'c';
						image_name[3] = 't';

						image_name_started = true;
						continue;
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
				if(delay_counter < 2){
					delay_counter++;
				}
				else{
					tcp_connected = false;
					camera_order = CAMERA_IDLE;
					sony_a7r_state = CAM_IDLE_MODE;
					delay_mode = false;
				}
			}
		break;
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

void idle(void){

}