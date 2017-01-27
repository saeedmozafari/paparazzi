#define RTK_RECEIVE_C

#include "RTK_receive.h"

struct link_device *RTK_receive_device;

void RTK_receive_init(void){
    
    RTK_receive_device = &((RTK_RTCM3_UART_DEV).device);
	uart_periph_set_bits_stop_parity(&RTK_RTCM3_UART_DEV, UBITS_8, USTOP_1, UPARITY_NO);
	uart_periph_set_baudrate(&RTK_RTCM3_UART_DEV, RTK_RTCM3_UART_BAUD);

}
void relay_msg(uint8_t length, uint8_t *relay_data){

	int i;
	for (i = 0; i < length; i++) {
    	RTK_receive_device->put_byte(RTK_receive_device->periph, 0, relay_data[i]);
  	}
}