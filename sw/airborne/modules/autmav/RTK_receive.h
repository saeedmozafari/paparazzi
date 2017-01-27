#ifndef RTK_RECEIVE_H
#define RTK_RECEIVE_H

#include "subsystems/datalink/datalink.h" // dl_buffer
#include "generated/airframe.h"           // AC_ID

#include "pprzlink/pprzlink_device.h"
#include "mcu_periph/uart.h"

extern void relay_msg(uint8_t length, uint8_t *relay_data);
extern void RTK_receive_init(void);

static inline void parse_DL_RTCM_INJECT(void)
{
	//uint8_t packet_id = DL_RTCM_INJECT_packet_id(dl_buffer);
	uint8_t data_length = DL_RTCM_INJECT_data_length(dl_buffer);
	uint8_t *RTCM3_data = DL_RTCM_INJECT_data(dl_buffer);
	relay_msg(data_length, RTCM3_data);
}

#endif