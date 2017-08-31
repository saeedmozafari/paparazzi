
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
#include "subsystems/abi.h"

struct link_device *sonar_dev;

#ifndef SONAR_PORT
#define SONAR_PORT uart6
#endif

#ifndef SONAR_BAUD
#define SONAR_BAUD B9600
#endif


float range;
uint8_t write_idx=0;

union uint162bytes u2b;

uint8_t buffer[10];

static void send_sonar_altitude(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_SONAR_ALTITUDE(trans, dev, AC_ID,
                                          &range);
}

void sonar_init(void)
{
 register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_SONAR_ALTITUDE, send_sonar_altitude);

  sonar_dev = &((SONAR_PORT).device);
  uart_periph_set_bits_stop_parity(&SONAR_PORT, UBITS_8, USTOP_1, UPARITY_NO);
  uart_periph_set_baudrate(&SONAR_PORT, SONAR_BAUD);

  //register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_COMPUTER_STATUS, send_computer_state);
}

void sonar_get_periodic(void)
{

  char response = 0;
  if (sonar_dev->char_available(sonar_dev->periph)) {
    //range = 0;
    response = (char) sonar_dev->get_byte(sonar_dev->periph);
    if(response==219){
    	if(write_idx==1){
    			 range=buffer[0]/100.0;
    			AbiSendMsgAGL(AGL_SONAR_ADC_ID, range);
    	}
    	write_idx=0;
    }else{
    	buffer[write_idx]=response;
    	write_idx++;
    }

    if(write_idx==2){
    	u2b.bytes[1]=buffer[0];
    	u2b.bytes[0]=buffer[1];
    	    range=u2b.value/100.0;
    AbiSendMsgAGL(AGL_SONAR_ADC_ID, range);
    }

}
}


