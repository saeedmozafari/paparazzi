
#include "pprzlink/pprzlink_device.h"
#include "mcu_periph/uart.h"
#include "subsystems/datalink/datalink.h"
#include "generated/airframe.h"
#include "subsystems/datalink/telemetry.h"
#include "led.h"
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
#define SONAR_PORT uart4
#endif

#ifndef SONAR_BAUD
#define SONAR_BAUD B9600
#endif


float range=0;
uint8_t write_idx=0;
bool rbit;

union uint162bytes u2b;

char buffer[3];

static void send_sonar_altitude(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_SONAR_ALTITUDE(trans, dev, AC_ID,
                                          &range);
}

void sonar_init(void)
{
 register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_SONAR_ALTITUDE, send_sonar_altitude);
rbit=0;
  sonar_dev = &((SONAR_PORT).device);
  uart_periph_set_bits_stop_parity(&SONAR_PORT, UBITS_8, USTOP_1, UPARITY_NO);
  uart_periph_set_baudrate(&SONAR_PORT, SONAR_BAUD);

  //register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_COMPUTER_STATUS, send_computer_state);
}

void sonar_get_periodic(void)
{

  char response = 0;
  while (sonar_dev->char_available(sonar_dev->periph)) {
    //range=222;
    //range = 0;
    response = (char) sonar_dev->get_byte(sonar_dev->periph);
    range=response;
    if(response=='R'){
      //range=2222;
      rbit=1;
      write_idx=1;
    }
    if(rbit==1 && write_idx!=0){
    buffer[write_idx-1]=response;
    write_idx++;
    }
    
    if(write_idx>3){
      //buffer[write_idx]=0x00;
      write_idx=0;
      rbit=0;
      //range=atoi(buffer[2]);
      //range = 5;
     /* u2b.bytes[2]=buffer[3];
      u2b.bytes[1]=buffer[2];
      u2b.bytes[0]=buffer[1];
      range=u2b.value*2.54;*/


      AbiSendMsgAGL(AGL_SONAR_ADC_ID, range);
    }
  }
    


}



