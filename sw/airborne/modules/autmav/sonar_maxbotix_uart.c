
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
#include "sonar_maxbotix_uart.h"

struct link_device *sonar_dev;

#ifndef SONAR_PORT
#define SONAR_PORT uart5
#endif

#ifndef SONAR_BAUD
#define SONAR_BAUD B9600
#endif
uint16_t update_freq=0;
uint8_t flag2=0;
uint8_t flag=0;
uint16_t frq_count = 0;
int32_t init_time = 0;
uint8_t final_time = 0;
char sonbuf2[10];
uint8_t sonbuf2_len=0;
float range;
static void send_sonar_altitude(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_SONAR_ALTITUDE(trans, dev, AC_ID,
                                          &range, &update_freq, &frq_count, &final_time);
}

void sonar_init(void)
{
	range=0;
 register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_SONAR_ALTITUDE, send_sonar_altitude);
  sonar_dev = &((SONAR_PORT).device);
  uart_periph_set_bits_stop_parity(&SONAR_PORT, UBITS_8, USTOP_1, UPARITY_NO);
  uart_periph_set_baudrate(&SONAR_PORT, SONAR_BAUD);

  //register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_COMPUTER_STATUS, send_computer_state);
}

void sonar_get_periodic(void)
{
//range=122;

   int32_t sum = 0;

  // int16_t nbytes = uart->available();
   uint16_t count = 0;
  while (sonar_dev->char_available(sonar_dev->periph)) {
    //range++;
    //range = 0;
    char c = (char) sonar_dev->get_byte(sonar_dev->periph);

    if (c == 'R') {
            sonbuf2[sonbuf2_len] = 0;
            sum += (int)atoi(sonbuf2);
            count++;
            sonbuf2_len = 0;
        } else if (isdigit(c)) {
            sonbuf2[sonbuf2_len++] = c;
            if (sonbuf2_len == sizeof(sonbuf2)) {
                // too long, discard the line
                sonbuf2_len = 0;
            }
        }
}

        range = (2.54f * sum / count)/100;
        if(flag==0 && get_sys_time_usec()>=20000000){
          init_time=get_sys_time_usec();
           flag=1;
           frq_count=0;
        }


        frq_count++;

        if((get_sys_time_usec()-init_time)>=20000000 && flag2==0){
        	final_time=((get_sys_time_usec()-init_time)/1000000);
        	update_freq=frq_count/final_time;
        	flag2=1;
        }
    //range=response;
    /*if(response=='R'){
      range++;
      rbit=1;
      write_idx=1;
    }*/
    


      AbiSendMsgAGL(AGL_SONAR_ADC_ID, range);
    
  }
    



