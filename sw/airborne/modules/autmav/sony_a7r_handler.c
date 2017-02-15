#include "sony_a7r_handler.h"
#include "serial_port.h"

struct SerialPort *serial_port;
uint32_t serial_baud = B115200;
char *serial_device = "/dev/ttyUSB0";

void sony_a7r_handler_setup(){
	int ret = serial_port_open_raw(serial_port, serial_device, serial_baud);
	if (ret != 0) {
    	serial_port_free(serial_port);
    	exit(EXIT_FAILURE);
	}
}