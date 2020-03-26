/* *****************************************************************************
 * DCOTSS DPOPS uDACs RevB Duct Sensors / Pump control monitor code
 *
 *    Based on Nort Embedded Architecture - Real-time asyn/non-blocking
 *    callback approach with cached host interface.
 *
 *    Main While(1) 
 *		poll host, 
 *		poll peripherals
 *
 *      state-clock period = 100us (defines "real-time")
 *      no state anywhere can block for longer than this period.
 * 
 * Revision 1	03/27/2020  Michael Litchfield
 *
 */
#include <atmel_start.h>
#include "SPI_PR_SN.h"

int main(void) {
	atmel_start_init();
	ps_spi_reset();
		
	while (1) {
		gpio_set_pin_level(PMP_CNTL_1, true);           // pulse every state clock
		for(uint8_t ii=0; ii<3; ii++) {gpio_set_pin_level(PMP_CNTL_1, true);}
		gpio_set_pin_level(PMP_CNTL_1, false);
		ps_spi_poll();
	}
}
