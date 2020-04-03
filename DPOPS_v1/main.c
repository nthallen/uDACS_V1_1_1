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
#include "subbus.h"
#include "SPI_PR_SN.h"
#include "Timer_Setup.h"

int main(void) {
	// initialize the micro and its peripherals the start the System Timer
	atmel_start_init();
	TIMER_0_go();
	
	// Add in all needed drives and test for success
	if (subbus_add_driver(&sb_base)    ||
		subbus_add_driver(&sb_fail_sw) ||				
															// subbus_add_driver(&sb_spi)     ||
		subbus_add_driver(&sb_ps_spi)) { while(true); }		// if True => some driver is mis configured.
	
	// reset all resettable drivers and spin forever
	subbus_reset();											//ps_spi_reset();
	while (1) {
		gpio_set_pin_level(PMP_CNTL_1, true);				// Scope Debug - pulse every state clock
		for(uint8_t ii=0; ii<3; ii++) {gpio_set_pin_level(PMP_CNTL_1, true);}
		gpio_set_pin_level(PMP_CNTL_1, false);
		
		subbus_poll();										//ps_spi_poll();
	}
}
