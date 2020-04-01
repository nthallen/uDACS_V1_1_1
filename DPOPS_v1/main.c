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

/* *******************************************************************************************************
 * Use RTC Hardware resource to generate a 32 bit counter that increments once 
 * every 1000 uSec (once every 8000 CPU clocks)
 * ~ 4 usec of overhead associated with ISR so set ticks to 996
 *
 */

static struct timer_task TIMER_0_task1;
static uint32_t count_1msec = 0;

static void TIMER_0_task1_cb(const struct timer_task *const timer_task) {
	count_1msec++;
	gpio_toggle_pin_level(PA18);
}

void TIMER_0_go(void) {
	uint32_t clock_cycles = 7996;                              // # of Timer Clocks (effected by Prescaler)
	timer_set_clock_cycles_per_tick(&TIMER_0, clock_cycles);   // between each interrupt.
	TIMER_0_task1.interval = 1;                  // Number of Timer Interrupts prior to calling callback task
	TIMER_0_task1.cb       = TIMER_0_task1_cb;   // the task to call on task.interval-th interrupt
	TIMER_0_task1.mode     = TIMER_TASK_REPEAT;  // Re-queue or do it only once

	timer_add_task(&TIMER_0, &TIMER_0_task1);
	timer_start(&TIMER_0);
}

int main(void) {
	atmel_start_init();
	ps_spi_reset();
	TIMER_0_go();
	while (1) {
		gpio_set_pin_level(PMP_CNTL_1, true);           // pulse every state clock
		for(uint8_t ii=0; ii<3; ii++) {gpio_set_pin_level(PMP_CNTL_1, true);}
		gpio_set_pin_level(PMP_CNTL_1, false);
		ps_spi_poll();
	}
}
