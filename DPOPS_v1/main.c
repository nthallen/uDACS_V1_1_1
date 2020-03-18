#include <atmel_start.h>
#include "SPI_PR_SN.h"

int main(void) {
	atmel_start_init();
	ps_spi_reset();
		
	while (1) {
		gpio_set_pin_level(PMP_CNTL_1, true);           // pulse every state clock
		gpio_set_pin_level(PMP_CNTL_1, false);
		ps_spi_poll();
	}
}
