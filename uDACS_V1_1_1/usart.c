#include <peripheral_clk_config.h>
#include <hpl_gclk_base.h>
#include <hpl_pm_base.h>
#include <hal_usart_async.h>
#include "uDACS_pins.h"
// #include "uDACS_driver_init.h"
#include "usart.h"


static struct usart_async_descriptor USART_CTRL;
/*! Buffer for the receive ringbuffer */
static uint8_t USART_CTRL_rx_buffer[USART_CTRL_RX_BUFFER_SIZE];

/*! Buffer to accumulate output before sending */
static uint8_t USART_CTRL_tx_buffer[USART_CTRL_TX_BUFFER_SIZE];
 /*! The number of characters in the tx buffer */
static int nc_tx;
volatile int USART_CTRL_tx_busy = 0;
static struct io_descriptor *USART_CTRL_io;

#ifdef __cplusplus
extern "C" {
#endif


/**
 * \brief USART Clock initialization function
 *
 * Enables register interface and peripheral clock
 * Copied from driver_init.c: will need manual editing when that changes
 */
void USART_CTRL_CLOCK_init() {
	_pm_enable_bus_clock(PM_BUS_APBC, SERCOM5);
	_gclk_enable_channel(SERCOM5_GCLK_ID_CORE, CONF_GCLK_SERCOM5_CORE_SRC);
}

/**
 * \brief USART pinmux initialization function
 *
 * Set each required pin to USART functionality
 * Copied from driver_init.c: will need manual editing when that changes
 */
void USART_CTRL_PORT_init() {
	gpio_set_pin_function(UART_TX, PINMUX_PB22D_SERCOM5_PAD2);
	gpio_set_pin_function(UART_RX, PINMUX_PB23D_SERCOM5_PAD3);
}


static void tx_cb_USART_CTRL(const struct usart_async_descriptor *const io_descr) {
	/* Transfer completed */
	USART_CTRL_tx_busy = 0;
}

/**
 * \brief Callback for received characters.
 * We do nothing here, but if we don't set it up, the low-level receive character
 * function won't be called either. This is of course undocumented behavior.
 */
static void rx_cb_USART_CTRL(const struct usart_async_descriptor *const io_descr) {}

/**
 * \brief USART initialization function
 *
 * Enables USART peripheral, clocks and initializes USART driver
 */
void USART_CTRL_init(void) {
	USART_CTRL_CLOCK_init();
	usart_async_init(&USART_CTRL, SERCOM5, USART_CTRL_rx_buffer,
                   USART_CTRL_RX_BUFFER_SIZE, (void *)NULL);
	USART_CTRL_PORT_init();
}

static void USART_CTRL_write(const uint8_t *text, int count) {
	while (USART_CTRL_tx_busy) {}
	USART_CTRL_tx_busy = 1;
	io_write(USART_CTRL_io, text, count);
}

void uart_init(void) {
	USART_CTRL_init();
	usart_async_register_callback(&USART_CTRL, USART_ASYNC_TXC_CB, tx_cb_USART_CTRL);
	usart_async_register_callback(&USART_CTRL, USART_ASYNC_RXC_CB, rx_cb_USART_CTRL);
	usart_async_register_callback(&USART_CTRL, USART_ASYNC_ERROR_CB, 0);
	usart_async_get_io_descriptor(&USART_CTRL, &USART_CTRL_io);
	usart_async_enable(&USART_CTRL);
  nc_tx = 0;
}

int uart_recv(uint8_t *buf, int nbytes) {
	return io_read(USART_CTRL_io, (uint8_t *)buf, nbytes);
}

void uart_flush_input(void) {
  usart_async_flush_rx_buffer(&USART_CTRL);
}

void uart_send_char(int8_t c) {
  if (nc_tx >= USART_CTRL_TX_BUFFER_SIZE) {
    /* We can't be flushing, or nc_tx would be zero. Characters cannot be
       added to the buffer until _tx_busy is clear. */
    assert(USART_CTRL_tx_busy == 0,__FILE__,__LINE__);
    uart_flush_output();
  }
  while (USART_CTRL_tx_busy) {}
  assert(nc_tx < USART_CTRL_TX_BUFFER_SIZE,__FILE__,__LINE__);
  USART_CTRL_tx_buffer[nc_tx++] = (uint8_t)c;
}

void uart_flush_output(void) {
  int nc = nc_tx;
  assert(USART_CTRL_tx_busy == 0,__FILE__,__LINE__);
  nc_tx = 0;
  USART_CTRL_write(USART_CTRL_tx_buffer, nc);
}

#ifdef __cplusplus
};
#endif
