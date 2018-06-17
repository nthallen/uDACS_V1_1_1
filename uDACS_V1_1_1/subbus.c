/* subbus.c for Atmel Studio
 */
#include "subbus.h"

static subbus_driver_t *drivers[SUBBUS_MAX_DRIVERS];
static int n_drivers = 0;

/** @return true on error.
 * Possible errors include too many drivers or drivers not in ascending order.
 */
bool subbus_add_driver(subbus_driver_t *driver) {
  if ((n_drivers >= SUBBUS_MAX_DRIVERS) ||
      ((n_drivers > 0) && (drivers[n_drivers-1]->high > driver->low)) ||
      driver->high < driver->low || driver->read == 0 || driver->write == 0)
    return true;
  drivers[n_drivers++] = driver;
  return false;
}

void subbus_reset(void) {
  int i;
  for (i = 0; i < n_drivers; ++i) {
    if (drivers[i]->reset) {
      (*(drivers[i]->reset))();
    }
  }
}

void subbus_poll(void) {
  int i;
  for (i = 0; i < n_drivers; ++i) {
    if (drivers[i]->poll) {
      (*drivers[i]->poll)();
    }
  }
}

/**
 * @return non-zero on success (acknowledge)
 */
int subbus_read( uint16_t addr, uint16_t *rv ) {
  int i;
  for (i = 0; i < n_drivers; ++i) {
    if (addr < drivers[i]->low) return 0;
    if (addr <= drivers[i]->high) return (*drivers[i]->read)(addr, rv);
  }
  *rv = 0;
  return 0;
}

/**
 * @return non-zero on success (acknowledge)
 */
int subbus_write( uint16_t addr, uint16_t data) {
  int i;
  for (i = 0; i < n_drivers; ++i) {
    if (addr < drivers[i]->low) return 0;
    if (addr <= drivers[i]->high) return (*drivers[i]->write)(addr, data);
  }
  return 0;
}

void set_fail(uint16_t arg) {
#if defined(SUBBUS_FAIL_ADDR)
  subbus_write(SUBBUS_FAIL_ADDR, arg);
#endif
}

#if SUBBUS_INTERRUPTS
extern volatile uint8_t subbus_intr_req;
void init_interrupts(void);
int intr_attach(int id, uint16_t addr);
int intr_detach( uint16_t addr );
void intr_service(void);
#endif


static int sb_base_read(uint16_t addr, uint16_t *rv) {
  switch (addr) {
    case SUBBUS_BDID_ADDR:
      *rv = SUBBUS_BUILD_NUM;
      return 1;
    case SUBBUS_BDID_ADDR+1:
      *rv = SUBBUS_BOARD_ID;
      return 1;
    case 0:
    case SUBBUS_INTA_ADDR:
    default:
      *rv = 0;
      return 0;
  }
}

static int sb_base_write(uint16_t addr, uint16_t data) {
  return 0; // Nothing here is writable
}

subbus_driver_t sb_base = { 0, SUBBUS_BDID_ADDR+1, sb_base_read, sb_base_write, 0, 0 };

static uint16_t fail_reg = 0;

static int sb_fail_sw_read(uint16_t addr, uint16_t *rv) {
  switch(addr) {
    case SUBBUS_FAIL_ADDR:
      *rv = fail_reg;
      return 1;
    case SUBBUS_SWITCHES_ADDR:
      *rv = 0;
      return 1;
    default:
      *rv = 0;
      return 0;
  }
}

static int sb_fail_sw_write(uint16_t addr, uint16_t data) {
  switch(addr) {
    case SUBBUS_FAIL_ADDR:
      fail_reg = data;
      return 1;
    case SUBBUS_SWITCHES_ADDR:
    default:
      return 0; // not writable
  }
}

static void sb_fail_sw_reset() {
  fail_reg = 0;
}

subbus_driver_t sb_fail_sw = { SUBBUS_FAIL_ADDR, SUBBUS_SWITCHES_ADDR,
    sb_fail_sw_read, sb_fail_sw_write, sb_fail_sw_reset};
