#include "subbus.h"
#include "control.h"

static uint16_t fail_reg = 0;

/************************************************************************/
/* Interrupt Handling                                                   */
/************************************************************************/
#if 0 // SUBBUS_INTERRUPTS (just to make sure)

const board_intr_t subbus_t::bd_intrs[] = {
};

#define N_INTERRUPTS (sizeof(bd_intrs)/sizeof(board_intr_t))

interrupt_t subbus_t::interrupts[N_INTERRUPTS];

volatile uint8_t subbus_t::intr_req = 0;

void subbus_t::init_interrupts(void) {
  int i;
  for ( i = 0; i < N_INTERRUPTS; i++) {
    interrupts[i].active = 0;
  }
}

/**
 * Associates the interrupt at the specified base address with the
 * specified interrupt ID. Does not check to see if the interrupt
 * is already active, since that should be handled in the driver.
 * The only possible error is that the specified address is not
 * in our defined list, so ENOENT.
 * @return non-zero on success. Will return zero if board not listed in table,
 * or no acknowledge detected.
 */
int subbus_t::intr_attach(int id, uint16_t addr) {
  int i;
  for ( i = 0; i < N_INTERRUPTS; i++ ) {
    if ( bd_intrs[i].address == addr ) {
      interrupts[i].active = 1;
      interrupts[i].intr_id = id;
      return write( addr, 0x20 );
    }
  }
  return 0;
}

/**
 * @return non-zero on success. Will return zero if board is not listed in the table
 * or no acknowledge is detected.
 */
int subbus_t::intr_detach( uint16_t addr ) {
  int i;
  for ( i = 0; i < N_INTERRUPTS; i++ ) {
    if ( bd_intrs[i].address == addr ) {
      interrupts[i].active = 0;
      return write(addr, 0);
      return 1;
    }
  }
  return 0;
}

void subbus_t::intr_service(void) {
  uint16_t ivec;
  if ( this->sb_read(SUBBUS_INTA_ADDR, &ivec) ) {
    int i;
    for ( i = 0; i < N_INTERRUPTS; i++ ) {
      if (ivec & bd_intrs[i].bitmask) {
        if (interrupts[i].active) {
          SendCodeVal('I', interrupts[i].intr_id);
          return;
        } // ### else could send an error, attempt to disable...
      }
    }
  } else SendErrorMsg("11"); // No ack on INTA
}

#endif // SUBBUS_INTERRUPTS

#if USE_SUBBUS

subbus_t::subbus_t() {
  //add_driver(new subbus_base);
}

#if USE_SUBBUS_DRIVERS
void subbus_t::add_driver(subbus_driver *drv) {
#if 0
  std::list<subbus_driver*>::iterator drvi;
  for (drvi = drivers.begin(); drvi != drivers.end(); ++drvi) {
    if (drv->high < (*drvi)->low) {
      drivers.insert(drvi,drv);
      return;
    } else if (drv->low <= (*drvi)->high) {
      while (true) {} // Invalid address overlap
    }
  }
  drivers.insert(drivers.end(),drv);
#endif
}
#endif

/**
 * @return non-zero if EXPACK is detected.
 */
int subbus_t::sb_read( uint16_t addr, uint16_t *rv ) {
#if 0
  std::list<subbus_driver*>::iterator drvi;
  for (drvi = drivers.begin(); drvi != drivers.end(); ++drvi) {
    if (addr >= (*drvi)->low && addr <= (*drvi)->high)
      return (*drvi)->read(addr, rv);
  }
#endif
  return 0; // no driver found
}

/**
 * @return non-zero if EXPACK is detected.
 */
int subbus_t::write(uint16_t addr, uint16_t data) {
#if 0
  std::list<subbus_driver*>::iterator drvi;
  for (drvi = drivers.begin(); drvi != drivers.end(); ++drvi) {
    if (addr >= (*drvi)->low && addr <= (*drvi)->high)
      return (*drvi)->write(addr, data);
  }
#endif
  return 0; // no driver found
}

void subbus_t::reset(void) {
#if 0
  std::list<subbus_driver*>::iterator drvi;
  for (drvi = drivers.begin(); drvi != drivers.end(); ++drvi) {
    (*drvi)->reset();
  }
#endif
}


subbus_driver::subbus_driver(uint16_t low, uint16_t high) {
  // this->low = low;
  // this->high = high;
}


#if USE_SUBBUS_DRIVERS

/* Default Drivers */

int subbus_base::dr_read(uint16_t addr, uint16_t *rv) {
  switch (addr) {
    case 0:
    case SUBBUS_INTA_ADDR:
      return 0;
    case SUBBUS_BDID_ADDR:
      *rv = SUBBUS_BUILD_NUM;
      return 1;
    case SUBBUS_BDID_ADDR+1:
      *rv = SUBBUS_BOARD_ID;
      return 1;
    default:
      return 0;
  }
}

int subbus_base::dr_write(uint16_t addr, uint16_t data) {
  return 0; // Nothing here is writable
}

void subbus_base::reset() {}

int subbus_fail_sw::read(uint16_t addr, uint16_t *rv) {
  switch(addr) {
    case SUBBUS_FAIL_ADDR:
      *rv = fail_reg;
      return 1;
    case SUBBUS_SWITCHES_ADDR:
      *rv = 0;
      return 1;
    default:
      return 0;
  }
}

int subbus_fail_sw::write(uint16_t addr, uint16_t data) {
  switch(addr) {
    case SUBBUS_FAIL_ADDR:
      fail_reg = data;
      return 1;
    case SUBBUS_SWITCHES_ADDR:
    default:
      return 0; // not writable
  }
}

void subbus_fail_sw::reset() {
  fail_reg = 0;
}

#endif // USE_SUBBUS_DRIVERS
#endif // USE_SUBBUS
