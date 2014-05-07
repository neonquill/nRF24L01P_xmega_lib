#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include "xmega_lib/clksys_driver.h"
#include "xmega_lib/serial.h"

#include "nordic_wireless.h"

/*
 * Pins:
 *
 * Port A:
 *  1: Debug LED
 *  5: nordic CE
 *  6: nordic _CS_
 *
 * Port B:
 *  2: nordic IRQ
 *
 * Port D: (uses SPI)
 *  1: nordic SCK
 *  2: nordic MISO
 *  3: nordic MOSI
 *
 * Port E:
 *  2: Serial Rx
 *  3: Serial Tx
 */
void
setup_pins(void) {
  /* For direction: 0 = input, 1 = output. */

  /* Port A: 1, 5, 6 output, rest don't care (output). */
  PORTA.DIR = 0xff;
  // Leave the IR LED, nordic CE, nordic _CS_ all low.
  PORTA.OUT = 0x00;

  /* Port B: 2 input, rest don't care (output). */
  PORTB.DIR = (PIN0_bm | PIN1_bm | PIN3_bm | PIN4_bm | PIN5_bm | PIN6_bm |
               PIN7_bm);
  PORTB.OUT = 0x00;
  /* Turn on a low level interrupt on the nordic IRQ pin. */
  PORTB.INTCTRL = PORT_INT0LVL_LO_gc;
  PORTB.INT0MASK = PIN2_bm;
  /* Trigger on a falling edge. */
  PORTB.PIN2CTRL = PORT_ISC_FALLING_gc;

  /* Port C: don't care: all outputs, start at 0. */
  PORTC.DIR = 0xff;
  PORTC.OUT = 0x00;

  /* Port D: 1, 3 outputs; 2 input, reset don't care (output). */
  PORTD.DIR = (PIN0_bm | PIN1_bm | PIN3_bm | PIN4_bm | PIN5_bm | PIN6_bm |
               PIN7_bm);
  // Leave all the nordic pins low.
  PORTD.OUT = 0x00;

  /* Port E: 2 input, 3 output, reset don't care (output). */
  PORTE.DIR = (PIN0_bm | PIN1_bm | PIN3_bm);
  // Start all lines out low.
  PORTE.OUT = 0x00;
}

/* Interrupt routine for the nordic IRQ. */
ISR(PORTB_INT0_vect) {
  /* XXX */
}

void
blink(int count) {
  int i;

  for (i = 0; i < count; i++) {
    PORTA.OUTSET = PIN1_bm;
    _delay_ms(125);
    PORTA.OUTCLR = PIN1_bm;
    _delay_ms(250);
    PORTA.OUTSET = PIN1_bm;
    _delay_ms(125);
  }

  _delay_ms(400);
}

void
setup_clock(void) {
  // Enable the 32 MHz internal oscillator.
  CLKSYS_Enable(OSC_RC32MEN_bm);

  // Wait for the 32 MHz clock to be ready.
  do {
  } while (CLKSYS_IsReady(OSC_RC32MRDY_bm) == 0);

  // Switch to the 32 MHz clock for the main system clock.
  // XXX Check the return value!
  CLKSYS_Main_ClockSource_Select(CLK_SCLKSEL_RC32M_gc);

  // Turn off all the other clocks.
  // XXX What about the external clock?
  CLKSYS_Disable(OSC_PLLEN_bm | OSC_RC32KEN_bm | OSC_RC2MEN_bm);

  // Enable automatic calibration of the 32MHz clock.
  OSC.DFLLCTRL = OSC_RC32MCREF_XOSC32K_gc;
  DFLLRC32M.CTRL = DFLL_ENABLE_bm;
}

static uint8_t broadcast_address[5] = {0xe7, 0xe7, 0xe7, 0xe7, 0xe7};
static uint8_t boot_address[5] = {0x3e, 0x3e, 0x3e, 0x3e, 0x3e};

void
nordic_setup(void) {
  nordic_init();
  nordic_set_channel(1);
  nordic_setup_pipe(0, broadcast_address, sizeof(broadcast_address), 1,
                    VARIABLE_PAYLOAD_LEN);

#ifdef SERIAL_DEBUG
  // XXX Delay so serial write works correctly...
  _delay_ms(1000);
  serial_write_string("Bob\r\n");
  _delay_ms(1000);
  serial_write_string("Fred\r\n");
  nordic_print_radio_config();
#endif
}

void
setup(void) {
  setup_clock();

  setup_pins();

  usart_115200();

  /* Enable a low level interrupt on port A, pin 2. */
  PORTA.INTCTRL = PORT_INT0LVL_LO_gc;
  PORTA.INT0MASK = PIN2_bm;

  // Enable low and medium level interrupts.
  PMIC.CTRL = PMIC_LOLVLEN_bm | PMIC_MEDLVLEN_bm;

  /* Enable interrupts. */
  sei();

  /* Set up the nordic radio. */
  nordic_setup();
  nordic_start_listening();
}

void
loop(void) {
  uint8_t data[32];
  char txt[32];
  uint8_t status;
  uint8_t len;
  static uint8_t count = 0;

  blink(1);

  if (nordic_data_ready()) {
    len = 3;
    status = nordic_get_data(data, &len);
    snprintf(txt, 32, "0x%x * %d,%d,%d\r\n", status, data[0], data[1], data[2]);
    serial_write_string(txt);

    nordic_clear_interrupts();
    nordic_flush_tx_fifo();

    /* Send a response. */
#if 0
    _delay_ms(500);
    data[0] = 33;
    data[2] = 99;
    snprintf(txt, 32, "s: %d,%d,%d\r\n", data[0], data[1], data[2]);
    serial_write_string(txt);
    nordic_write_data(data, 3);
    nordic_start_listening();
#endif
  }

  //nordic_print_radio_config();

  /* To keep us from getting stuck. */
  nordic_flush_tx_fifo();
  nordic_clear_interrupts();

  data[0] = 33;
  data[1] = count;
  data[2] = 99;
  snprintf(txt, 32, "s: %d,%d,%d\r\n", data[0], data[1], data[2]);
  serial_write_string(txt);
  nordic_set_tx_addr(broadcast_address, sizeof(broadcast_address));
  nordic_set_rx_addr(broadcast_address, sizeof(broadcast_address), 0);
  nordic_write_data(data, 3);

  data[0] = 22;
  data[1] = 88;
  data[2] = count;
  data[3] = 111;
  snprintf(txt, 32, "s: %d,%d,%d,%d\r\n", data[0], data[1], data[2], data[3]);
  serial_write_string(txt);
  nordic_set_tx_addr(boot_address, sizeof(boot_address));
  nordic_set_rx_addr(boot_address, sizeof(boot_address), 0);
  nordic_write_data(data, 4);

  nordic_set_rx_addr(broadcast_address, sizeof(broadcast_address), 0);

  count++;

  nordic_print_radio_config();

  _delay_ms(1000);
}

int
main(void) {
  setup();

  blink(3);

  while(1) {
    loop();
  }
}
