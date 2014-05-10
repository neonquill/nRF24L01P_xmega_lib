#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <string.h>

#include "xmega_lib/clksys_driver.h"
#include "xmega_lib/serial.h"
#include "xbootapi.h"

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

volatile static uint8_t nordic_interrupt = 0;

/* Interrupt routine for the nordic IRQ. */
ISR(PORTB_INT0_vect) {
  nordic_interrupt = 1;
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


void
nordic_setup(void) {
  uint8_t address[5] = {0xe7, 0xe7, 0xe7, 0xe7, 0xe7};
  uint8_t address2[5] = {0x3e, 0x3e, 0x3e, 0x3e, 0x3e};
  nordic_init();
  nordic_set_channel(1);
  nordic_setup_pipe(0, address, 5, 1, VARIABLE_PAYLOAD_LEN);
  /* Set up a second pipe to receive firmware updates. */
  nordic_setup_pipe(1, address2, 5, 1, VARIABLE_PAYLOAD_LEN);

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

#define DEBUG_CRC 0

void
process_data(uint8_t data[], uint8_t len) {
  static uint8_t page_buffer[SPM_PAGESIZE];
  static uint32_t addr = 0;
  static uint8_t page_committed = 0;
  static uint16_t buffer_offset = 0;
  uint16_t target_crc, crc;
#if DEBUG_CRC
  uint8_t byte;
  uint16_t i;
#endif
  char txt[32];

  if (len < 1) {
    return;
  }

  // XXX Should probably make this a switch.
  if (data[0] == 'B') {
    // Copy the data into the page buffer.

    // Convert len to exclude the command and length.
    len -= 3;

    // Grab the buffer offset from the packet.
    memcpy(&buffer_offset, &data[1], sizeof(buffer_offset));

    if ((buffer_offset + len) > SPM_PAGESIZE) {
      serial_write_string("Buffer overflow!\r\n");
      return;
    }
    memcpy(&page_buffer[buffer_offset], &data[3], len);
    page_committed = 0;
    buffer_offset += len;

  } else if (data[0] == 'e') {
    // Erase the memory.
    if (xboot_app_temp_erase() != XB_SUCCESS) {
      serial_write_string("Erase failed!\r\n");
      return;
    }
    addr = 0;
    page_committed = 0;
    buffer_offset = 0;

  } else if (data[0] == 'm') {
    // Commit the block.
    if (page_committed) {
      serial_write_string("Page already committed!\r\n");
      return;
    }

    // Pad the buffer.
    snprintf(txt, sizeof(txt), "pad at %x\r\n", buffer_offset);
    serial_write_string(txt);

    if (buffer_offset < SPM_PAGESIZE) {
      memset(&page_buffer[buffer_offset], 0xff, SPM_PAGESIZE - buffer_offset);
    }

    // XXX Check for error.
    if (xboot_app_temp_write_page(addr, page_buffer, 0) != XB_SUCCESS) {
      serial_write_string("Write page failed!\r\n");
      return;
    }
    addr += SPM_PAGESIZE;
    page_committed = 1;
    buffer_offset = 0;

  } else if (data[0] == 'w') {
    // Finalize the data.
    if (len < 3) {
      serial_write_string("Final msg err!\r\n");
      return;
    }

    memcpy(&target_crc, &data[1], sizeof(target_crc));
#if DEBUG_CRC
    serial_write_string("CRC\r\n");
    for (i = 0, addr = XB_APP_TEMP_START; i < XB_APP_TEMP_SIZE; i++) {
      byte = pgm_read_byte_near(addr);
      if (byte != 0xff) {
        snprintf(txt, sizeof(txt), "%x: %x\r\n", (uint16_t)addr, byte);
        serial_write_string(txt);
      }
      addr++;
    }
    serial_write_string("END CRC\r\n");
#endif
    xboot_app_temp_crc16(&crc);
    if (crc != target_crc) {
      serial_write_string("CRC error! ");
      snprintf(txt, sizeof(txt), "%x != %x\r\n", crc, target_crc);
      serial_write_string(txt);
      return;
    }

    // Check
    if (xboot_install_firmware(target_crc) != XB_SUCCESS) {
      serial_write_string("Install error!\r\n");
      return;
    }

    serial_write_string("Rebooting!\r\n");
    blink(4);

    xboot_reset();
  }
}

#define DEBUG_PACKET_BYTES 0

void
process_incoming_data(void) {
  struct packet_data *packet;
  uint8_t pipe;
  static uint8_t count = 0;
#if DEBUG_PACKET_BYTES
  uint8_t i;
  char txt[32];
#endif

  //nordic_print_radio_config();

  while (1) {
    packet = nordic_get_packet();
    if (packet == NULL) {
      break;
    }

    // At this point, we know we have data.
    blink(1);

    pipe = packet->pipe;

#if DEBUG_PACKET_BYTES
    snprintf(txt, 32, "%d, %d * %d", count, pipe, packet->data[0]);
    serial_write_string(txt);
    for (i = 1; i < packet->len; i++) {
      snprintf(txt, 32, ",%d", packet->data[i]);
      serial_write_string(txt);
    }
    serial_write_string("\r\n");
#endif

    count++;

    if (pipe == 1) {
      process_data(packet->data, packet->len);
    }

    //nordic_flush_tx_fifo();
    //nordic_flush_rx_fifo();
  }
}

void
loop(void) {
  char b;

  if (nordic_interrupt) {
    nordic_interrupt = 0;
    nordic_process_interrupt();
    process_incoming_data();
  }

  /* To keep us from getting stuck. */
  //nordic_flush_tx_fifo();
  //nordic_clear_interrupts();

  /* Check for incoming serial data. */
  /* XXX Create a flag for this. */
  if (serial_get_byte(&b)) {
    /* For now, only serial data is for the bootloader. */
    if (b == 0x1B) {
      /* Reset to start the bootloader. */
      CCPWrite(&RST.CTRL, RST_SWRST_bm);
    }
  }

  //_delay_ms(1000);
}

int
main(void) {
  setup();

  blink(3);

  while(1) {
    loop();
  }
}
