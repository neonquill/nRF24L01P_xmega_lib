#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>

#include "spi.h"
#include "nRF24L01P.h"
#include "nordic_wireless.h"
#include "xmega_lib/serial.h"

// XXX Create interrupt to handle pin change on irq from radio.

// Local variables used to keep track of register values.
static uint8_t en_rxaddr;
static uint8_t en_aa = 0x00;
static uint8_t dynpd = 0x00;
static uint8_t feature = 0x00;

// Keep track of the packet size per pipe.
static uint8_t pipe_payload_len[6] = {0};

// XXX Need to pass in the chip select pin into this library.
static void
nordic_cs_low(void) {
  PORTA.OUTCLR = PIN6_bm;
}

static void
nordic_cs_high(void) {
  PORTA.OUTSET = PIN6_bm;
}

// XXX Need to pass in the chip enable pin into this library.
static void
nordic_ce_low(void) {
  PORTA.OUTCLR = PIN5_bm;
}

static void
nordic_ce_high(void) {
  PORTA.OUTSET = PIN5_bm;
}

static uint8_t
nordic_config_register(uint8_t address, uint8_t value) {
  uint8_t status;

  nordic_cs_low();

  status = spi_transfer(W_REGISTER | address);
  spi_transfer(value);

  nordic_cs_high();

  return(status);
}

static uint8_t
nordic_write_register(uint8_t address, uint8_t *value, uint8_t len) {
  uint8_t status;
  int8_t i;

  nordic_cs_low();

  status = spi_transfer(W_REGISTER | address);
  for (i = len - 1; i >= 0; i--) {
    spi_transfer(value[i]);
  }

  nordic_cs_high();

  return(status);
}

static uint8_t
nordic_read_register(uint8_t address, uint8_t *value, uint8_t len) {
  uint8_t status;
  int8_t i;

  nordic_cs_low();

  status = spi_transfer(R_REGISTER | address);
  for (i = len, value = value + (len - 1); i > 0; i--, value--) {
    *value = spi_transfer(0xff);
  }

  nordic_cs_high();

  return(status);
}

static void
save_port0_address(uint8_t *addr, uint8_t len) {
  // XXX Need to implement.
}

/* static */ uint8_t
nordic_get_status(void) {
  uint8_t status;

  nordic_cs_low();

  // A null command will get the current status.
  status = spi_transfer(0xff);

  nordic_cs_high();

  return(status);
}

void
nordic_init(void) {
  // SPI Mode 0.
  // Most Significant Bit first.
  // For data, least significant byte first.
  spi_init();

  // Reset radio to default configuration values.
  // All interrupts enabled, CRC on, 1 byte, powered down, transmit mode.
  nordic_config_register(CONFIG, _BV(EN_CRC));

  // Make sure any pending data is flushed.
  nordic_flush_tx_fifo();
  nordic_flush_rx_fifo();
  nordic_clear_interrupts();

  // XXX Disable all reading channels.
}

void
nordic_set_channel(uint8_t channel) {
  uint8_t set_channel;
  int8_t count = 0;

  do {
    nordic_config_register(RF_CH, channel);
    nordic_read_register(RF_CH, &set_channel, 1);
    count++;
  } while ((channel != set_channel) && (count < 4));
}

// XXX See footnote d on page 63 for restrictions on ack retransmit with
// variable payloads.

void
nordic_setup_pipe(uint8_t pipe, uint8_t *addr, uint8_t addr_len,
		  uint8_t enable_aa, uint8_t payload_len) {

  if (pipe > 5) {
    return;
  }

  // We need to keep a copy of the address of pipe 0, just in case we
  // decide to use auto-acknowledge.
  if (pipe == 0) {
    save_port0_address(addr, addr_len);
  }

  // Send the address.
  nordic_write_register(RX_ADDR_P0 + pipe, addr, addr_len);
  
  // Update the bitmask of enabled addresses.
  en_rxaddr |= _BV(pipe);
  nordic_config_register(EN_RXADDR, en_rxaddr);

  // Update the bitmask of auto-acknowledge flags.
  if (enable_aa) {
    en_aa |= _BV(pipe);
  }
  nordic_config_register(EN_AA, en_aa);

  // Set up the payload lengths.
  pipe_payload_len[pipe] = payload_len;
  if (payload_len == VARIABLE_PAYLOAD_LEN) {
    dynpd |= _BV(pipe);
    nordic_config_register(DYNPD, dynpd);
    feature |= EN_DPL;
    nordic_config_register(FEATURE, feature);
  } else {
    nordic_config_register(RX_PW_P0 + pipe, payload_len);
  }
}

void
nordic_disable_pipe(uint8_t pipe) {
  // Update the bitmask of enabled addresses.
  en_rxaddr &= (uint8_t)~_BV(pipe);
  nordic_config_register(EN_RXADDR, en_rxaddr);
}

void
nordic_start_listening(void) {
  // XXX Add a callback function as a parameter. ???
  //rx_callback = callback;

  // XXX Save the CONFIG scheme?
  nordic_config_register(CONFIG, _BV(EN_CRC) | _BV(PWR_UP) | _BV(PRIM_RX));

  // Set CE pin high.
  nordic_ce_high();
  return;
}

void
nordic_stop_listening(void) {
  // XXX Set CE pin low.

  // XXX Now read out the payload data?

  // XXX Power down?
}

uint8_t
nordic_data_ready(void) {
  uint8_t fifo_status;

  nordic_read_register(FIFO_STATUS, &fifo_status, 1);
  return(!(fifo_status & _BV(RX_EMPTY)));
}

/**
 * Get a packet from the radio.
 *
 * @param[out] buf  Buffer to store the data in.
 * @param[in,out] len  Length of the buffer (in), length of the payload (out).
 * @return Status register or 0xff on error.
 */
uint8_t
nordic_get_data(uint8_t *buf, uint8_t *len) {
  uint8_t status;
  uint8_t payload_length;
  int8_t pipe;
  int i;

  // Make sure there's something in the buffer.
  if (!nordic_data_ready()) {
    return(0xff);
  }

  // XXX Probably don't need to re-fetch the status.
  status = nordic_get_status();
  // The status register says which pipe the first packet came from.
  pipe = (status >> 1) & 0x7;
  payload_length = pipe_payload_len[pipe];

  nordic_cs_low();

  // XXX If we're using dynamic payload length, need to check the length.
  if ((feature & EN_DPL) && (payload_length == VARIABLE_PAYLOAD_LEN)) {
    spi_transfer(R_RX_PL_WID);
    payload_length = spi_transfer(0xff);
  }

  // If it's larger than 32 bytes, flusth the RX FIFO.
  if (payload_length > 32) {
    spi_transfer(FLUSH_RX);
    nordic_cs_high();
    return(0xff);
  }

  // Read the payload.
  status = spi_transfer(R_RX_PAYLOAD);
  for (i = 0; i < payload_length; i++) {
    if (i < *len) {
      *buf = spi_transfer(0xff);
      buf++;
    } else {
      spi_transfer(0xff);
    }
  }
  *len = payload_length;

  nordic_cs_high();

  return(status);

  // XXX Clear RX_DR IRQ - after everything is transfered?
}


static uint8_t
nordic_transfer_payload(uint8_t *buf, uint8_t len) {
  uint8_t status;

  nordic_cs_low();

  status = spi_transfer(W_TX_PAYLOAD);
  for(; len > 0; len--, buf++) {
    spi_transfer(*buf);
  }

  nordic_cs_high();

  return(status);
}

void
nordic_write_data(uint8_t *buf, uint8_t len) {
  /* If we were listening, we need to return to Standby-I. */
  nordic_ce_low();

  // Power up in transmit mode.
  nordic_config_register(CONFIG, _BV(EN_CRC) | _BV(PWR_UP));

  // Transfer the payload to the radio.
  nordic_transfer_payload(buf, len);

  // Must wait for Tpd2stby ms before the CE pin is set high.
  // Worst case appears to be 4.5ms
  _delay_us(500);

  // Set chip enable high to actually start the transfer.
  nordic_ce_high();

  // Delay at least 10us to make sure we transmit.
  // XXX Eventually, might want to check status...
  _delay_us(15);

  // Turn off chip enable, causing to go back to sleep after the transfer.
  nordic_ce_low();

  // XXX XXX XXX Read more about transmitting, not sure if this is right.
  // XXX Probably need to power down...
}

// XXX Probably shouldn't be an externally visable call.
uint8_t
nordic_clear_interrupts(void) {
  uint8_t clear_bits = _BV(MAX_RT) | _BV(TX_DS) | _BV(RX_DR);
  uint8_t status;

  status = nordic_write_register(STATUS, &clear_bits, 1);
  return(status);
}

void
nordic_flush_tx_fifo(void) {
  nordic_cs_low();

  spi_transfer(FLUSH_TX);

  nordic_cs_high();
}

void
nordic_flush_rx_fifo(void) {
  nordic_cs_low();

  spi_transfer(FLUSH_RX);

  nordic_cs_high();
}

#ifdef SERIAL_DEBUG
void
nordic_print_radio_config(void) {
  uint8_t buffer[5];
  char txt[32];
  uint8_t i;

  uint8_t status = nordic_read_register(RX_ADDR_P0, buffer, 5);
  serial_write_string("\r\nStatus: ");
  snprintf(txt, 32, "%x\r\n", status);
  serial_write_string(txt);

  serial_write_string("Rx address 0: ");
  for (i = 0; i < 5; i++) {
    snprintf(txt, 32, "%02x ", buffer[i]);
    serial_write_string(txt);
  }
  serial_write_string("\r\n");

  status = nordic_read_register(RX_ADDR_P1, buffer, 5);
  serial_write_string("Rx address 1: ");
  for (i = 0; i < 5; i++) {
    snprintf(txt, 32, "%02x ", buffer[i]);
    serial_write_string(txt);
  }
  serial_write_string("\r\n");

  status = nordic_read_register(TX_ADDR, buffer, 5);
  serial_write_string("Tx address : ");
  for (i = 0; i < 5; i++) {
    snprintf(txt, 32, "%02x ", buffer[i]);
    serial_write_string(txt);
  }
  serial_write_string("\r\n");

  nordic_read_register(EN_AA, buffer, 1);
  snprintf(txt, 32, "AA: %x\r\n", *buffer);
  serial_write_string(txt);

  nordic_read_register(EN_RXADDR, buffer, 1);
  snprintf(txt, 32, "EN: %x\r\n", *buffer);
  serial_write_string(txt);

  nordic_read_register(RF_CH, buffer, 1);
  snprintf(txt, 32, "CH: %x\r\n", *buffer);
  serial_write_string(txt);

  nordic_read_register(FIFO_STATUS, buffer, 1);
  snprintf(txt, 32, "FI: %x\r\n", *buffer);
  serial_write_string(txt);

  nordic_read_register(CONFIG, buffer, 1);
  snprintf(txt, 32, "CONFIG: %x\r\n", *buffer);
  serial_write_string(txt);
}
#endif
