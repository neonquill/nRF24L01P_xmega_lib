#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>

#include "spi.h"
#include "nRF24L01P.h"
#include "nordic_wireless.h"
#include "xmega_lib/serial.h"

#include "nordic_wireless.h"

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

uint8_t
nordic_set_rx_addr(uint8_t *addr, uint8_t addr_len, uint8_t pipe) {
  uint8_t status;

  status = nordic_write_register(RX_ADDR_P0 + pipe, addr, addr_len);
  return(status);
}

// XXX See footnote d on page 63 for restrictions on ack retransmit with
// variable payloads.
// XXX To transmit variable payload lengths, the transmitter must have
// the DPL_P0 bit set.
void
nordic_setup_pipe(uint8_t pipe, uint8_t *addr, uint8_t addr_len,
		  uint8_t enable_aa, uint8_t payload_len) {

  if (pipe > 5) {
    return;
  }

  nordic_set_rx_addr(addr, addr_len, pipe);

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
    feature |= _BV(EN_DPL);
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

/**
 * Set the transmit address.
 *
 * NOTE: If using auto-acknowledge, the port 0 address needs to be set
 * to the same address before transmitting.  This ensures that we
 * receive the ack packet.
 */
uint8_t
nordic_set_tx_addr(uint8_t *addr, uint8_t addr_len) {
  uint8_t status;

  status = nordic_write_register(TX_ADDR, addr, addr_len);
  return(status);
}

uint8_t
nordic_data_ready(uint8_t status) {
  return((status & _BV(RX_DR)) != 0);
}

#if 0
XXX This might not be correct.
uint8_t
nordic_rx_fifo_empty(void) {
  uint8_t fifo_status;

  nordic_read_register(FIFO_STATUS, &fifo_status, 1);
  return((fifo_status & _BV(RX_EMPTY)) == 0);
}
#endif

/**
 * Get a packet from the radio.
 *
 * NOTE: Assumes that we've already verified that there is data to read.
 *
 * @param[in] status  Recent value of the status register.
 * @param[out] packet  Pointer to a packet structure to store the data in.
 * @return Boolean true if success.
 */
uint8_t
nordic_read_packet(uint8_t status, struct packet_data *packet) {
  uint8_t payload_length;
  int8_t pipe;
  int i;
  uint8_t *buf;
#ifdef SERIAL_DEBUG
  char txt[32];
#endif

#ifdef SERIAL_DEBUG
  snprintf(txt, 32, "Stat: 0x%x\r\n", status);
  serial_write_string(txt);
#endif

  // The status register says which pipe the first packet came from.
  pipe = (status >> 1) & 0x7;
  payload_length = pipe_payload_len[pipe];
  packet->pipe = pipe;

  // XXX If we're using dynamic payload length, need to check the length.
  if ((feature & _BV(EN_DPL)) && (payload_length == VARIABLE_PAYLOAD_LEN)) {
    nordic_cs_low();
    spi_transfer(R_RX_PL_WID);
    payload_length = spi_transfer(0xff);
    nordic_cs_high();

#ifdef SERIAL_DEBUG
    snprintf(txt, 32, "Var len: %d\r\n", payload_length);
    serial_write_string(txt);
#endif
  } else {
#ifdef SERIAL_DEBUG
    snprintf(txt, 32, "Fix len: %d\r\n", payload_length);
    serial_write_string(txt);
#endif
  }

  // If it's larger than 32 bytes, flusth the RX FIFO.
  if (payload_length > 32) {
    nordic_cs_low();
    spi_transfer(FLUSH_RX);
    nordic_cs_high();
#ifdef SERIAL_DEBUG
    snprintf(txt, 32, "Too long!\r\n");
    serial_write_string(txt);
#endif
    return(0);
  }

  // Read the payload.
  nordic_cs_low();
  spi_transfer(R_RX_PAYLOAD);
  for (i = 0, buf = packet->data; i < payload_length; i++) {
      *buf = spi_transfer(0xff);
      buf++;
  }
  nordic_cs_high();
  packet->len = payload_length;

  return(1);
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

uint8_t
nordic_write_data(uint8_t *buf, uint8_t len) {
  uint8_t status;
  uint8_t data;

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

  /* Spin until the transmit finishes. */
  while (1) {
    status = nordic_get_status();
    if (((status & _BV(TX_DS)) != 0) || ((status & _BV(MAX_RT)) != 0)) {
      data = _BV(TX_DS) | _BV(MAX_RT);
      nordic_write_register(STATUS, &data, 1);
      break;
    }
    _delay_ms(100);
  }

  return(status & _BV(TX_DS));
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

/* Buffer to store packets. */
static struct packet_data g_packets[3];
static int8_t g_first_valid_packet = -1;
static int8_t g_next_empty_packet = 0;

/**
 * Process interrupt data.
 *
 * This function assumes that all data buffered in this function
 * is consumed before being called again.
 */
void
nordic_process_interrupt(void) {
  uint8_t status, success;
  uint8_t clear_bits = 0;

  status = nordic_get_status();

  if (status & _BV(MAX_RT)) {
    // serial_write_string("Max retransmit\r\n");
    clear_bits |= _BV(MAX_RT);
  }

  if (status & _BV(TX_DS)) {
    // serial_write_string("Successful transfer\r\n");
    clear_bits |= _BV(TX_DS);
  }

  /*
   * From the data sheet:
   *  The RX_DR IRQ is asserted by anew packet arrival event.  The
   *  procedure for handling this interrupt should be:
   *    1) read payload through SPI,
   *    2) clear RX_DR IRQ,
   *    3) read FIFO_STATUS to check if there are more payloads
   *       available in RX FIFO,
   *    4) if there are more data in RX FIFO, repeat from step 1).
   */
  if (status & _BV(RX_DR)) {
    // serial_write_string("Data ready\r\n");
    clear_bits |= _BV(RX_DR);

    /* Read in all the received packets. */
    while (g_next_empty_packet < 3) {
      success = nordic_read_packet(status, &g_packets[g_next_empty_packet]);
      if (success) {
        g_next_empty_packet++;
        if (g_first_valid_packet < 0) {
          g_first_valid_packet = 0;
        }
      }

      /* Clear the interrupts. */
      nordic_write_register(STATUS, &clear_bits, 1);

      /* Get the status for the next pipe value. */
      status = nordic_get_status();
      if ((status & 0x0e) == 0x0e) {
        /* RX fifo is empty, can stop now. */
        break;
      }

      /* Set the clear bits for the next round. */
      clear_bits = _BV(RX_DR);
    }
  }
}

/**
 * Get a cached packet.
 *
 * @return Pointer to a packet struct, or NULL.
 */
struct packet_data *
nordic_get_packet(void) {
  struct packet_data *p;

  if (g_first_valid_packet < 0) {
    return(NULL);
  }

  if ((g_first_valid_packet >= 3) ||
      (g_first_valid_packet == g_next_empty_packet)) {
    g_first_valid_packet = -1;
    g_next_empty_packet = 0;
    return(NULL);
  }

  p = &g_packets[g_first_valid_packet];

  g_first_valid_packet++;
  if (g_first_valid_packet >= 3) {
    g_first_valid_packet = -1;
    g_next_empty_packet = 0;
  }

  return(p);
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

  nordic_read_register(DYNPD, buffer, 1);
  snprintf(txt, 32, "DYNPD: %x\r\n", *buffer);
  serial_write_string(txt);

  nordic_read_register(FEATURE, buffer, 1);
  snprintf(txt, 32, "FEAT: %x\r\n", *buffer);
  serial_write_string(txt);
}
#endif
