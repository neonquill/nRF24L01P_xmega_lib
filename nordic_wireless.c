#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>

#include "spi.h"
#include "nRF24L01P.h"
#include "nordic_wireless.h"
#include "xmega_lib/serial.h"
#include "config.h"

#include "nordic_wireless.h"

/* Local variables used to keep track of register values. */
static uint8_t en_rxaddr;
static uint8_t en_aa = 0x00;
static uint8_t dynpd = 0x00;
static uint8_t feature = 0x00;
static uint8_t config = 0x00;

/* Keep track of the packet size per pipe. */
static uint8_t pipe_payload_len[6] = {0};

/* Keep track of the state of the radio. */
static enum nordic_state radio_state;

/**
 * Simple wrapper to set the nordic chip select pin low.
 */
static void
nordic_cs_low(void) {
  NORDIC_CS_PORT(OUTCLR) = NORDIC_CS_PIN;
}

/**
 * Simple wrapper to set the nordic chip select pin high.
 */
static void
nordic_cs_high(void) {
  NORDIC_CS_PORT(OUTSET) = NORDIC_CS_PIN;
}

/**
 * Simple wrapper to set the nordic chip enable pin low.
 */
static void
nordic_ce_low(void) {
  NORDIC_CE_PORT(OUTCLR) = NORDIC_CE_PIN;
}

/**
 * Simple wrapper to set the nordic chip enable pin high.
 */
static void
nordic_ce_high(void) {
  NORDIC_CE_PORT(OUTSET) = NORDIC_CE_PIN;
}

/**
 * Set the value of a single byte register.
 *
 * @param[in] address  Address of the register.
 * @param[in] value  Value to set the register to.
 * @return  The value of the status register.
 */
static uint8_t
nordic_config_register(uint8_t address, uint8_t value) {
  uint8_t status;

  nordic_cs_low();

  status = spi_transfer(W_REGISTER | address);
  spi_transfer(value);

  nordic_cs_high();

  return(status);
}

/**
 * Set the value of a multiple byte register.
 *
 * @param[in] address  Address of the register.
 * @param[in] value  Pointer to an array of values.
 * @param[in] len  Length of the value array.
 * @return  The value of the status register.
 */
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

/**
 * Get the value of a multiple byte register.
 *
 * @param[in] address  Address of the register.
 * @param[out] value  Pointer to a storage array for the data.
 * @param[in] len  Number of bytes to read.
 * @return  The value of the status register.
 */
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

/**
 * Power down the radio.
 *
 * @return Boolean, true if successful.
 */
static uint8_t
nordic_power_down(void) {
  config = _BV(EN_CRC);
  nordic_config_register(CONFIG, config);
  radio_state = POWER_DOWN;
  return(1);
}

/**
 * Switch to the standby state.
 *
 * @return Boolean, true if successful.
 */
static uint8_t
nordic_standby(void) {
  if (radio_state == STANDBY_I) {
    /* Already here, just return. */
    return(1);
  }

  if (radio_state == POWER_DOWN) {
    /* Power on. */
    config |= _BV(PWR_UP);
    nordic_config_register(CONFIG, config);
    radio_state = XO_START_UP;

    /*
     * Must wait for Tpd2stby ms before we're in standby.
     * Worst case appears to be 4.5ms.
     */
    _delay_us(500);
    radio_state = STANDBY_I;
    return(1);
  }

  /* All other states require setting CE to low to return to standby. */
  nordic_ce_low();
  radio_state = STANDBY_I;
  return(1);
}

/**
 * Switch to the receive mode state.
 *
 * @return Boolean, true if successful.
 */
static uint8_t
nordic_receive_mode(void) {
  uint8_t new_config;

  if (radio_state == RX) {
    /* Already here, just return. */
    return(1);
  }

  /* First, switch to standby. */
  /* If we're in a TX state, this will return us to standby. */
  nordic_ce_low();
  /* If we're powered down, move back to standby. */
  if (radio_state == POWER_DOWN) {
    if (!nordic_standby()) {
      return(0);
    }
  }

  /* Now switch to receive mode. */
  new_config = config | _BV(PRIM_RX);
  if (new_config != config) {
    config = new_config;
    nordic_config_register(CONFIG, config);
  }

  /* Now set CE high. */
  nordic_ce_high();
  /*
   * If we wanted to monitor the settling period:
   *  radio_state = RX_SETTLING;
   *  _delay_us(130);
   */
  radio_state = RX;
  return(1);
}

/**
 * Prepare to switch to the transmit state.
 *
 * Unlike the other states we don't actually switch to the final
 * state.  This just puts us into standby with the right bits set.  We
 * can then set the CE pin high when we're ready.
 *
 * @return Boolean, true if successful.
 */
static uint8_t
nordic_transmit_standby(void) {
  uint8_t new_config;

  if ((radio_state == TX) || (radio_state == TX_SETTLING)) {
    /* Already here, just return. */
    return(1);
  }

  /* First, switch to standby. */
  /* If we're in a RX state, this will return us to standby. */
  nordic_ce_low();
  /* If we're powered down, move back to standby. */
  if (radio_state == POWER_DOWN) {
    if (!nordic_standby()) {
      return(0);
    }
  }

  /* Now set the config state correctly. */
  new_config = config & ~(_BV(PRIM_RX));
  if (new_config != config) {
    config = new_config;
    nordic_config_register(CONFIG, config);
  }

  radio_state = STANDBY_I;
  return(1);
}

/**
 * Get the value of the status register.
 *
 * @return  The value of the status register.
 */
static uint8_t
nordic_get_status(void) {
  uint8_t status;

  nordic_cs_low();

  /* A null command will get the current status. */
  status = spi_transfer(0xff);

  nordic_cs_high();

  return(status);
}

/**
 * Initialize the nordic library.
 */
void
nordic_init(enum ack_payload_state ack_payload) {
  /*
   * SPI Mode 0.
   * Most Significant Bit first.
   * For data, least significant byte first.
   */
  spi_init();

  /*
   * Reset radio to default configuration values.
   * All interrupts enabled, CRC on, 1 byte, powered down, transmit mode.
   */
  config = _BV(EN_CRC);
  nordic_config_register(CONFIG, config);
  radio_state = POWER_DOWN;

  if (ack_payload == ENABLE_ACK_PAYLOAD) {
    /* Turn on ack payloads. */
    feature |= _BV(EN_ACK_PAY);
    nordic_config_register(FEATURE, feature);
  }

  /* Make sure any pending data is flushed. */
  nordic_flush_tx_fifo();
  nordic_flush_rx_fifo();
  nordic_clear_interrupts();

  // XXX Disable all reading channels?
}

/**
 * Set the radio channel.
 *
 * @param[in] channel  Channel to use.
 */
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

/**
 * Set the receive address for a given pipe.
 *
 * NOTE: This is intended for use on pipe 0 when sending
 * auto-acknowledged traffic.  In that case, pipe 0 should be set to
 * the destination address before transmitting.  This is the address
 * the acknowledgment packet will be sent to.  After receiving the
 * acknowledgment packet, this function can be used to return pipe 0
 * to the original address.
 *
 * No attempt is made to validate that the pipe has already been enabled.
 *
 * @param[in] addr  Array of bytes describing the address.
 * @param[in] addr_len  Number of bytes in the address.
 * @param[in] pipe  The number of the pipe to modify.
 *
 * @return The value of the status register.
 */
uint8_t
nordic_set_rx_addr(uint8_t *addr, uint8_t addr_len, uint8_t pipe) {
  uint8_t status;

  status = nordic_write_register(RX_ADDR_P0 + pipe, addr, addr_len);
  return(status);
}

/**
 * Set up a receive pipe.
 *
 * NOTE: To transmit variable payload lengths, the transmitter must have
 *   variable payload lengths enabled on pipe 0.
 *
 * Notes from the data sheet:
 *   If ACK packet payload is activated, ACK packets have dynamic
 *   payload lengths and the Dynamic Payload Length feature should be
 *   enabled for pipe 0 on the transmitter and receiver.  This is to
 *   ensure that they receive the ACK packets with payloads.
 *
 *   If the ACK payload is more than 15 byte in 2Mbps mode the ARD
 *   must be 500μS or more, and if the ACK payload is more than 5 byte
 *   in 1Mbps mode the ARD must be 500μS or more. In 250kbps mode
 *   (even when the payload is not in ACK) the ARD must be 500μS or
 *   more.
 *
 * @param[in] pipe  The number of the pipe to configure.
 * @param[in] addr  Pointer to an array of address bytes.
 * @param[in] addr_len  Number of bytes in the address.
 * @param[in] enable_aa  Boolean, true if auto-acknowledge should be
 *   enabled.
 * @param[in] payload_len  Length of the payload expected on a pipe,
 *   or VARIABLE_PAYLOAD_LEN if the payload length is variable.
 */
void
nordic_setup_pipe(uint8_t pipe, uint8_t *addr, uint8_t addr_len,
		  uint8_t enable_aa, uint8_t payload_len) {

  if (pipe > 5) {
    return;
  }

  nordic_set_rx_addr(addr, addr_len, pipe);

  /* Update the bitmask of enabled addresses. */
  en_rxaddr |= _BV(pipe);
  nordic_config_register(EN_RXADDR, en_rxaddr);

  /* Update the bitmask of auto-acknowledge flags. */
  if (enable_aa) {
    en_aa |= _BV(pipe);
  }
  nordic_config_register(EN_AA, en_aa);

  /* Set up the payload lengths. */
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

/**
 * Disable a receive pipe.
 *
 * @param[in] pipe  Number of the pipe to disable.
 */
void
nordic_disable_pipe(uint8_t pipe) {
  /* Update the bitmask of enabled addresses. */
  en_rxaddr &= (uint8_t)~_BV(pipe);
  nordic_config_register(EN_RXADDR, en_rxaddr);
}

/**
 * Put the radio in receive mode.
 */
void
nordic_start_listening(void) {
  nordic_receive_mode();
}

/**
 * Take the radio out of receive mode.
 */
void
nordic_stop_listening(void) {
  nordic_power_down();
}

/**
 * Set the transmit address.
 *
 * NOTE: If using auto-acknowledge, the port 0 address needs to be set
 * to the same address before transmitting.  This ensures that we
 * receive the ack packet.
 *
 * @param[in] addr  Pointer to an array of address bytes.
 * @param[in] addr_len  Number of bytes in the address.
 *
 * @return Value of the status register.
 */
uint8_t
nordic_set_tx_addr(uint8_t *addr, uint8_t addr_len) {
  uint8_t status;

  status = nordic_write_register(TX_ADDR, addr, addr_len);
  return(status);
}

#define DEBUG_READ 0

/**
 * Get a packet from the radio.
 *
 * NOTE: Assumes that we've already verified that there is data to read.
 *
 * @param[in] status  Recent value of the status register.
 * @param[out] packet  Pointer to a packet structure to store the data in.
 *
 * @return Boolean true if success.
 */
static uint8_t
nordic_read_packet(uint8_t status, struct packet_data *packet) {
  uint8_t payload_length;
  int8_t pipe;
  int i;
  uint8_t *buf;
#if DEBUG_READ
  char txt[32];
#endif

#if DEBUG_READ
  snprintf(txt, 32, "Stat: 0x%x\r\n", status);
  serial_write_string(txt);
#endif

  /* The status register says which pipe the first packet came from. */
  pipe = (status >> 1) & 0x7;
  payload_length = pipe_payload_len[pipe];
  packet->pipe = pipe;

  /* If we're using dynamic payload length, need to check the length. */
  if ((feature & _BV(EN_DPL)) && (payload_length == VARIABLE_PAYLOAD_LEN)) {
    nordic_cs_low();
    spi_transfer(R_RX_PL_WID);
    payload_length = spi_transfer(0xff);
    nordic_cs_high();

#if DEBUG_READ
    snprintf(txt, 32, "Var len: %d\r\n", payload_length);
    serial_write_string(txt);
#endif
  } else {
#if DEBUG_READ
    snprintf(txt, 32, "Fix len: %d\r\n", payload_length);
    serial_write_string(txt);
#endif
  }

  /* If the payload is larger than 32 bytes, flusth the RX FIFO. */
  if (payload_length > 32) {
    nordic_cs_low();
    spi_transfer(FLUSH_RX);
    nordic_cs_high();
#if DEBUG_READ
    snprintf(txt, 32, "Too long!\r\n");
    serial_write_string(txt);
#endif
    return(0);
  }

  /* Read the payload. */
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

/**
 * Helper to transfer a packet payload from the radio.
 *
 * @param[in] buf  Pointer to the buffer to store the data in.
 * @param[in] len  Number of bytes to transfer.
 *
 * @return  The value of the status register.
 */
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

/**
 * Send a packet.
 *
 * @param[in] buf  Buffer containing the data.
 * @param[in] len  Length of the data to send.
 */
void
nordic_write_data(uint8_t *buf, uint8_t len) {
  /* Switch to transmit mode. */
  nordic_transmit_standby();

  /* Transfer the payload to the radio. */
  nordic_transfer_payload(buf, len);

  /* Set chip enable high to actually start the transfer. */
  nordic_ce_high();

  /* Delay at least 10us to make sure we transmit. */
  _delay_us(15);

  /* Turn off chip enable, causing to go back to sleep after the transfer. */
  nordic_ce_low();
}

/**
 * Set the ack payload.
 *
 * @param[in] buf  Pointer to the buffer to store the data in.
 * @param[in] len  Number of bytes to transfer.
 * @param[in] pipe  The pipe to set the ack on.
 *
 * @return  The value of the status register.
 */
uint8_t
nordic_set_ack_payload(uint8_t *buf, uint8_t len, uint8_t pipe) {
  uint8_t status;

  nordic_cs_low();

  status = spi_transfer(W_ACK_PAYLOAD + pipe);
  for (; len > 0; len--, buf++) {
    spi_transfer(*buf);
  }

  nordic_cs_high();

  return(status);
}

/**
 * Clear all interrupts.
 *
 * @return The value of the status register.
 */
// XXX Probably shouldn't be an externally visable call.
uint8_t
nordic_clear_interrupts(void) {
  uint8_t clear_bits = _BV(MAX_RT) | _BV(TX_DS) | _BV(RX_DR);
  uint8_t status;

  status = nordic_write_register(STATUS, &clear_bits, 1);
  return(status);
}

/**
 * Flush the transmit buffer.
 */
void
nordic_flush_tx_fifo(void) {
  nordic_cs_low();

  spi_transfer(FLUSH_TX);

  nordic_cs_high();
}

/**
 * Flush the receive buffer.
 */
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
 *
 * @return The original status register.
 */
uint8_t
nordic_process_interrupt(void) {
  uint8_t status, success, return_status;
  uint8_t clear_bits = 0;

  status = nordic_get_status();
  return_status = status;

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

  } else {
    /* No receive data, so we have to clear the interrupts. */
    nordic_write_register(STATUS, &clear_bits, 1);
  }

  return(return_status);
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
/**
 * Print the various registers from the radio.
 */
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
