#ifndef NORDIC_WIRELESS_H
#define NORDIC_WIRELESS_H 1

enum {
  VARIABLE_PAYLOAD_LEN = 0xff
};

enum nordic_state {
  /* Powered down. */
  POWER_DOWN,

  /* Transitioning from powered down to standby. */
  XO_START_UP,

  /* Standard idle state. */
  STANDBY_I,

  /* Transitioning to receive mode. */
  RX_SETTLING,

  /* Receive mode. */
  RX,

  /* Transitioning to transmit mode. */
  TX_SETTLING,

  /* Transmitting packets. */
  TX,

  /* In transmit mode, without packets to send. */
  STANDBY_II
};

struct packet_data {
  uint8_t pipe;
  uint8_t len;
  uint8_t data[32];
};

void nordic_init(void);
void nordic_set_channel(uint8_t channel);
uint8_t nordic_set_rx_addr(uint8_t *addr, uint8_t addr_len, uint8_t pipe);
void nordic_setup_pipe(uint8_t pipe, uint8_t *addr, uint8_t addr_len,
		       uint8_t enable_aa, uint8_t payload_len);
void nordic_disable_pipe(uint8_t pipe);
void nordic_start_listening(void);
void nordic_stop_listening(void);

uint8_t nordic_write_data(uint8_t *buf, uint8_t len);

uint8_t nordic_set_tx_addr(uint8_t *addr, uint8_t addr_len);
uint8_t nordic_data_ready(uint8_t status);
uint8_t nordic_get_data(uint8_t status, uint8_t *buf, uint8_t *len);
struct packet_data *nordic_get_packet(void);
void nordic_process_interrupt(void);

/* XXX Delete these? */
#ifdef SERIAL_DEBUG
void nordic_print_radio_config(void);
#endif
uint8_t nordic_clear_interrupts(void);
void nordic_flush_tx_fifo(void);
void nordic_flush_rx_fifo(void);

#endif /* NORDIC_WIRELESS_H */
