#ifndef NORDIC_WIRELESS_H
#define NORDIC_WIRELESS_H 1

enum {
  VARIABLE_PAYLOAD_LEN = 0xff
};

enum ack_payload_state {
  NO_ACK_PAYLOAD,
  ENABLE_ACK_PAYLOAD,
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

/* XXX nRF24L01P.h conflicts with other headers, so import these. */
#ifndef RX_DR
#define RX_DR       6
#endif

#ifndef TX_DS
#define TX_DS       5
#endif

#ifndef MAX_RT
#define MAX_RT      4
#endif

struct packet_data {
  uint8_t pipe;
  uint8_t len;
  uint8_t data[32];
};

void nordic_init(enum ack_payload_state ack_payload);
void nordic_set_channel(uint8_t channel);
uint8_t nordic_set_rx_addr(uint8_t *addr, uint8_t addr_len, uint8_t pipe);
void nordic_setup_pipe(uint8_t pipe, uint8_t *addr, uint8_t addr_len,
		       uint8_t enable_aa, uint8_t payload_len);
void nordic_disable_pipe(uint8_t pipe);
void nordic_start_listening(void);
void nordic_stop_listening(void);

void nordic_write_data(uint8_t *buf, uint8_t len);
uint8_t nordic_set_ack_payload(uint8_t *buf, uint8_t len, uint8_t pipe);

uint8_t nordic_set_tx_addr(uint8_t *addr, uint8_t addr_len);
uint8_t nordic_data_ready(uint8_t status);
uint8_t nordic_get_data(uint8_t status, uint8_t *buf, uint8_t *len);
struct packet_data *nordic_get_packet(void);
uint8_t nordic_process_interrupt(void);

/* XXX Delete these? */
#ifdef SERIAL_DEBUG
void nordic_print_radio_config(void);
#endif
void nordic_flush_tx_fifo(void);
void nordic_flush_rx_fifo(void);

#endif /* NORDIC_WIRELESS_H */
