#ifndef NORDIC_WIRELESS_H
#define NORDIC_WIRELESS_H 1

enum {
  VARIABLE_PAYLOAD_LEN = 0xff
};

void nordic_init(void);
void nordic_set_channel(uint8_t channel);
uint8_t nordic_set_rx_addr(uint8_t *addr, uint8_t addr_len, uint8_t pipe);
void nordic_setup_pipe(uint8_t pipe, uint8_t *addr, uint8_t addr_len,
		       uint8_t enable_aa, uint8_t payload_len);
void nordic_disable_pipe(uint8_t pipe);
void nordic_start_listening(void);
void nordic_stop_listening(void);

void nordic_write_data(uint8_t *buf, uint8_t len);

uint8_t nordic_set_tx_addr(uint8_t *addr, uint8_t addr_len);
uint8_t nordic_data_ready(void);
uint8_t nordic_get_data(uint8_t *buf, uint8_t *len);

/* XXX Delete these? */
uint8_t nordic_get_status(void);
#ifdef SERIAL_DEBUG
void nordic_print_radio_config(void);
#endif
uint8_t nordic_clear_interrupts(void);
void nordic_flush_tx_fifo(void);
void nordic_flush_rx_fifo(void);

#endif /* NORDIC_WIRELESS_H */
