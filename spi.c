#include <avr/io.h>

// XXX We're using the USART here, not the SPI module.

// XXX Pass mode and bit ordering as parameters?
void
spi_init(void) {
  // Disable all interrupts for now.
  USARTD0.CTRLA = 0;
  // Enable the receiver and the transmitter.
  USARTD0.CTRLB = (USART_RXEN_bm | USART_TXEN_bm);
  // Master SPI mode.
  // Data order: MS bit first (DORD = 0).
  // Mode 0:  INVEN = 0 (set on the pin), UCPHA = 0.
  USARTD0.CTRLC = USART_CMODE_MSPI_gc;
  // Nordic can handle up to a 10 MHz clock.
  // BSEL = 1 => Periferal clock / 4 = 32 MHz / 4 = 8 MHz.
  // BSCALE doesn't appear to be used for master SPI mode.
  USARTD0.BAUDCTRLA = 1;
  USARTD0.BAUDCTRLB = 0;
}

uint8_t
spi_transfer(uint8_t data) {
  // Write the data to the data register.
  USARTD0.DATA = data;

  // Wait for the write/read to be finished.
  do {} while ((USARTD0.STATUS & USART_TXCIF_bm) == 0);

  // Clear the transmit complete flag.
  // XXX This would be done automatically if we used the interrupt.
  USARTD0.STATUS = USART_TXCIF_bm;

  // Return data is now in the USART data register.
  return(USARTD0.DATA);
}
