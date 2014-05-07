#include <avr/interrupt.h>
#include "serial.h"
#include "usart_driver.h"

struct serial serial1;

/*! \brief Receive complete interrupt service routine.
 *
 *  Receive complete interrupt service routine.
 *  Calls the common receive complete handler with pointer to the correct USART
 *  as argument.
 */
ISR(USARTE0_RXC_vect)
{
	USART_RXComplete(&serial1);
}


/*! \brief Data register empty  interrupt service routine.
 *
 *  Data register empty  interrupt service routine.
 *  Calls the common data register empty complete handler with pointer to the
 *  correct USART as argument.
 */
ISR(USARTE0_DRE_vect)
{
	USART_DataRegEmpty(&serial1);
}

void
serial_write(char b) {
  bool success;

  do {
    success = USART_TXBuffer_PutByte(&serial1, b);
  } while (!success);
}

void
serial_write_string(const char *str) {
  for (; *str != '\0'; str++) {
    serial_write(*str);
  }
}

/**
 * Get one byte of serial data.
 *
 * Data returned in b.  Return value indicates if anything was written.
 */
int8_t
serial_get_byte(char *b) {
  if (USART_RXBufferData_Available(&serial1)) {
    *b = USART_RXBuffer_GetByte(&serial1);
    return(true);
  } else {
    return(false);
  }
}

void
usart_115200(void) {
  // Initialize the serial driver, with low interrupt priority.
  USART_InterruptDriver_Initialize(&serial1, &USARTE0, USART_DREINTLVL_LO_gc);

  // Set mode of operation. 8-N-1.
  // XXX ??? USART_Format_set...
  USARTE0.CTRLC = (USART_CMODE_ASYNCHRONOUS_gc | USART_PMODE_DISABLED_gc |
		   USART_CHSIZE_8BIT_gc);

  // Enable the receive interrupt.
  USART_RxdInterruptLevel_Set(serial1.usart, USART_RXCINTLVL_LO_gc);

  // Set the buad rate and frame format.
  // From the Excel sheet from Atmel.
  USART_Baudrate_Set(&USARTE0, 1047, -6);

  // Enable the transmit and receive.
  // XXX ??? USARTE0.CTRLB = USART_RXEN_bm | USART_TXEN_bm;
  USART_Rx_Enable(serial1.usart);
  USART_Tx_Enable(serial1.usart);
}
