#include <avr/io.h>
#include <util/delay.h>

/*
 * Pins:
 *
 * Port A:
 *  1: Debug LED
 *
 * Port E:
 *  2: Serial Rx
 *  3: Serial Tx
 */
void
setup_pins(void) {
  /* For direction: 0 = input, 1 = output. */

  /* Port A: 1 output, rest don't care (output). */
  PORTA.DIR = 0xff;
  // Leave the IR LED low.
  PORTA.OUT = 0x00;
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
setup(void) {
  setup_pins();
}


void
loop(void) {
  blink(3);
  _delay_ms(1000);
}

int
main(void) {
  setup();

  while(1) {
    loop();
  }
}
