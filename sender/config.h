#ifndef CONFIG_H
#define CONFIG_H 1

/*
 * The port macros here use the PORTA_OUTSET form rather than the
 * PORTA.OUTSET form.  This saves an extra memory lookup at run time,
 * but makes the macro expansion more complicated.
 */

/* Port and pin for the nordic chip select input. */
#define NORDIC_CS_PORT(op) PORTA_##op
#define NORDIC_CS_PIN PIN6_bm

/* Port and pin for the nordic chip enable input. */
#define NORDIC_CE_PORT(op) PORTA_##op
#define NORDIC_CE_PIN PIN5_bm

#endif /* CONFIG_H */
