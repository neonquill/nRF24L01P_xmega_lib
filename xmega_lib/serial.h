#ifndef SERIAL_H
#define SERIAL_H 1

void serial_write(char b);
void serial_write_string(const char *str);
int8_t serial_get_byte(char *b);
void usart_115200(void);

#endif /* SERIAL_H */
