#ifndef CMD_UART_H
#define CMD_UART_H

/* Nhan lenh go tu terminal qua USART1 (PA9/PA10, 115200) va thuc thi.
 * Go "h" de xem danh sach lenh. */

void cmd_uart_init(void);
void cmd_uart_poll(void);   /* goi lien tuc trong while(1) */

#endif /* CMD_UART_H */
