#ifndef CMD_UART_H
#define CMD_UART_H

#include "home.h"

/* Nhan lenh go tu terminal qua USART1 (PA9/PA10, 115200) va thuc thi.
 * Go "h" de xem danh sach lenh. Byte nhan bang ngat (uart_rx.c). Goi nhi phan tu Pi
 * duoc chuyen cho ros_link.c. */

void cmd_uart_init(void);
void cmd_uart_poll(void);   /* goi lien tuc trong while(1) */

/* Home ca tay kem mo gripper (dung chung cho lenh "home" tren console va CMD_HOME tu Pi) */
home_status_t cmd_home_all(void);

#endif /* CMD_UART_H */
