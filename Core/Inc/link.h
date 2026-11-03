#ifndef LINK_H
#define LINK_H

#include <stdint.h>

/* Lop giao tiep chung cho console va ROS, tren 2 duong:
 *   - USB CDC (cong USB-C tren board, Pi thay la /dev/ttyACM0)  : duong chinh
 *   - USART1 (PA9 TX / PA10 RX, 115200)                       : du phong, console qua WCH-Link
 *
 * Duong nao nhan duoc goi ROS hop le dau tien thi ROS chay tren duong do.
 * Trong che do ROS: duong ROS chi chua goi nhi phan (printf bi chan tren duong nay),
 * duong con lai van in printf de theo doi (vd console WCH-Link khi ROS chay qua USB).
 *
 * LINK_USE_USB = 0: build khong can USB (khi CubeMX chua bat USB_DEVICE), chi dung UART nhu cu. */
#ifndef LINK_USE_USB
#define LINK_USE_USB 1
#endif

#define LINK_USB   0u
#define LINK_UART  1u

void    link_init(void);                       /* goi SAU MX_USB_Device_Init va cmd_uart_init */
void    link_task(void);                       /* goi lien tuc trong vong lap: day du lieu USB di */

int     link_get(uint8_t *b, uint8_t *src);    /* lay 1 byte tu USB (uu tien) hoac UART */
int     link_get_ros(uint8_t *b);              /* che do ROS: chi byte tu duong ROS, bo byte duong kia */
int     link_any_key(void);                    /* co byte bat ky (USB/UART) -> 1, da lay ra */
void    link_flush_input(void);                /* bo het byte dang cho */

void    link_ros_bind(void);                   /* ros_link goi khi vao che do ROS: chot duong vua nhan */
uint8_t link_ros_src(void);
void    link_ros_write(const uint8_t *d, uint16_t len);   /* goi nhi phan ra duong ROS */

void    link_console_putc(uint8_t c);          /* printf */
void    link_test_write(const uint8_t *d, uint16_t len);  /* canstress u: ghi ra ca 2 duong */

/* Goi tu CDC_Receive_FS trong usbd_cdc_if.c (ngat USB) */
void    link_usb_on_rx(const uint8_t *buf, uint32_t len);
int     link_usb_connected(void);
uint32_t link_usb_tx_dropped(void);

#endif /* LINK_H */
