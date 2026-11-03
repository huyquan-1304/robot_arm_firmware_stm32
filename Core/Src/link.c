#include "link.h"
#include "main.h"
#include <stddef.h>
#include "uart_rx.h"
#include "ros_link.h"

#if LINK_USE_USB
#include "usb_device.h"
#include "usbd_cdc_if.h"
extern USBD_HandleTypeDef hUsbDeviceFS;
#endif

extern UART_HandleTypeDef huart1;   /* do CubeMX tao trong main.c */

static uint8_t cur_src = LINK_USB;  /* duong cua byte vua lay boi link_get */
static uint8_t ros_src = LINK_USB;  /* duong ROS dang dung */

/* ===================== USB CDC ===================== */
#if LINK_USE_USB
#define URX_SIZE 1024u              /* luy thua cua 2 */
#define UTX_SIZE 2048u
#define CHUNK    63u                /* < 64 (goi toi da Full Speed): khong can goi rong (ZLP) */

static volatile uint8_t  urx[URX_SIZE];
static volatile uint16_t urx_head = 0, urx_tail = 0;
static volatile uint32_t urx_overflow = 0;

static uint8_t  utx[UTX_SIZE];
static uint16_t utx_head = 0, utx_tail = 0;
static uint32_t utx_dropped = 0;
static uint8_t  chunk[2][CHUNK];    /* 2 bo dem luan phien: bo dang gui khong bi ghi de */
static uint8_t  chunk_idx = 0;

void link_usb_on_rx(const uint8_t *buf, uint32_t len)
{
    for (uint32_t i = 0; i < len; i++) {
        uint16_t next = (uint16_t)((urx_head + 1u) & (URX_SIZE - 1u));
        if (next == urx_tail) { urx_overflow++; continue; }
        urx[urx_head] = buf[i];
        urx_head = next;
    }
}

int link_usb_connected(void)
{
    return hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED;
}

uint32_t link_usb_tx_dropped(void) { return utx_dropped; }

static int usb_get(uint8_t *b)
{
    if (urx_tail == urx_head) return 0;
    *b = urx[urx_tail];
    urx_tail = (uint16_t)((urx_tail + 1u) & (URX_SIZE - 1u));
    return 1;
}

static uint16_t utx_used(void) { return (uint16_t)((utx_head - utx_tail) & (UTX_SIZE - 1u)); }

/* Day toi da 63 byte ra USB neu lan gui truoc da xong. Khong bao gio cho. */
static void usb_flush(void)
{
    if (!link_usb_connected()) { utx_tail = utx_head; return; }   /* chua co may chu: bo */
    uint16_t n = 0, t = utx_tail;
    uint8_t *c = chunk[chunk_idx];
    while (t != utx_head && n < CHUNK) {
        c[n++] = utx[t];
        t = (uint16_t)((t + 1u) & (UTX_SIZE - 1u));
    }
    if (n == 0) return;
    if (CDC_Transmit_FS(c, n) == USBD_OK) {   /* BUSY: giu nguyen, lan sau gui lai */
        utx_tail = t;
        chunk_idx ^= 1u;
    }
}

static void usb_write(const uint8_t *d, uint16_t len, uint8_t flush_now)
{
    if (!link_usb_connected()) return;
    if ((uint16_t)(UTX_SIZE - 1u - utx_used()) < len) { utx_dropped++; return; }  /* bo ca goi, khong gui nua goi */
    for (uint16_t i = 0; i < len; i++) {
        utx[utx_head] = d[i];
        utx_head = (uint16_t)((utx_head + 1u) & (UTX_SIZE - 1u));
    }
    if (flush_now || utx_used() >= CHUNK) usb_flush();
}
#else
void link_usb_on_rx(const uint8_t *buf, uint32_t len) { (void)buf; (void)len; }
int  link_usb_connected(void) { return 0; }
uint32_t link_usb_tx_dropped(void) { return 0; }
static int  usb_get(uint8_t *b) { (void)b; return 0; }
static void usb_flush(void) { }
static void usb_write(const uint8_t *d, uint16_t len, uint8_t f) { (void)d; (void)len; (void)f; }
#endif

/* ===================== UART ===================== */
static void uart_write(const uint8_t *d, uint16_t len)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)d, len, 10);
}

/* ===================== CHUNG ===================== */
void link_init(void)
{
    uart_rx_init();
}

void link_task(void)
{
    usb_flush();
}

int link_get(uint8_t *b, uint8_t *src)
{
    if (usb_get(b))      { cur_src = LINK_USB;  if (src) *src = LINK_USB;  return 1; }
    if (uart_rx_get(b))  { cur_src = LINK_UART; if (src) *src = LINK_UART; return 1; }
    return 0;
}

int link_get_ros(uint8_t *b)
{
    uint8_t x;
    if (ros_src == LINK_USB) {
        while (uart_rx_get(&x)) { }          /* bo byte console tren UART trong luc ROS chay */
        return usb_get(b);
    }
    while (usb_get(&x)) { }
    return uart_rx_get(b);
}

int link_any_key(void)
{
    uint8_t b;
    usb_flush();
    return link_get(&b, NULL);
}

void link_flush_input(void)
{
    uint8_t b;
    while (link_get(&b, NULL)) { }
}

void link_ros_bind(void)      { ros_src = cur_src; }
uint8_t link_ros_src(void)    { return ros_src; }

void link_ros_write(const uint8_t *d, uint16_t len)
{
    if (ros_src == LINK_USB) usb_write(d, len, 1);
    else                     uart_write(d, len);
}

void link_console_putc(uint8_t c)
{
    if (ros_link_active()) {
        /* Duong ROS chi chua goi nhi phan: chi in ra duong con lai */
        if (ros_src == LINK_USB) uart_write(&c, 1);
        else                     usb_write(&c, 1, (uint8_t)(c == '\n'));
        return;
    }
    usb_write(&c, 1, (uint8_t)(c == '\n'));
    uart_write(&c, 1);
}

void link_test_write(const uint8_t *d, uint16_t len)
{
    usb_write(d, len, 1);
    uart_write(d, len);
}
