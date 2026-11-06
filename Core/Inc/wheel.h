#ifndef WHEEL_H
#define WHEEL_H

#include <stdint.h>
#include "zdt_x42s.h"

/* BANH XE MECANUM: driver ZDT M370 + dong co JGB37-520 ti so 1:131.
 * Cam chung bus FDCAN1 voi tay may (cung hub ZDT). Giao thuc M370 giong X42S
 * (ID CAN = dia chi << 8, checksum co dinh 0x6B) nen dung lai toan bo zdt_x42s.c.
 *
 * GIAI DOAN 1: test 1 banh qua console bang lenh "bx ...". Go "h" de xem danh sach.
 *
 * QUY UOC DIA CHI (tay may dang dung 1..4, banh xe KHONG duoc trung):
 *   5 = FR truoc-phai   6 = FL truoc-trai   7 = RL sau-trai   8 = RR sau-phai
 *   (theo bang test 04/11, xem mecanum.h)
 *
 * Sau khi dat ti so truyen = 131 tren driver, M370 tu quy doi toc do va vi tri ra TRUC RA
 * (manual M370 muc 5.5.7). Moi so RPM / do trong file nay la cua BANH XE, khong phai cua dong co.
 *
 * Driver M370 phai dat tren man OLED truoc khi test:
 *   P_Serial = CAN1_MAP, CAN_Baud = 500000, ID_Addr = 5..8, Response = Receive (mac dinh)
 *   Cal: chay 1 lan khi banh khong cham dat. */

#define WHEEL_TEST_ADDR      5u        /* dia chi mac dinh cua banh dang test */
#define WHEEL_GEAR_X100      13100u    /* ti so truyen 1:131, don vi 0.01 */
#define WHEEL_RADIUS_MM      37.5f     /* banh mecanum duong kinh 75 mm */
#define WHEEL_LX_MM          85.0f     /* nua chieu dai co so: tam xe toi truc banh truoc/sau (dung cho 4 banh) */
#define WHEEL_LY_MM          144.0f    /* nua vet banh: tam xe toi mat phang banh trai/phai (dung cho 4 banh) */

#define WHEEL_MAX_RPM_X10    300       /* gioi han 30.0 RPM tai banh (~118 mm/s). Do thuc te toi da 33.7 RPM */
#define WHEEL_ACC_DEFAULT    100u      /* gia toc mac dinh, RPM/s */
#define WHEEL_RUN_S_DEFAULT  5u        /* "bx v" tu dung sau 5 s neu khong ghi so giay */
#define WHEEL_TURN_RPM_X10   300       /* "bx q": quay theo vong o 30.0 RPM */
#define WHEEL_TURN_MAX       20        /* "bx q": toi da 20 vong moi lenh */

/* Goi lien tuc trong vong lap (da goi trong cmd_uart_poll): tu dung banh khi het gio */
void wheel_task(void);

/* Xu ly lenh console "bx ..." (argv[0] = "bx") */
void wheel_cmd(int argc, char **argv);

/* Huy hen gio tu dung cua lenh "bx v" (khi chuyen sang lenh "xe") */
void wheel_cancel_timer(void);

/* In cac dong huong dan lenh "bx" (goi trong print_help) */
void wheel_print_help(void);

/* Dat toc do 1 banh, 0.1 RPM tai banh, am = quay nguoc. Tu cat theo WHEEL_MAX_RPM_X10.
 * Giai doan 4 banh se dung lai ham nay. */
zdt_status_t wheel_set_rpm_x10(uint8_t addr, int32_t rpm_x10);

#endif /* WHEEL_H */
