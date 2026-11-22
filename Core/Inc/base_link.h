#ifndef BASE_LINK_H
#define BASE_LINK_H

#include <stdint.h>

/* GIAO TIEP XE 4 BANH VOI RASPBERRY PI, chung khung nhi phan va cong voi ros_link (USB CDC / USART1).
 *
 * Pi gui toc do TUNG BANH (dong hoc Mecanum tinh tren Pi bang mecanum_drive_controller).
 * STM32 chay PID toc do 4 banh (motor.c, 200 Hz) va tra ve so xung encoder + toc do do duoc.
 *
 * QUY UOC
 *   - Thu tu banh: 0 FL, 1 FR, 2 RL, 3 RR (trung mot_id_t).
 *   - Duong = banh lan VE TRUOC, ca 4 banh (dau rieng tung motor/encoder da xu ly trong motor.h).
 *     Trung voi truc +Y cua 4 khop banh trong URDF.
 *   - Toc do: int16, don vi mrad/s (0.001 rad/s) TAI TRUC BANH.
 *   - Vi tri: int32, so xung encoder cong don, BASE_COUNTS_PER_REV xung / vong banh.
 *
 * LENH Pi -> STM32 (khung | 0xAA | 0x55 | TYPE | LEN | PAYLOAD | CRC16 | nhu ros_link.h)
 *   0x10 CMD_BASE_VEL     seq u16 | w 4 x i16 (mrad/s)                                10 byte
 *                         Chi co tac dung khi da BAT (CMD_BASE_ENABLE 1).
 *                         Neu co banh vuot BASE_RPM_LIMIT: thu nho DEU ca 4 banh (giu huong chuyen dong),
 *                         bao bit SATURATED.
 *                         Toc do khong doi ngay ma tang / giam dan voi gia toc BASE_ACC_RPM_S,
 *                         4 banh cung ti le nen xe van di dung huong trong luc tang / giam toc.
 *   0x11 CMD_BASE_ENABLE  on u8. 1 = bat, dong thoi XOA loi ket truc / nguoc encoder va co ESTOP.    1 byte
 *                         0 = dung 4 banh va tat.
 *   0x12 CMD_BASE_PING    rong. Giu che do ROS khi chua chay (khong lam tay may bi doc CAN).      0 byte
 *   0x03 CMD_ESTOP        (chung voi tay may) dung ca 4 banh, TAT, bao bit ESTOP.
 *
 * TRA LOI STM32 -> Pi, moi BASE_CYCLE_MS khi dang o che do ROS (ke ca khi chua bat):
 *   0x90 STATE_BASE       seq u16 | counts 4 x i32 | w 4 x i16 (mrad/s) | status u16          28 byte
 *                         seq = seq cua CMD_BASE_VEL gan nhat.
 *
 * AN TOAN
 *   - Watchdog: dang bat ma qua BASE_WATCHDOG_MS khong co CMD_BASE_VEL -> giam toc ve 0 ca 4 banh,
 *     bao bit WATCHDOG. Van giu BAT: lenh toc do tiep theo chay lai binh thuong.
 *   - CMD_ESTOP, tat, thoat che do ROS: dung NGAY, khong qua bo giam toc.
 *   - Pi im lang qua 1 s (ros_link): thoat che do ROS, dung 4 banh, TAT.
 *   - Moi lenh I2C xuong module MKE-M18 kem thoi gian tu dung 100 ms (motor.c): STM32 treo thi motor tu dung.
 *
 * TAY MAY: STM32 chi doc vi tri tay may qua CAN sau khi Pi gui goi cua tay may (0x01..0x07).
 * Node chi chay xe (chi gui 0x10..0x12) khong lam STM32 doc CAN, nen test xe duoc khi tay may chua cap nguon.
 */

#define T_CMD_BASE_VEL      0x10u
#define T_CMD_BASE_ENABLE   0x11u
#define T_CMD_BASE_PING     0x12u
#define T_STATE_BASE        0x90u

#define LEN_CMD_BASE_VEL    10u
#define LEN_STATE_BASE      28u

#define BASE_CYCLE_MS       20u      /* gui STATE_BASE 50 Hz */
#define BASE_WATCHDOG_MS    200u

/* TOC DO VUA PHAI: xe 7.5 kg (tay may + pin), uu tien chinh xac, khong truot.
 * Cung muc voi lenh console "xe v" (MEC_VEL_RPM_MAX) va "xe tien" (MEC_MOVE_ACC) trong mecanum.h.
 *   25 RPM = 2.62 rad/s banh = 0.105 m/s (banh 80 mm). Xoay tai cho toi da 0.46 rad/s (k = 0.229 m).
 *   30 RPM/s: tu dung yen len 25 RPM mat 0.83 s, gia toc xe 0.126 m/s2. Dung tu 0.105 m/s di them ~4 cm. */
#define BASE_RPM_LIMIT      25.0f    /* toc do banh toi da, RPM. Tran cung cua motor.c van la MOT_RPM_MAX 40 */
#define BASE_ACC_RPM_S      30.0f    /* gia toc banh toi da, RPM/s, ap cho ca tang va giam toc */
#define BASE_COUNTS_PER_REV 7392     /* 11 xung x 4 canh x hop so 168 (motor.h) */

/* Bit cua truong status trong STATE_BASE */
#define BASE_ST_ENABLED     (1u << 0)   /* dang nhan lenh toc do */
#define BASE_ST_WATCHDOG    (1u << 1)   /* qua BASE_WATCHDOG_MS khong co lenh, da dat toc do 0 */
#define BASE_ST_FAULT_FL    (1u << 2)   /* banh FL bi ngat (ket truc hoac encoder nguoc chieu) */
#define BASE_ST_FAULT_FR    (1u << 3)
#define BASE_ST_FAULT_RL    (1u << 4)
#define BASE_ST_FAULT_RR    (1u << 5)
#define BASE_ST_SATURATED   (1u << 6)   /* lenh gan nhat vuot gioi han, da thu nho deu */
#define BASE_ST_I2C_ERROR   (1u << 7)   /* trong chu ky vua qua co loi I2C (NACK / loi bus) */
#define BASE_ST_ESTOP       (1u << 8)   /* da nhan CMD_ESTOP, can CMD_BASE_ENABLE 1 de chay lai */

/* Goi tu ros_link.c */
void base_link_reset(void);                                      /* vao che do ROS: dung xe, TAT */
int  base_link_handle(uint8_t type, const uint8_t *p, uint8_t len);  /* 1 = goi cua xe, da xu ly */
void base_link_task(void);                                       /* moi vong lap khi o che do ROS */
void base_link_estop(void);                                      /* CMD_ESTOP */
void base_link_stop(void);                                       /* thoat che do ROS */

#endif /* BASE_LINK_H */
