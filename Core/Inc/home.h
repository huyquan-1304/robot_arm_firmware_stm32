#ifndef HOME_H
#define HOME_H

#include <stdint.h>
#include "joint.h"

/* HOME BANG DRIVER ZDT (che do EndStop, manual X42S muc 3.2.4).
 *
 * Cam bien hanh trinh noi vao chan En cua driver tung khop (menu P_Pul = ESI_RCO).
 * Chan En xuong 0 V = cam bien tac dong. STM32 KHONG doc GPIO nua, trang thai cam bien
 * doc qua CAN bang lenh 3D (chi doc khi can, khong doc lien tuc de khong chiem bus CAN).
 *
 * Quy trinh home 1 khop:
 *   1. Dang dung tren cam bien: STM32 nhich cham ra ngoai (nguoc chieu dir) roi lui them 2 do
 *   2. Dat thong so home cho driver (lenh 4C, khong luu) va ra lenh home (lenh 9A 03)
 *   3. Driver chay theo chieu dir toi khi chan En = 0 V thi dung. STM32 doc co 3B moi 30 ms,
 *      theo doi quang duong, bam phim / ESTOP thi gui 9C de dung
 *   4. Khong thay trong travel: home nguoc chieu (gap mep xa, cong them band_x10)
 *   5. Dat offset tu vi tri luc driver dung, dua khop ve goc 0 (GIUA vung kim loai)
 */

typedef enum {
    HOME_OK = 0,
    HOME_ERR_ARG,          /* so khop sai */
    HOME_ERR_NOT_CONFIG,   /* en = 0 hoac chua do cw_pos (lenh hcw) */
    HOME_ERR_DRIVER,       /* driver khong tra loi / tu choi lenh */
    HOME_ERR_NOT_FOUND,    /* chay het quang duong / het thoi gian ma khong thay cam bien */
    HOME_ERR_STUCK_ON,     /* cam bien luon tac dong: sai loai cam bien (NC noi thang), dut day, lap sai */
    HOME_ERR_STALL,        /* dong co duoc ra lenh chay nhung khong nhuc nhich */
    HOME_ERR_TIMEOUT,
    HOME_ERR_ABORT,        /* nguoi dung bam phim / Pi gui ESTOP */
} home_status_t;

/* Ham tra ve 1 neu nguoi dung muon huy */
typedef int (*home_abort_fn)(void);

void home_init(home_abort_fn abort_fn);

/* Trang thai cam bien khop n (1..4), doc qua CAN 2 lan cach 2 ms. 1 = tac dong. Loi CAN tra ve 0 */
uint8_t home_sensor(uint8_t n);
/* Muc tho cua chan En (1 = cao), de do loi day */
uint8_t home_pin_level(uint8_t n);
/* Giu lai cho tuong thich, KHONG lam gi (khong doc CAN lien tuc) */
void    home_sensor_update(void);

uint8_t home_is_homed(uint8_t n);

home_status_t home_joint(uint8_t n);
home_status_t home_all(void);

/* Do bang chinh quy trinh home cua driver, KHONG phu thuoc goc 0 hien tai, KHONG doi offset:
 *   home_x10 = goc khop tai cho driver dung, tinh tu GIUA vung kim loai (goc 0 moi)
 *   band_x10 = be rong vung kim loai
 * dir_override: +1 / -1 = dung chieu nay, 0 = dung dir trong bang */
home_status_t home_measure(uint8_t n, int8_t dir_override, int32_t *home_x10, int32_t *band_x10);

/* Chay khop theo chieu joint_dir bang STM32, dung khi cam bien tac dong (lenh lsj) */
home_status_t home_jog_to_sensor(uint8_t n, int8_t joint_dir, int32_t *joint_x10);

/* Thu chieu CW cua driver: ra lenh home CW trong 0.5 s roi dung.
 * *cw_pos = +1 neu CW lam vi tri dong co TANG, -1 neu GIAM. Dien vao cot cw_pos (lenh hcw) */
home_status_t home_probe_cw(uint8_t n, int8_t *cw_pos);

const char *home_status_str(home_status_t s);

#endif /* HOME_H */
