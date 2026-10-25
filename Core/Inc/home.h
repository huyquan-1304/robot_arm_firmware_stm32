#ifndef HOME_H
#define HOME_H

#include <stdint.h>
#include "joint.h"

/* Cam bien hanh trinh (tiem can NPN NC) doc bang GPIO cua STM32 va quy trinh ve goc (homing).
 *
 * Quy uoc tin hieu: 1 = DANG TAC DONG (co kim loai truoc dau cam bien HOAC dut day), 0 = tu do.
 *
 * Quy trinh home 1 khop: GAP CAM BIEN LA DUNG NGAY.
 *   - Dang dung tren kim loai: nhich cham ra toi khi cam bien vua nha (mep gan) roi dung
 *   - Khong: chay theo chieu dir, gap cam bien thi dung (mep gan)
 *   - Khong thay trong travel: chay nguoc, gap cam bien thi dung (mep xa, cong them band_x10)
 *   - Tinh offset tu vi tri dong co doc ngay luc gap
 * Bam bat ky phim nao tren terminal trong luc home = dung ngay.
 */

typedef enum {
    HOME_OK = 0,
    HOME_ERR_ARG,          /* so khop sai */
    HOME_ERR_NOT_CONFIG,   /* khop chua bat home trong bang (enabled = 0) */
    HOME_ERR_DRIVER,       /* driver khong tra loi / tu choi lenh */
    HOME_ERR_NOT_FOUND,    /* chay het quang duong cho phep ma khong thay cam bien */
    HOME_ERR_STUCK_ON,     /* lui ra roi ma cam bien van tac dong: dut day, sai cuc tinh, lap sai */
    HOME_ERR_STALL,        /* dong co duoc ra lenh chay nhung khong nhuc nhich: bi ket co khi */
    HOME_ERR_TIMEOUT,
    HOME_ERR_ABORT,        /* nguoi dung bam phim */
} home_status_t;

/* Ham tra ve 1 neu nguoi dung muon huy (vd co phim bam tren UART) */
typedef int (*home_abort_fn)(void);

/* Cau hinh GPIO cho 4 cam bien. Goi 1 lan sau khi HAL da khoi dong */
void home_init(home_abort_fn abort_fn);

/* Trang thai cam bien khop n (1..4) da loc nhieu (3 mau lien tiep, 1 ms/mau) */
uint8_t home_sensor(uint8_t n);
/* Muc logic tho tren chan GPIO (chua dao cuc tinh, chua loc), de do loi day */
uint8_t home_pin_level(uint8_t n);
/* Goi thuong xuyen (moi vong lap) de bo loc cap nhat */
void    home_sensor_update(void);

uint8_t home_is_homed(uint8_t n);

/* Home 1 khop: dat offset de goc khop tai cam bien = home_x10 trong bang */
home_status_t home_joint(uint8_t n);

/* Home tat ca khop co enabled = 1 theo thu tu HOME_ORDER trong home.c */
home_status_t home_all(void);

/* Do goc mep gan (home_x10) va be rong vung kim loai (band_x10), theo goc 0 HIEN TAI. KHONG doi offset.
 * Khop phai dang o phia binh thuong cua cam bien (ngoai vung kim loai). Chay duoc ca khi enabled = 0.
 * dir_override: +1 / -1 = dung chieu nay thay cho dir trong bang, 0 = dung dir trong bang. */
home_status_t home_measure(uint8_t n, int8_t dir_override, int32_t *home_x10, int32_t *band_x10);

/* Chay khop n theo chieu goc khop joint_dir (+1 tang, -1 giam) voi toc do tim nhanh trong bang,
 * DUNG NGAY khi cam bien tac dong, tra ve goc khop luc cham. Khong can dien dir/enabled truoc.
 * Van gioi han quang duong travel_x10 va bam phim de huy. Khong doi offset. */
home_status_t home_jog_to_sensor(uint8_t n, int8_t joint_dir, int32_t *joint_x10);

const char *home_status_str(home_status_t s);

#endif /* HOME_H */
