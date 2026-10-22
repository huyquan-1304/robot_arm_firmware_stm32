#ifndef JOINT_H
#define JOINT_H

#include <stdint.h>
#include "zdt_x42s.h"

/* Lop KHOP: lam viec theo goc o DAU RA cua khop (sau hop so), don vi 0.1 do.
 * Tu quy doi sang goc truc dong co: goc_dong_co = offset + huong * ti_so * goc_khop
 *
 * Khop n (1..4) dung driver co ID_Addr = n.
 * Diem 0: hien tai = vi tri luc bat nguon (driver tu lay lam 0).
 *         Sau nay them cong tac hanh trinh: joint_home() se chay ve cong tac roi dat lai 0.
 */

#define JOINT_COUNT 4

typedef struct {
    uint8_t  addr;        /* ID_Addr cua driver */
    uint16_t ratio;       /* ti so truyen: so vong dong co / 1 vong khop (4, 27, 14, 1) */
    int8_t   dir;         /* +1 hoac -1: dao chieu neu ra lenh goc duong ma khop quay am */
    int32_t  min_x10;     /* gioi han goc khop, 0.1 do */
    int32_t  max_x10;
    uint16_t dps_x10;     /* toc do toi da cua KHOP, 0.1 do/giay */
    uint16_t acc_x10;     /* gia toc cua KHOP, 0.1 do/giay^2 (cang nho khoi dong cang em) */
    int32_t  offset_x10;  /* goc dong co (0.1 do) ung voi goc khop = 0. Dung cho home sau nay */
    int32_t  target_x10;  /* goc khop dich gan nhat da ra lenh */
} joint_t;

typedef enum {
    JOINT_OK = 0,
    JOINT_ERR_ARG,        /* so khop sai */
    JOINT_ERR_LIMIT,      /* goc nam ngoai gioi han */
    JOINT_ERR_DRIVER,     /* driver tra loi loi, xem joint_last_driver_error() */
} joint_status_t;

void           joint_init(void);
joint_t       *joint_get(uint8_t n);                 /* n = 1..4, NULL neu sai */
const char    *joint_status_str(joint_status_t s);
zdt_status_t   joint_last_driver_error(void);

/* Chay 1 khop toi goc tuyet doi (0.1 do), dung toc do/gia toc rieng cua khop */
joint_status_t joint_move(uint8_t n, int32_t deg_x10);

/* Chay 1 khop them mot goan so voi dich truoc do (0.1 do), van kiem tra gioi han */
joint_status_t joint_move_rel(uint8_t n, int32_t delta_x10);

/* Chay ca 4 khop cung luc toi 4 goc, tu chinh toc do de TAT CA BAT DAU VA TOI NOI CUNG LUC.
 * Khop nao co toc do toi da thap (vd khop 2) se quyet dinh thoi gian chung. */
joint_status_t joint_move_all(const int32_t deg_x10[JOINT_COUNT]);

/* Doc goc thuc cua khop (0.1 do) tu encoder */
joint_status_t joint_read(uint8_t n, int32_t *deg_x10);

/* Dat toc do (0.1 do/s) va gia toc (0.1 do/s^2) cho khop. acc_x10 = 0 thi giu nguyen */
joint_status_t joint_set_speed(uint8_t n, uint16_t dps_x10, uint16_t acc_x10);

/* Lay vi tri hien tai cua khop lam goc 0 */
joint_status_t joint_zero_here(uint8_t n);

/* n = 0: tat ca khop */
joint_status_t joint_enable(uint8_t n, int on);
void           joint_stop_all(void);

/* Du phong cho cong tac hanh trinh, chua lam */
joint_status_t joint_home(uint8_t n);

#endif /* JOINT_H */
