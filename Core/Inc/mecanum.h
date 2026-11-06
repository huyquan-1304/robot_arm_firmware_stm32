#ifndef MECANUM_H
#define MECANUM_H

#include <stdint.h>
#include "zdt_x42s.h"

/* XE 4 BANH MECANUM: 4 driver ZDT M370 tren FDCAN1, lenh console "xe ...".
 *
 * DONG HOC NGHICH (tai lieu dong hoc cua project, pt 2.20), L = lx + ly:
 *   FL = vx - vy - L*wz      FR = vx + vy + L*wz
 *   RL = vx + vy - L*wz      RR = vx - vy + L*wz
 *   vx > 0 = tien, vy > 0 = sang TRAI, wz > 0 = quay nguoc chieu kim (nhin tu tren xuong)
 *   Ket qua la toc do / quang duong tai vanh banh, "duong = day xe di toi" cho ca 4 banh.
 *
 * DAU (SIGN): lenh gui driver = SIGN x gia tri tren. Theo bang test: ca 4 banh lan ve truoc
 * khi driver quay CCW (lenh am), nen SIGN = -1. Banh nao chay nguoc thi doi dau o day,
 * hoac dao tam thoi bang lenh "xe dao <dia chi>". */

/* Dia chi driver theo vi tri (theo bang test ngay 04/11) */
#define MEC_ADDR_FL   6u
#define MEC_ADDR_FR   5u
#define MEC_ADDR_RL   7u
#define MEC_ADDR_RR   8u

#define MEC_SIGN_FL   (-1)
#define MEC_SIGN_FR   (-1)
#define MEC_SIGN_RL   (-1)
#define MEC_SIGN_RR   (-1)

/* Chay theo quang duong / goc (che do vi tri, 4 banh bat dau va toi noi cung luc) */
#define MEC_MOVE_RPM_X10   200    /* toc do banh chay nhieu nhat: 20.0 RPM = 78.5 mm/s tai vanh */
#define MEC_MOVE_ACC       60u    /* gia toc banh chay nhieu nhat, RPM/s */
#define MEC_DIST_DEFAULT   300    /* mm, khi khong ghi quang duong */

/* He so hieu chinh goc xoay: banh mecanum truot nen xe thuong xoay thieu.
 * Lenh 90 do ma xe xoay 85 do thi dat 90/85 = 1.059 */
#define MEC_ROT_GAIN       1.00f

/* Che do van toc "xe v" tu dung sau so giay nay neu khong ghi */
#define MEC_VEL_S_DEFAULT  3u

void mecanum_task(void);                       /* goi lien tuc (da goi trong cmd_uart_poll) */
void mecanum_cmd(int argc, char **argv);       /* lenh console "xe ..." */
void mecanum_print_help(void);

/* Dat van toc than xe: vx, vy mm/s, wz rad/s. Tu thu nho deu ca 4 banh neu vuot gioi han.
 * Dung lai cho giao tiep ROS sau nay. Tra ve ZDT_OK neu ca 4 banh nhan lenh */
zdt_status_t mecanum_set_velocity(float vx, float vy, float wz);

/* Dung ca 4 banh ngay (lenh FE) */
void mecanum_stop(void);

#endif /* MECANUM_H */
