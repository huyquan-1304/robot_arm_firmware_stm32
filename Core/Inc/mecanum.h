#ifndef MECANUM_H
#define MECANUM_H

#include <stdint.h>

/* XE 4 BANH MECANUM tren lop motor.c (TB6612 + encoder), lenh console "xe ...".
 *
 * DONG HOC NGHICH (tai lieu dong hoc cua project, pt 2.20), L = lx + ly:
 *   FL = vx - vy - L*wz      FR = vx + vy + L*wz
 *   RL = vx + vy - L*wz      RR = vx - vy + L*wz
 *   vx > 0 = tien, vy > 0 = sang TRAI, wz > 0 = quay nguoc chieu kim (nhin tu tren xuong)
 *   Ket qua la toc do / quang duong tai vanh banh, duong = banh lan ve truoc
 *   (dau rieng tung motor da xu ly trong motor.c). */

#define MEC_RADIUS_MM      37.5f     /* banh mecanum 75 mm */
#define MEC_LX_MM          85.0f     /* nua chieu dai co so: tam xe toi truc banh truoc/sau */
#define MEC_LY_MM          144.0f    /* nua vet banh: tam xe toi mat phang banh trai/phai */

/* Chay theo quang duong / goc (4 banh bat dau va toi noi cung luc) */
#define MEC_MOVE_RPM       25.0f     /* toc do banh chay nhieu nhat, RPM (~98 mm/s tai vanh) */
#define MEC_MOVE_ACC       40.0f     /* gia toc banh chay nhieu nhat, RPM/s */
#define MEC_DIST_DEFAULT   300       /* mm, khi khong ghi quang duong */

/* He so hieu chinh goc xoay: banh mecanum truot nen xe thuong xoay thieu.
 * Lenh 90 do ma xe xoay 85 do thi dat 90/85 = 1.059 */
#define MEC_ROT_GAIN       1.00f

/* Che do van toc "xe v" tu dung sau so giay nay neu khong ghi */
#define MEC_VEL_S_DEFAULT  3u

void mecanum_task(void);                       /* goi lien tuc (da goi trong cmd_uart_poll) */
void mecanum_cmd(int argc, char **argv);       /* lenh console "xe ..." */
void mecanum_print_help(void);

/* Dat van toc than xe: vx, vy mm/s, wz rad/s. Tu thu nho deu ca 4 banh neu vuot gioi han.
 * Dung lai cho giao tiep ROS sau nay */
void mecanum_set_velocity(float vx, float vy, float wz);

/* Dung ca 4 banh ngay */
void mecanum_stop(void);

#endif /* MECANUM_H */
