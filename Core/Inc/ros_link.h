#ifndef ROS_LINK_H
#define ROS_LINK_H

#include <stdint.h>

/* Giao tiep nhi phan voi Raspberry Pi (node stm32_bridge_node.py) qua USART1.
 *
 * Dung chung USART1 voi console go lenh (cmd_uart):
 *   - Binh thuong: console text nhu cu.
 *   - Nhan duoc 1 goi nhi phan hop le: vao CHE DO ROS, tat moi printf, gui STATE moi 20 ms.
 *   - 1 giay khong nhan goi nao: thoat CHE DO ROS, dung dong co neu dang chay, tro lai console.
 *
 * KHUNG (little-endian):
 *   | 0xAA | 0x55 | TYPE u8 | LEN u8 | PAYLOAD (LEN byte) | CRC16 u16 |
 *   CRC16-CCITT (da thuc 0x1021, gia tri dau 0xFFFF) tinh tren TYPE + LEN + PAYLOAD.
 *
 * GOC KHOP: int32 don vi 0.1 do, theo QUY UOC CUA FIRMWARE (bang joint.c: chieu, diem 0, gioi han).
 *   Node tren Pi tu doi rad (URDF) <-> 0.1 do (firmware) bang tham so joint_sign, joint_offset_deg.
 *
 * LENH Pi -> STM32
 *   0x01 CMD_JOINT    seq u16 | goc 4 x i32 (0.1 do) | servo_us u16 (0 = giu nguyen)   20 byte
 *   0x02 CMD_ENABLE   on u8 (1 = nhan lenh chuyen dong, 0 = dung va bo qua lenh)           1 byte
 *                     Chua home du 4 khop (NOT_HOMED) thi KHONG bat
 *   0x03 CMD_ESTOP    rong. Dung 4 khop ngay (dong co VAN GIU luc, khong tha roi)           0 byte
 *   0x04 CMD_SET_ZERO mask u8 (bit0 = khop 1 ... bit3 = khop 4). Chi khi dang TAT,           1 byte
 *                     bo qua khop da home bang cam bien
 *   0x05 CMD_PING     rong. Giu ket noi khi chua bat dong co                                 0 byte
 *   0x06 CMD_HOME     rong. Home ca tay bang cam bien (giong lenh "home" tren console,       0 byte
 *                     kem mo gripper). Chi khi dang TAT. Mat khoang 30 s
 *
 * HOME BANG CAM BIEN: 2 cach
 *   - Go "home" tren console truoc khi bat node tren Pi.
 *   - Pi gui CMD_HOME. Trong luc home: bit HOMING = 1, STM32 gui STATE moi 100 ms (goc chua cap nhat),
 *     bo qua CMD_ENABLE / CMD_SET_ZERO / CMD_HOME. CMD_ESTOP huy home ngay.
 *   Pi thay NOT_HOMED = 0 va HOMING = 0 la da home du 4 khop. Home loi thi NOT_HOMED van = 1.
 *
 * TRA LOI STM32 -> Pi (moi chu ky 20 ms khi o che do ROS)
 *   0x81 STATE        seq u16 | goc thuc 4 x i32 (0.1 do) | servo_us u16 | status u16 | loop_ms u8
 *                     = 23 byte. seq = seq cua CMD_JOINT gan nhat.
 */

/* Bit cua truong status */
#define ROS_ST_ENABLED      (1u << 0)   /* dang nhan lenh chuyen dong */
#define ROS_ST_WATCHDOG     (1u << 1)   /* qua ROS_WATCHDOG_MS khong co CMD_JOINT, da tu dung */
#define ROS_ST_CAN_ERROR    (1u << 2)   /* co driver khong tra loi khi doc vi tri */
#define ROS_ST_STALL_J1     (1u << 3)   /* ket truc / bao ve ket truc khop 1 .. 4 */
#define ROS_ST_STALL_J2     (1u << 4)
#define ROS_ST_STALL_J3     (1u << 5)
#define ROS_ST_STALL_J4     (1u << 6)
#define ROS_ST_NOT_HOMED    (1u << 7)   /* con khop chua home bang cam bien tiem can (home.c) */
#define ROS_ST_LIMIT        (1u << 8)   /* co lenh ngoai gioi han khop, lenh do bi bo */
#define ROS_ST_ESTOP        (1u << 9)   /* da nhan CMD_ESTOP */
#define ROS_ST_HOMING       (1u << 10)  /* dang home theo CMD_HOME */

void    ros_link_init(void);

/* Dua tung byte nhan duoc vao. Tra ve 1 neu byte thuoc goi nhi phan (console bo qua byte nay) */
int     ros_link_feed(uint8_t c);

/* Goi lien tuc trong while(1): watchdog, gui lenh xuong driver, doc vi tri, gui STATE */
void    ros_link_task(void);

/* 1 = dang o che do ROS, console khong duoc printf */
uint8_t ros_link_active(void);

/* Goi lien tuc trong luc home do Pi yeu cau (qua ham huy cua home.c): nhan va xu ly goi tu Pi,
 * gui STATE moi 100 ms. Tra ve 1 neu Pi gui CMD_ESTOP (huy home). */
int     ros_link_poll_blocking(void);

#endif /* ROS_LINK_H */
