#ifndef ZDT_X42S_H
#define ZDT_X42S_H

#include <stdint.h>

/* Dieu khien driver ZDT X42S V2 (FIRMWARE X) qua CAN, checksum co dinh 0x6B.
 * Tham khao: ZDT_X42S user manual V1.0.5, chuong 4.2 (CAN) va chuong 5 (lenh).
 *
 * Gia tri tra ve cua cac ham:
 *   ZDT_OK         driver tra loi 02 (lenh dung)
 *   ZDT_ERR_PARAM  driver tra loi E2 (sai tham so, hoac dang bao ve dong/qua nhiet/ket truc)
 *   ZDT_ERR_FORMAT driver tra loi EE (sai dinh dang lenh)
 *   ZDT_ERR_TIMEOUT khong co tra loi trong thoi gian cho
 *   ZDT_ERR_CAN    loi gui CAN
 */
typedef enum {
    ZDT_OK = 0,
    ZDT_ERR_PARAM,
    ZDT_ERR_FORMAT,
    ZDT_ERR_TIMEOUT,
    ZDT_ERR_CAN,
    ZDT_ERR_OTHER,
} zdt_status_t;

/* Kieu chay vi tri (byte "运动模式" cua lenh FD) */
typedef enum {
    ZDT_MOVE_REL_TARGET  = 0,   /* tuong doi so voi vi tri DICH truoc do (khong tich luy sai so) */
    ZDT_MOVE_ABSOLUTE    = 1,   /* tuyet doi so voi diem 0 */
    ZDT_MOVE_REL_CURRENT = 2,   /* tuong doi so voi vi tri THUC hien tai */
} zdt_move_mode_t;

/* Co trang thai (lenh 3A) */
#define ZDT_FLAG_ENABLED     (1u << 0)   /* Ens_TF: dang enable */
#define ZDT_FLAG_REACHED     (1u << 1)   /* Prf_TF: da toi vi tri dich */
#define ZDT_FLAG_STALL       (1u << 2)   /* Cgi_TF: dang ket truc */
#define ZDT_FLAG_STALL_PROT  (1u << 3)   /* Cgp_TF: da kich hoat bao ve ket truc */

const char  *zdt_status_str(zdt_status_t s);

zdt_status_t zdt_read_version(uint8_t addr, uint16_t *fw, uint16_t *hw);
zdt_status_t zdt_enable(uint8_t addr, int on);
zdt_status_t zdt_stop(uint8_t addr);
zdt_status_t zdt_zero_position(uint8_t addr);
zdt_status_t zdt_clear_protection(uint8_t addr);

/* Xoa che do "tu chay toc do khi bat nguon" (lenh F7 1C, firmware X) */
zdt_status_t zdt_clear_autorun(uint8_t addr);

/* Che do toc do (lenh F6, firmware X). rpm_x10 am = quay nguoc, 0 = dung */
zdt_status_t zdt_speed(uint8_t addr, int32_t rpm_x10, uint16_t acc);

/* Doc co he thong (lenh 3B): bit0 Enc_Rdy, bit1 Cal_Rdy, bit4 qua nhiet, bit5 qua dong */
zdt_status_t zdt_read_home_flags(uint8_t addr, uint8_t *flags);

/* Doc lenh tra loi dai nhieu frame (vd 43 7A: trang thai he thong, 42 6C: cau hinh).
 * buf = [code, du lieu...], need = tong so byte can (tinh ca code, 36 cho 2 lenh tren) */
zdt_status_t zdt_read_long(uint8_t addr, uint8_t code, uint8_t aux, uint8_t *buf, uint8_t need);

/* 1 = in moi frame CAN gui/nhan ra terminal de do loi */
extern uint8_t zdt_debug;

/* Chay vi tri theo duong cong hinh thang (lenh FD, firmware X).
 *   deg_x10 : goc tinh theo 0.1 do (900 = 90.0 do), am = quay nguoc
 *   rpm_x10 : toc do toi da tinh theo 0.1 RPM (600 = 60.0 RPM), toi da 30000
 *   acc, dec: gia toc / giam toc (RPM/s), 0..65535
 */
zdt_status_t zdt_move(uint8_t addr, int32_t deg_x10, uint16_t rpm_x10,
                      uint16_t acc, uint16_t dec, zdt_move_mode_t mode);

/* Nhu zdt_move nhung chon co dong bo: sync = 1 thi driver nhan lenh nhung CHUA chay,
 * cho toi khi goi zdt_sync_trigger() (dung de nhieu khop bat dau cung luc) */
zdt_status_t zdt_move_ex(uint8_t addr, int32_t deg_x10, uint16_t rpm_x10,
                         uint16_t acc, uint16_t dec, zdt_move_mode_t mode, uint8_t sync);

/* Kich hoat tat ca lenh dang cho dong bo (gui 00 FF 66 6B qua dia chi quang ba) */
zdt_status_t zdt_sync_trigger(void);

/* Doc vi tri thuc (lenh 36), ket qua theo 0.1 do */
zdt_status_t zdt_read_position(uint8_t addr, int32_t *deg_x10);

/* Doc toc do thuc (lenh 35), ket qua theo 0.1 RPM */
zdt_status_t zdt_read_speed(uint8_t addr, int32_t *rpm_x10);

/* Doc co trang thai (lenh 3A) */
zdt_status_t zdt_read_flags(uint8_t addr, uint8_t *flags);

/* Tham so vong dieu khien firmware X (manual 5.6.14 / 5.6.15).
 * Mac dinh X42S: pos_kp_trap = pos_kp_direct = 126640, vel_kp = 15600, vel_ki = 26 */
typedef struct {
    uint32_t pos_kp_trap;     /* Kp vong vi tri khi chay duong cong hinh thang (lenh FD) */
    uint32_t pos_kp_direct;   /* Kp vong vi tri khi chay truc tiep co gioi han toc do */
    uint32_t vel_kp;          /* Kp vong toc do */
    uint32_t vel_ki;          /* Ki vong toc do */
} zdt_pid_t;

#define ZDT_PID_DEFAULT_POS_KP  126640u
#define ZDT_PID_DEFAULT_VEL_KP   15600u
#define ZDT_PID_DEFAULT_VEL_KI      26u
/* He so cung mac dinh, DO THUC TE tren driver chua chinh (khac so ghi trong manual):
 *   firmware 204: 384 (manual ghi 388)     firmware 206: 1560 */
#define ZDT_STIFF_DEFAULT_FW204    384u
#define ZDT_STIFF_DEFAULT_FW206   1560u

/* store = 0: chi dung toi khi tat nguon driver. store = 1: luu vinh vien */
zdt_status_t zdt_read_pid(uint8_t addr, zdt_pid_t *p);
zdt_status_t zdt_write_pid(uint8_t addr, const zdt_pid_t *p, uint8_t store);

/* He so cung (manual 5.6.26 / 5.6.27). Lon hon = cung hon, qua lon dong co rung */
zdt_status_t zdt_read_stiffness(uint8_t addr, uint32_t *v);
zdt_status_t zdt_write_stiffness(uint8_t addr, uint32_t v, uint8_t store);

/* Goi thuong xuyen trong vong lap chinh: xu ly tin driver tu gui
 * (vd. "FD 9F 6B" = da toi vi tri, can menu Response = Both hoac Reached).
 * Tra ve dia chi driver vua bao toi noi, 0 neu khong co. Moi lan goi tra ve 1 driver. */
uint8_t zdt_poll_reached(void);

#endif /* ZDT_X42S_H */
