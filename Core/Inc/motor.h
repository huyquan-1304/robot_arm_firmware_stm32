#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

/* 4 MOTOR BANH XE: JGB37-520 12 V 168:1 co encoder Hall, 2 module MKE-M18 (BD62130, I2C).
 *
 *   Driver  : I2C3 (PC8 SCL, PC9 SDA), 100 kHz, che do ngat
 *             module TRAI 0x40: kenh A = FL, kenh B = RL
 *             module PHAI 0x41: kenh A = FR, kenh B = RR
 *   Encoder : TIM2 FL (PA0/PA1), TIM4 FR (PD12/PD13), TIM1 RL (PC0/PC1), TIM5 RR (PB2/PC12)
 *   PID     : ngat TIM6, 200 Hz (5 ms). Gui lenh I2C 100 Hz (moi 2 chu ky PID)
 *
 * QUY UOC: so duong = banh lan VE TRUOC cho ca 4 banh. Dau rieng cua tung motor / encoder
 * dat o day (MOT_DIR_xx, MOT_ENC_xx) hoac dao tam bang lenh "m <banh> dao" / "m <banh> daoe".
 *
 * CHONG NHIEU / AN TOAN: moi lenh I2C kem thoi gian tu dung MOT_CMD_HOLD_MS. Neu STM32 treo
 * hoac bus I2C hong, module tu cat motor. Bus treo duoc tu phuc hoi (9 xung SCL). */

typedef enum { MOT_FL = 0, MOT_FR, MOT_RL, MOT_RR, MOT_COUNT } mot_id_t;

/* Dia chi I2C 2 module (dat bang lenh "m addr <cu> <moi>", mac dinh xuat xuong 0x40) */
#define MOT_I2C_ADDR_LEFT    0x40u    /* module TRAI: FL (kenh A), RL (kenh B) */
#define MOT_I2C_ADDR_RIGHT   0x41u    /* module PHAI: FR (kenh A), RR (kenh B) */
#define MOT_CMD_HOLD_MS      100u     /* module tu dung neu khong nhan lenh moi trong thoi gian nay */

/* Thong so co khi */
#define MOT_GEAR             168.0f
#define MOT_ENC_PPR          11.0f                                   /* xung / vong truc motor / kenh */
#define MOT_COUNTS_PER_REV   (MOT_ENC_PPR * 4.0f * MOT_GEAR)          /* 7392 xung / vong banh */
#define MOT_RPM_NOLOAD       60.0f                                   /* toc do khong tai o 12 V */
#define MOT_RPM_MAX          40.0f                                   /* gioi han lenh (co tai ~46 RPM) */

/* Dau mac dinh (+1 / -1). Doi sau khi test bang "m <banh> pwm 30" va "m <banh>" */
#define MOT_DIR_FL   (+1)
#define MOT_DIR_FR   (+1)
#define MOT_DIR_RL   (+1)
#define MOT_DIR_RR   (+1)
#define MOT_ENC_FL   (+1)
#define MOT_ENC_FR   (+1)
#define MOT_ENC_RL   (+1)
#define MOT_ENC_RR   (+1)

/* PID toc do mac dinh (don vi: % PWM). Chinh bang lenh "m pid" */
#define MOT_KFF_DEFAULT      (100.0f / MOT_RPM_NOLOAD)   /* % / RPM: bu truoc theo toc do */
#define MOT_KP_DEFAULT       2.0f                        /* % / RPM */
#define MOT_KI_DEFAULT       15.0f                       /* % / (RPM.s) */
#define MOT_KPOS_DEFAULT     120.0f                      /* RPM / vong sai so vi tri (che do vi tri) */

/* Bao ve: PWM > 85% ma toc do < 2 RPM lien tuc 1 s -> ngat motor */
#define MOT_STALL_DUTY       85.0f
#define MOT_STALL_RPM        2.0f
#define MOT_STALL_MS         1000u

void  motor_init(void);            /* goi 1 lan trong main, sau cac MX_xxx_Init() */
void  motor_task(void);            /* goi lien tuc (da goi trong cmd_uart_poll): thong bao, phuc hoi I2C */
void  motor_cmd(int argc, char **argv);
void  motor_print_help(void);

/* ---- API cho lop tren (mecanum, ROS) ---- */
void  mot_stop_all(void);                              /* dung ca 4 banh */
void  mot_set_rpm(mot_id_t m, float rpm);              /* che do van toc, duong = lan ve truoc */
/* Che do vi tri: quay them 'deg' do (do banh), toc do toi da rpm_max, gia toc acc RPM/s */
void  mot_move_rel(mot_id_t m, float deg, float rpm_max, float acc);
/* Bat dau nhieu banh cung luc: goi mot_group_begin(), cac mot_move_rel(), roi mot_group_go() */
void  mot_group_begin(void);
void  mot_group_go(void);
int   mot_move_done(mot_id_t m);                       /* 1 = da toi dich (hoac khong o che do vi tri) */
float mot_get_rpm(mot_id_t m);                         /* toc do do duoc */
float mot_get_deg(mot_id_t m);                         /* vi tri banh, do */
int32_t mot_get_counts(mot_id_t m);
void  mot_zero(mot_id_t m);
int   mot_fault(mot_id_t m);                           /* 1 = da ngat do ket truc */
const char *mot_name(mot_id_t m);

#endif /* MOTOR_H */
