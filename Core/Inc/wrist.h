#ifndef WRIST_H
#define WRIST_H

#include <stdint.h>

/* KHOP 5: servo MG90S xoay gripper, TIM3 CH2 (cung timer 50 Hz voi gripper TIM3 CH1).
 * CubeMX: TIM3 -> Channel2 = PWM Generation CH2, chan PA7 (hoac PC7). KHONG dung PB5.
 *
 * Goc khop 5 tinh theo 0.1 do. Goc 0 = gripper song song mat phang canh tay (quy uoc khi lap).
 * Doi goc ra xung theo 3 diem hieu chuan trong wrist.c (-90, 0, +90 do), noi suy tuyen tinh tung nua.
 * Servo co bien tro ben trong nen biet goc tuyet doi: KHONG can cam bien, KHONG can home.
 * Luc bat nguon CHUA phat xung (servo tha long). Chi phat xung khi co lenh goc dau tien. */

void     wrist_start(void);                 /* goi 1 lan sau MX_TIM3_Init, sau servo_start */
void     wrist_update(void);                /* goi lien tuc: chay cham toi dich */

/* Chay cham toi goc deg_x10 voi toc do WRIST_DPS. Tra ve 0 neu ngoai gioi han (khong chay) */
uint8_t  wrist_move_x10(int32_t deg_x10);
void     wrist_stop(void);                  /* dung tai cho (giu goc hien tai) */
void     wrist_release(void);               /* ngat xung, servo tha long */

/* Xung tho de hieu chuan, KHONG doi theo goc, khong gioi han goc. 0 = ngat xung */
void     wrist_set_us(uint16_t us);

int32_t  wrist_get_x10(void);               /* goc dang phat (0.1 do) */
uint16_t wrist_get_us(void);                /* xung dang phat, 0 = chua phat */
uint8_t  wrist_busy(void);                  /* 1 = dang chay cham chua toi dich */

#endif /* WRIST_H */
