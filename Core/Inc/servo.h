#ifndef SERVO_H
#define SERVO_H

#include <stdint.h>

/* Servo MG996R tren TIM3 CH1 (PA6).
 * CubeMX: TIM3 Prescaler = 15, Period = 19999 -> 1 tick = 1 us, chu ky 20 ms (50 Hz)
 * (voi clock timer 16 MHz tu HSI). */

#define SERVO_US_MIN  500    /* gioi han an toan, thu dan tu 1000..2000 truoc */
#define SERVO_US_MAX  2500

void     servo_start(void);
void     servo_set_us(uint16_t us);     /* do rong xung, micro giay. 0 = ngat xung (tha long) */
void     servo_set_deg(uint16_t deg);   /* 0..180 do, quy doi tuyen tinh sang 500..2500 us */
uint16_t servo_get_us(void);

/* Chay cham toi 'us', moi 20 ms tien 'step' micro giay (vd step 10 = 500 us/giay) */
void     servo_move_to(uint16_t us, uint16_t step);
/* Goi lien tuc trong while(1). Tra ve 1 khi vua toi dich */
uint8_t  servo_update(void);

#endif /* SERVO_H */
