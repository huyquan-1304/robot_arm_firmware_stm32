#include "motor.h"
#include "link.h"
#include "main.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Handle do CubeMX tao trong main.c */
extern TIM_HandleTypeDef htim1;   /* PWM 4 kenh */
extern TIM_HandleTypeDef htim2;   /* encoder FL (32 bit) */
extern TIM_HandleTypeDef htim4;   /* encoder FR (16 bit) */
extern TIM_HandleTypeDef htim5;   /* encoder RR (32 bit) */
extern TIM_HandleTypeDef htim8;   /* encoder RL (16 bit) */
extern TIM_HandleTypeDef htim6;   /* nhip PID 200 Hz */

#define DT_S        0.005f        /* chu ky ngat TIM6 */
#define DT_MS       5u
#define HIST_N      4u            /* toc do = trung binh 4 chu ky (20 ms) */
#define POS_TOL_REV (1.0f / 360.0f)   /* toi dich khi sai so < 1 do */

#define STBY_PORT   GPIOE
#define STBY_PIN    GPIO_PIN_10

typedef enum { MODE_OFF = 0, MODE_PWM, MODE_VEL, MODE_POS } mot_mode_t;

/* Su kien de motor_task() in ra (ISR khong printf) */
#define EVT_TIMEOUT   0x01u
#define EVT_STALL     0x02u
#define EVT_DONE      0x04u
#define EVT_REVERSE   0x08u

typedef struct {
    /* cau hinh phan cung */
    const char        *name;
    TIM_HandleTypeDef *pwm_tim;
    uint32_t           pwm_ch;
    GPIO_TypeDef      *in1_port; uint16_t in1_pin;
    GPIO_TypeDef      *in2_port; uint16_t in2_pin;
    TIM_HandleTypeDef *enc_tim;
    uint8_t            enc32;      /* 1 = timer 32 bit */
    int8_t             dir;        /* dau PWM -> lan ve truoc */
    int8_t             enc_dir;    /* dau encoder -> lan ve truoc */

    /* trang thai (ISR ghi, main doc) */
    volatile mot_mode_t mode;
    uint32_t           enc_last;
    volatile int32_t   counts;
    int32_t            hist[HIST_N];
    uint8_t            hidx;
    volatile float     rpm;
    volatile float     duty;       /* % ap dung, -100..100 */

    /* lenh */
    float              duty_cmd;   /* MODE_PWM */
    float              v_ref;      /* MODE_VEL, RPM */
    float              integ;

    /* che do vi tri, don vi vong banh va vong/giay */
    float              p_target, p_ref, prof_v, prof_vmax, prof_acc;
    volatile uint8_t   done;

    volatile uint32_t  ms_left;    /* tu dung (0 = khong hen gio) */
    uint8_t            off_at_zero; /* het gio o che do van toc: ve 0 roi tha */
    uint32_t           stall_ms, rev_ms;
    volatile uint8_t   fault;
    volatile uint8_t   evt;
} mot_t;

static mot_t mt[MOT_COUNT] = {
    { "FL", &htim1, TIM_CHANNEL_1, GPIOB, GPIO_PIN_12, GPIOB, GPIO_PIN_13, &htim2, 1, MOT_DIR_FL, MOT_ENC_FL },
    { "FR", &htim1, TIM_CHANNEL_2, GPIOB, GPIO_PIN_14, GPIOB, GPIO_PIN_15, &htim4, 0, MOT_DIR_FR, MOT_ENC_FR },
    { "RL", &htim1, TIM_CHANNEL_3, GPIOD, GPIO_PIN_14, GPIOD, GPIO_PIN_15, &htim8, 0, MOT_DIR_RL, MOT_ENC_RL },
    { "RR", &htim1, TIM_CHANNEL_4, GPIOE, GPIO_PIN_7,  GPIOE, GPIO_PIN_8,  &htim5, 1, MOT_DIR_RR, MOT_ENC_RR },
};

static float kff  = MOT_KFF_DEFAULT;
static float kp   = MOT_KP_DEFAULT;
static float ki   = MOT_KI_DEFAULT;
static float kpos = MOT_KPOS_DEFAULT;

static volatile uint8_t started = 0;

/* Lenh vi tri xep hang cho nhom (mot_group_begin / go) */
static uint8_t grp_open = 0;
static struct { uint8_t on; float deg, vmax, acc; } staged[MOT_COUNT];

/* ------------------------------------------------------------------ */

static float absf(float x) { return (x < 0.0f) ? -x : x; }
static float clampf(float x, float lo, float hi) { return (x < lo) ? lo : (x > hi) ? hi : x; }

static float sqrt_f(float x)                  /* tranh phu thuoc libm */
{
    if (x <= 0.0f) return 0.0f;
    float r = (x > 1.0f) ? x : 1.0f;
    for (int i = 0; i < 12; i++) r = 0.5f * (r + x / r);
    return r;
}

/* In so thuc 1 chu so thap phan (printf khong ho tro %f) */
static void pf1(float v)
{
    int32_t x = (int32_t)(v * 10.0f + ((v < 0.0f) ? -0.5f : 0.5f));
    if (x < 0) { putchar('-'); x = -x; }
    printf("%ld.%ld", (long)(x / 10), (long)(x % 10));
}

static void stby(int on)
{
    HAL_GPIO_WritePin(STBY_PORT, STBY_PIN, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/* duty % (duong = lan ve truoc). brake = 1: duty 0 thi phanh ngan, 0: tha troi */
static void output(mot_t *m, float duty, int brake)
{
    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(m->pwm_tim);
    float d = duty * (float)m->dir;
    m->duty = duty;

    if (d > 0.0f) {
        HAL_GPIO_WritePin(m->in1_port, m->in1_pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(m->in2_port, m->in2_pin, GPIO_PIN_RESET);
    } else if (d < 0.0f) {
        HAL_GPIO_WritePin(m->in1_port, m->in1_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(m->in2_port, m->in2_pin, GPIO_PIN_SET);
    } else if (brake) {                       /* IN1 = IN2 = H, PWM = H: phanh ngan */
        HAL_GPIO_WritePin(m->in1_port, m->in1_pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(m->in2_port, m->in2_pin, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(m->pwm_tim, m->pwm_ch, arr + 1u);
        return;
    } else {                                  /* IN1 = IN2 = L: tha troi */
        HAL_GPIO_WritePin(m->in1_port, m->in1_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(m->in2_port, m->in2_pin, GPIO_PIN_RESET);
    }
    float a = absf(d);
    if (a > 100.0f) a = 100.0f;
    __HAL_TIM_SET_COMPARE(m->pwm_tim, m->pwm_ch, (uint32_t)(a * (float)(arr + 1u) / 100.0f));
}

/* PID toc do -> duty %. Chong bao hoa tich phan */
static float vel_ctrl(mot_t *m, float v_target)
{
    float e   = v_target - m->rpm;
    float ff  = kff * v_target;
    float in2 = m->integ + e * DT_S;
    float u2  = ff + kp * e + ki * in2;
    if (!((u2 > 100.0f && e > 0.0f) || (u2 < -100.0f && e < 0.0f))) m->integ = in2;
    return clampf(ff + kp * e + ki * m->integ, -100.0f, 100.0f);
}

static void go_off(mot_t *m)
{
    m->mode = MODE_OFF;
    m->integ = 0.0f;
    m->ms_left = 0;
    output(m, 0.0f, 0);
}

/* ------------------------------------------------------------------ */
/* NGAT TIM6, 200 Hz */

static void motor_isr(void)
{
    for (int i = 0; i < MOT_COUNT; i++) {
        mot_t *m = &mt[i];

        /* 1. Doc encoder */
        uint32_t raw = m->enc_tim->Instance->CNT;
        int32_t  d   = m->enc32 ? (int32_t)(raw - m->enc_last)
                                : (int32_t)(int16_t)(uint16_t)(raw - m->enc_last);
        m->enc_last = raw;
        d *= m->enc_dir;
        m->counts += d;
        m->hist[m->hidx] = d;
        m->hidx = (uint8_t)((m->hidx + 1u) % HIST_N);
        int32_t sum = 0;
        for (unsigned k = 0; k < HIST_N; k++) sum += m->hist[k];
        m->rpm = (float)sum * 60.0f / (MOT_COUNTS_PER_REV * DT_S * (float)HIST_N);

        /* 2. Hen gio tu dung */
        if (m->ms_left) {
            if (m->ms_left <= DT_MS) {
                m->ms_left = 0;
                if (m->mode == MODE_PWM) go_off(m);
                else if (m->mode == MODE_VEL) { m->v_ref = 0.0f; m->off_at_zero = 1; }
                m->evt |= EVT_TIMEOUT;
            } else m->ms_left -= DT_MS;
        }

        /* 3. Dieu khien */
        float duty = 0.0f;
        int   brake = 0;
        switch (m->mode) {
        case MODE_PWM:
            duty = m->duty_cmd;
            break;
        case MODE_VEL:
            if (m->v_ref == 0.0f && absf(m->rpm) < 1.0f) {
                m->integ = 0.0f; brake = 1;
                if (m->off_at_zero) { m->off_at_zero = 0; go_off(m); }
            }
            else duty = vel_ctrl(m, m->v_ref);
            break;
        case MODE_POS: {
            float pos = (float)m->counts / MOT_COUNTS_PER_REV;
            float rem = m->p_target - m->p_ref;
            float a   = m->prof_acc;                    /* vong/s^2 */
            if (rem != 0.0f) {
                float sgn  = (rem > 0.0f) ? 1.0f : -1.0f;
                float vdes = sgn * sqrt_f(2.0f * a * absf(rem));
                if (absf(vdes) > m->prof_vmax) vdes = sgn * m->prof_vmax;
                float dv = vdes - m->prof_v;
                float dvmax = a * DT_S;
                m->prof_v += clampf(dv, -dvmax, dvmax);
                float step = m->prof_v * DT_S;
                if ((sgn > 0.0f && step >= rem) || (sgn < 0.0f && step <= rem) || absf(rem) < 1e-6f) {
                    m->p_ref = m->p_target; m->prof_v = 0.0f;
                } else m->p_ref += step;
            }
            float err = m->p_target - pos;
            if (m->p_ref == m->p_target && absf(err) < POS_TOL_REV) {
                if (!m->done) { m->done = 1; m->evt |= EVT_DONE; }
                m->integ = 0.0f;
                brake = 1;                              /* giu bang phanh ngan */
            } else {
                float v_cmd = m->prof_v * 60.0f + kpos * (m->p_ref - pos);
                duty = vel_ctrl(m, clampf(v_cmd, -MOT_RPM_MAX, MOT_RPM_MAX));
            }
            break;
        }
        default:
            break;
        }

        /* 4. Bao ve: ket truc, hoac encoder nguoc chieu (motor chay vot) */
        if (m->mode == MODE_VEL || m->mode == MODE_POS) {
            if (absf(duty) > MOT_STALL_DUTY && absf(m->rpm) < MOT_STALL_RPM) m->stall_ms += DT_MS;
            else m->stall_ms = 0;
            if (absf(duty) > 50.0f && duty * m->rpm < 0.0f && absf(m->rpm) > 5.0f) m->rev_ms += DT_MS;
            else m->rev_ms = 0;
            if (m->stall_ms >= MOT_STALL_MS) { go_off(m); m->fault = 1; m->evt |= EVT_STALL; continue; }
            if (m->rev_ms >= 300u)          { go_off(m); m->fault = 1; m->evt |= EVT_REVERSE; continue; }
        } else { m->stall_ms = 0; m->rev_ms = 0; }

        /* 5. Xuat */
        if (m->mode == MODE_OFF) output(m, 0.0f, 0);
        else output(m, duty, brake);
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6 && started) motor_isr();
}

/* ------------------------------------------------------------------ */

void motor_init(void)
{
    stby(0);
    for (int i = 0; i < MOT_COUNT; i++) {
        mot_t *m = &mt[i];
        /* Bo loc dau vao kenh 2 = 10 (CubeMX khong cho dat) */
        MODIFY_REG(m->enc_tim->Instance->CCMR1, TIM_CCMR1_IC2F, 10u << TIM_CCMR1_IC2F_Pos);
        HAL_TIM_Encoder_Start(m->enc_tim, TIM_CHANNEL_ALL);
        m->enc_last = m->enc_tim->Instance->CNT;
        HAL_TIM_PWM_Start(m->pwm_tim, m->pwm_ch);
        go_off(m);
    }
    started = 1;
    HAL_TIM_Base_Start_IT(&htim6);
}

static int any_active(void)
{
    for (int i = 0; i < MOT_COUNT; i++) if (mt[i].mode != MODE_OFF) return 1;
    return 0;
}

void mot_stop_all(void)
{
    __disable_irq();
    for (int i = 0; i < MOT_COUNT; i++) { go_off(&mt[i]); mt[i].done = 1; }
    __enable_irq();
    stby(0);
}

static void stop_one(mot_t *m)
{
    __disable_irq();
    go_off(m);
    m->done = 1;
    __enable_irq();
    if (!any_active()) stby(0);
}

void mot_set_rpm(mot_id_t id, float rpm)
{
    if (id >= MOT_COUNT) return;
    mot_t *m = &mt[id];
    if (m->fault) return;
    rpm = clampf(rpm, -MOT_RPM_MAX, MOT_RPM_MAX);
    __disable_irq();
    if (m->mode != MODE_VEL) { m->integ = 0.0f; m->mode = MODE_VEL; }
    m->v_ref = rpm;
    m->off_at_zero = 0;
    m->ms_left = 0;
    m->done = 1;
    __enable_irq();
    stby(1);
}

static void start_pos(mot_t *m, float deg, float rpm_max, float acc)
{
    float pos = (float)m->counts / MOT_COUNTS_PER_REV;
    m->p_target  = pos + deg / 360.0f;
    m->p_ref     = pos;
    m->prof_v    = 0.0f;
    m->prof_vmax = clampf(absf(rpm_max), 0.1f, MOT_RPM_MAX) / 60.0f;
    m->prof_acc  = ((acc > 0.1f) ? acc : 0.1f) / 60.0f;
    m->integ     = 0.0f;
    m->ms_left   = 0;
    m->done      = 0;
    m->mode      = MODE_POS;
}

void mot_move_rel(mot_id_t id, float deg, float rpm_max, float acc)
{
    if (id >= MOT_COUNT) return;
    if (grp_open) {
        staged[id].on = 1; staged[id].deg = deg; staged[id].vmax = rpm_max; staged[id].acc = acc;
        return;
    }
    if (mt[id].fault) return;
    __disable_irq();
    start_pos(&mt[id], deg, rpm_max, acc);
    __enable_irq();
    stby(1);
}

void mot_group_begin(void)
{
    memset(staged, 0, sizeof(staged));
    grp_open = 1;
}

void mot_group_go(void)
{
    grp_open = 0;
    stby(1);
    __disable_irq();                          /* 4 banh bat dau trong cung 1 chu ky */
    for (int i = 0; i < MOT_COUNT; i++)
        if (staged[i].on && !mt[i].fault) start_pos(&mt[i], staged[i].deg, staged[i].vmax, staged[i].acc);
    __enable_irq();
}

int   mot_move_done(mot_id_t id) { return (id < MOT_COUNT) ? (mt[id].mode != MODE_POS || mt[id].done) : 1; }
float mot_get_rpm(mot_id_t id)   { return (id < MOT_COUNT) ? mt[id].rpm : 0.0f; }
int32_t mot_get_counts(mot_id_t id) { return (id < MOT_COUNT) ? mt[id].counts : 0; }
float mot_get_deg(mot_id_t id)   { return (id < MOT_COUNT) ? (float)mt[id].counts * 360.0f / MOT_COUNTS_PER_REV : 0.0f; }
int   mot_fault(mot_id_t id)     { return (id < MOT_COUNT) ? mt[id].fault : 0; }
const char *mot_name(mot_id_t id){ return (id < MOT_COUNT) ? mt[id].name : "?"; }

void mot_zero(mot_id_t id)
{
    if (id >= MOT_COUNT) return;
    __disable_irq();
    mt[id].counts = 0;
    if (mt[id].mode == MODE_POS) { mt[id].p_target = 0.0f; mt[id].p_ref = 0.0f; mt[id].prof_v = 0.0f; }
    __enable_irq();
}

/* ------------------------------------------------------------------ */
/* THONG BAO TU ISR */

void motor_task(void)
{
    for (int i = 0; i < MOT_COUNT; i++) {
        mot_t *m = &mt[i];
        uint8_t e = m->evt;
        if (!e) continue;
        __disable_irq(); m->evt = 0; __enable_irq();
        if (e & EVT_STALL)   printf("\r\n[Banh %s: KET TRUC (PWM cao ma khong quay), da ngat. Kiem tra roi go 'm %s c']\r\n> ", m->name, m->name);
        if (e & EVT_REVERSE) printf("\r\n[Banh %s: ENCODER NGUOC CHIEU motor, da ngat. Go 'm %s daoe' roi 'm %s c']\r\n> ", m->name, m->name, m->name);
        if (e & EVT_TIMEOUT) printf("\r\n[Banh %s: het gio, da dung]\r\n> ", m->name);
    }
    if (!any_active()) stby(0);
}

/* ------------------------------------------------------------------ */
/* CONSOLE "m ..." */

static int parse_wheel(const char *s)
{
    if (!s) return -1;
    for (int i = 0; i < MOT_COUNT; i++) {
        const char *n = mt[i].name;
        if ((s[0] == n[0] || s[0] == n[0] + 32) && (s[1] == n[1] || s[1] == n[1] + 32) && s[2] == '\0') return i;
    }
    return -1;
}

static const char *mode_str(mot_mode_t md)
{
    switch (md) { case MODE_PWM: return "PWM"; case MODE_VEL: return "VAN TOC"; case MODE_POS: return "VI TRI"; default: return "TAT"; }
}

static void show_all(void)
{
    printf("Banh | che do  | PWM %%  | toc do RPM | vi tri do | xung     | loi\r\n");
    for (int i = 0; i < MOT_COUNT; i++) {
        mot_t *m = &mt[i];
        printf("  %s | %-7s | ", m->name, mode_str(m->mode));
        pf1(m->duty); printf(" | "); pf1(m->rpm); printf(" | "); pf1(mot_get_deg((mot_id_t)i));
        printf(" | %ld | %s\r\n", (long)m->counts, m->fault ? "KET/NGUOC" : "-");
    }
}

static void show_enc(void)
{
    for (int i = 0; i < MOT_COUNT; i++) {
        printf("  %s: %ld xung = ", mt[i].name, (long)mt[i].counts);
        pf1(mot_get_deg((mot_id_t)i)); printf(" do = ");
        pf1(mot_get_deg((mot_id_t)i) / 360.0f); printf(" vong\r\n");
    }
}

static void watch(void)
{
    printf("Theo doi toc do / vi tri 4 banh moi 0.2 s. Bam phim bat ky de thoat.\r\n");
    uint32_t next = HAL_GetTick();
    while (!link_any_key()) {
        link_task();
        motor_task();
        if ((int32_t)(HAL_GetTick() - next) >= 0) {
            next += 200u;
            for (int i = 0; i < MOT_COUNT; i++) {
                printf("%s ", mt[i].name); pf1(mt[i].rpm); printf(" RPM ");
                pf1(mot_get_deg((mot_id_t)i)); printf(" do  ");
            }
            printf("\r\n");
        }
    }
    printf("Thoat theo doi\r\n");
}

static void show_pid(void)
{
    printf("PID toc do: kp "); pf1(kp); printf(" | ki "); pf1(ki); printf(" | kff ");
    printf("%ld.%02ld", (long)(kff * 100.0f) / 100, (long)(kff * 100.0f) % 100);
    printf(" | kpos "); pf1(kpos); printf("   (don vi %% PWM / RPM)\r\n");
}

void motor_print_help(void)
{
    printf("  --- MOTOR BANH (TB6612). <banh> = fl, fr, rl, rr. KE XE LEN khi test ---\r\n");
    printf("  m                  trang thai 4 banh\r\n");
    printf("  m e                so xung encoder 4 banh (quay tay de kiem tra)\r\n");
    printf("  m w                theo doi toc do / vi tri lien tuc\r\n");
    printf("  m <banh> pwm <%%> [giay]  chay PWM KHONG PID (-100..100), tu dung sau 2 s\r\n");
    printf("  m <banh> v <rpm> [giay]  PID toc do, tu dung sau 5 s (0 = khong hen gio)\r\n");
    printf("  m <banh> q <vong>  quay dung so vong\r\n");
    printf("  m <banh> z / m z   dat vi tri = 0\r\n");
    printf("  m <banh> dao       dao chieu MOTOR (PWM duong ma banh lan lui)\r\n");
    printf("  m <banh> daoe      dao chieu ENCODER (lan toi ma so xung giam)\r\n");
    printf("  m <banh> c / m c   xoa loi ket truc\r\n");
    printf("  m pid [kp ki [kff [kpos]]]  xem / dat PID\r\n");
    printf("  m <banh> x / m x   dung 1 banh / DUNG CA 4\r\n");
}

void motor_cmd(int argc, char **argv)
{
    const char *a = (argc > 1) ? argv[1] : NULL;
    const char *b = (argc > 2) ? argv[2] : NULL;
    const char *c = (argc > 3) ? argv[3] : NULL;
    const char *d = (argc > 4) ? argv[4] : NULL;
    const char *e = (argc > 5) ? argv[5] : NULL;

    if (!a)                 { show_all(); return; }
    if (!strcmp(a, "e"))    { show_enc(); return; }
    if (!strcmp(a, "w"))    { watch(); return; }
    if (!strcmp(a, "x"))    { mot_stop_all(); printf("Da dung ca 4 banh\r\n"); return; }
    if (!strcmp(a, "z"))    { for (int i = 0; i < MOT_COUNT; i++) mot_zero((mot_id_t)i); printf("Vi tri 4 banh = 0\r\n"); return; }
    if (!strcmp(a, "c"))    { for (int i = 0; i < MOT_COUNT; i++) mt[i].fault = 0; printf("Da xoa loi 4 banh\r\n"); return; }
    if (!strcmp(a, "pid")) {
        if (b && c) {
            kp = strtof(b, NULL); ki = strtof(c, NULL);
            if (d) kff = strtof(d, NULL);
            if (e) kpos = strtof(e, NULL);
            for (int i = 0; i < MOT_COUNT; i++) mt[i].integ = 0.0f;
        }
        show_pid();
        return;
    }

    int w = parse_wheel(a);
    if (w < 0) { printf("Khong hieu 'm %s'. Banh: fl, fr, rl, rr. Go h de xem lenh.\r\n", a); return; }
    mot_t *m = &mt[w];

    if (!b) {
        printf("%s: %s, PWM ", m->name, mode_str(m->mode)); pf1(m->duty); printf(" %%, ");
        pf1(m->rpm); printf(" RPM, "); pf1(mot_get_deg((mot_id_t)w)); printf(" do, %ld xung, dau motor %+d, dau encoder %+d%s\r\n",
               (long)m->counts, m->dir, m->enc_dir, m->fault ? ", DANG LOI" : "");
        return;
    }
    if (!strcmp(b, "x")) { stop_one(m); printf("%s: dung\r\n", m->name); return; }
    if (!strcmp(b, "z")) { mot_zero((mot_id_t)w); printf("%s: vi tri = 0\r\n", m->name); return; }
    if (!strcmp(b, "c")) { m->fault = 0; printf("%s: da xoa loi\r\n", m->name); return; }
    if (!strcmp(b, "dao"))  { stop_one(m); m->dir = (int8_t)-m->dir;
        printf("%s: dau MOTOR = %+d. Nho sua MOT_DIR_%s trong motor.h\r\n", m->name, m->dir, m->name); return; }
    if (!strcmp(b, "daoe")) { stop_one(m); m->enc_dir = (int8_t)-m->enc_dir;
        printf("%s: dau ENCODER = %+d. Nho sua MOT_ENC_%s trong motor.h\r\n", m->name, m->enc_dir, m->name); return; }

    if (m->fault) { printf("%s dang loi. Go 'm %s c' truoc\r\n", m->name, m->name); return; }

    if (!strcmp(b, "pwm")) {
        if (!c) { printf("Vd: m %s pwm 30\r\n", m->name); return; }
        float p = clampf(strtof(c, NULL), -100.0f, 100.0f);
        uint32_t s = d ? (uint32_t)atoi(d) : 2u;
        if (s == 0u || s > 30u) s = 2u;
        __disable_irq();
        m->duty_cmd = p; m->mode = MODE_PWM; m->ms_left = s * 1000u; m->done = 1;
        __enable_irq();
        stby(1);
        printf("%s: PWM ", m->name); pf1(p); printf(" %% trong %lu s (khong PID)\r\n", (unsigned long)s);
    }
    else if (!strcmp(b, "v")) {
        if (!c) { printf("Vd: m %s v 20\r\n", m->name); return; }
        float v = strtof(c, NULL);
        uint32_t s = d ? (uint32_t)atoi(d) : 5u;
        mot_set_rpm((mot_id_t)w, v);
        __disable_irq(); m->ms_left = s * 1000u; __enable_irq();
        printf("%s: toc do ", m->name); pf1(clampf(v, -MOT_RPM_MAX, MOT_RPM_MAX)); printf(" RPM");
        if (s) printf(", tu dung sau %lu s\r\n", (unsigned long)s); else printf(", KHONG hen gio ('m x' de dung)\r\n");
    }
    else if (!strcmp(b, "q")) {
        if (!c) { printf("Vd: m %s q 1\r\n", m->name); return; }
        float rev = strtof(c, NULL);
        if (rev == 0.0f || absf(rev) > 20.0f) { printf("So vong sai (toi da 20)\r\n"); return; }
        mot_move_rel((mot_id_t)w, rev * 360.0f, 30.0f, 60.0f);
        printf("%s: quay ", m->name); pf1(rev); printf(" vong o 30 RPM. Xong go 'm %s' xem vi tri\r\n", m->name);
    }
    else printf("Khong hieu 'm %s %s'\r\n", a, b);
}
