#include "ros_link.h"
#include "joint.h"
#include "home.h"
#include "zdt_x42s.h"
#include "cmd_uart.h"
#include "uart_rx.h"
#include "main.h"
#include <stdio.h>
#include <string.h>

#ifndef USE_SERVO
#define USE_SERVO 1
#endif
#if USE_SERVO
#include "servo.h"
#endif

extern UART_HandleTypeDef huart1;   /* do CubeMX tao trong main.c */

/* ===================== THONG SO ===================== */
#define ROS_CYCLE_MS         20u     /* chu ky gui lenh / doc vi tri / gui STATE (50 Hz) */
#define ROS_WATCHDOG_MS     200u     /* dang chay ma qua lau khong co CMD_JOINT thi dung */
#define ROS_MODE_TIMEOUT_MS 1000u    /* qua lau khong co goi nao thi tro lai console */

/* ===================== GIAO THUC ===================== */
#define HDR0 0xAAu
#define HDR1 0x55u

#define T_CMD_JOINT    0x01u
#define T_CMD_ENABLE   0x02u
#define T_CMD_ESTOP    0x03u
#define T_CMD_SET_ZERO 0x04u
#define T_CMD_PING     0x05u
#define T_CMD_HOME     0x06u
#define T_STATE        0x81u

#define LEN_CMD_JOINT  20u
#define LEN_STATE      23u
#define MAX_PAYLOAD    32u

static uint16_t crc16_ccitt(const uint8_t *d, uint16_t n, uint16_t crc)
{
    while (n--) {
        crc ^= (uint16_t)(*d++) << 8;
        for (uint8_t i = 0; i < 8; i++) {
            crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u) : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

static void     put_u16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
static void     put_i32(uint8_t *p, int32_t v)
{
    uint32_t u = (uint32_t)v;
    p[0] = (uint8_t)u; p[1] = (uint8_t)(u >> 8); p[2] = (uint8_t)(u >> 16); p[3] = (uint8_t)(u >> 24);
}
static uint16_t get_u16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static int32_t  get_i32(const uint8_t *p)
{
    return (int32_t)((uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24));
}

/* ===================== TRANG THAI ===================== */
static uint8_t  active = 0;          /* dang o che do ROS */
static uint8_t  enabled = 0;         /* dang nhan lenh chuyen dong */
static uint16_t status = ROS_ST_NOT_HOMED;
static uint16_t last_seq = 0;

static uint32_t last_rx_tick = 0;    /* goi hop le gan nhat (bat ky loai) */
static uint32_t last_cmd_tick = 0;   /* CMD_JOINT gan nhat */
static uint32_t last_cycle_tick = 0;

static int32_t  cmd_x10[JOINT_COUNT];
static uint16_t cmd_servo_us = 0;
static uint8_t  have_new_cmd = 0;

static int32_t  pos_x10[JOINT_COUNT];  /* goc thuc doc tu encoder */
static uint8_t  flag_rr = 0;           /* doc co ket truc lan luot 1 khop moi chu ky */

/* Loi CAN thoang qua (1 frame hong, bus-off tu hoi phuc trong ~3 ms) khong bao CAN_ERROR ngay.
 * Chi bao khi loi lien tiep ROS_CAN_FAIL_LIMIT chu ky (3 x 20 ms = 60 ms). Lenh joint_stream loi
 * khong cap nhat dich trong joint.c, nen chu ky sau tu gui lai. */
#define ROS_CAN_FAIL_LIMIT 3u
static uint8_t  can_fail_streak = 0;

static uint8_t  home_req = 0;          /* Pi vua gui CMD_HOME, chay o ros_link_task */
static uint8_t  homing = 0;            /* dang home */
static uint8_t  home_abort = 0;        /* Pi gui CMD_ESTOP trong luc home */
static uint32_t last_home_state_tick = 0;

/* ===================== GUI ===================== */
static void send_frame(uint8_t type, const uint8_t *payload, uint8_t len)
{
    uint8_t f[4 + MAX_PAYLOAD + 2];
    f[0] = HDR0; f[1] = HDR1; f[2] = type; f[3] = len;
    if (len) memcpy(&f[4], payload, len);
    put_u16(&f[4 + len], crc16_ccitt(&f[2], (uint16_t)(2u + len), 0xFFFFu));
    HAL_UART_Transmit(&huart1, f, (uint16_t)(6u + len), 10);
}

static void send_state(uint8_t loop_ms)
{
    uint8_t p[LEN_STATE];
    put_u16(&p[0], last_seq);
    for (int i = 0; i < JOINT_COUNT; i++) put_i32(&p[2 + 4 * i], pos_x10[i]);
#if USE_SERVO
    put_u16(&p[18], servo_get_us());
#else
    put_u16(&p[18], 0);
#endif
    put_u16(&p[20], (uint16_t)(status | (enabled ? ROS_ST_ENABLED : 0u) | (homing ? ROS_ST_HOMING : 0u)));
    p[22] = loop_ms;
    send_frame(T_STATE, p, LEN_STATE);
}

/* ===================== XU LY LENH ===================== */
static void stop_motion(void)
{
    joint_stop_all();     /* lenh FE: dung ngay, dong co van giu luc */
    enabled = 0;
}

/* Tra ve 1 neu co khop doc loi (giu gia tri cu cho khop do) */
static uint8_t read_all_positions(void)
{
    uint8_t err = 0;
    for (uint8_t n = 1; n <= JOINT_COUNT; n++) {
        int32_t v;
        if (joint_read(n, &v) == JOINT_OK) pos_x10[n - 1] = v;
        else err = 1;
    }
    return err;
}

static void note_can_cycle(uint8_t err)
{
    if (err) {
        if (can_fail_streak < ROS_CAN_FAIL_LIMIT) can_fail_streak++;
        if (can_fail_streak >= ROS_CAN_FAIL_LIMIT) status |= ROS_ST_CAN_ERROR;
    } else {
        can_fail_streak = 0;
        status &= (uint16_t)~ROS_ST_CAN_ERROR;
    }
}

/* NOT_HOMED = con khop chua home bang cam bien (home.c) */
static void update_homed_flag(void)
{
    uint8_t all = 1;
    for (uint8_t n = 1; n <= JOINT_COUNT; n++) if (!home_is_homed(n)) all = 0;
    if (all) status &= (uint16_t)~ROS_ST_NOT_HOMED;
    else     status |= ROS_ST_NOT_HOMED;
}

static void on_enable(uint8_t on)
{
    if (!on) { stop_motion(); return; }

    /* Chua home du 4 khop thi goc 0 chua dung voi URDF: khong cho chay */
    update_homed_flag();
    if (status & ROS_ST_NOT_HOMED) { enabled = 0; return; }

    (void)joint_enable(0, 1);
    if (read_all_positions()) {           /* khong bat khi chua doc du vi tri 4 khop */
        status |= ROS_ST_CAN_ERROR;
        enabled = 0;
        return;
    }
    can_fail_streak = 0;
    status &= (uint16_t)~ROS_ST_CAN_ERROR;

    /* Dich ban dau = vi tri thuc: lenh dau tien tu Pi chi lech rat it, khong giat */
    for (uint8_t n = 1; n <= JOINT_COUNT; n++) {
        joint_set_target(n, pos_x10[n - 1]);
        cmd_x10[n - 1] = pos_x10[n - 1];
    }
    status &= (uint16_t)~(ROS_ST_WATCHDOG | ROS_ST_ESTOP | ROS_ST_LIMIT);
    have_new_cmd  = 0;
    last_cmd_tick = HAL_GetTick();
    enabled = 1;
}

/* Diem 0 bang tay cho khop CHUA home bang cam bien. Khop da home thi bo qua,
 * tranh xoa offset do cam bien tiem can dat. */
static void on_set_zero(uint8_t mask)
{
    if (enabled) return;                  /* chi dat diem 0 khi dang dung */
    for (uint8_t n = 1; n <= JOINT_COUNT; n++) {
        if (!(mask & (1u << (n - 1)))) continue;
        if (home_is_homed(n)) continue;
        (void)joint_zero_here(n);
    }
}


static void handle_frame(uint8_t type, const uint8_t *p, uint8_t len)
{
    last_rx_tick = HAL_GetTick();
    if (!active) {
        active = 1;
        zdt_debug = 0;                     /* khong in frame CAN ra UART nua */
        enabled = 0;
        last_cycle_tick = HAL_GetTick();
        can_fail_streak = 0;
        note_can_cycle(read_all_positions());
    }

    switch (type) {
    case T_CMD_JOINT:
        if (len != LEN_CMD_JOINT) return;
        last_seq = get_u16(&p[0]);
        for (int i = 0; i < JOINT_COUNT; i++) cmd_x10[i] = get_i32(&p[2 + 4 * i]);
        cmd_servo_us  = get_u16(&p[18]);
        have_new_cmd  = 1;
        last_cmd_tick = HAL_GetTick();
        break;
    case T_CMD_ENABLE:
        if (len == 1 && !homing) on_enable(p[0]);
        break;
    case T_CMD_ESTOP:
        stop_motion();
        status |= ROS_ST_ESTOP;
        if (homing) home_abort = 1;
        break;
    case T_CMD_SET_ZERO:
        if (len == 1 && !homing) on_set_zero(p[0]);
        break;
    case T_CMD_HOME:
        if (!enabled && !homing) home_req = 1;   /* chay trong ros_link_task, khong chay trong bo tach khung */
        break;
    case T_CMD_PING:
    default:
        break;
    }
}

/* ===================== BO TACH KHUNG ===================== */
typedef enum { S_IDLE, S_HDR1, S_TYPE, S_LEN, S_PAYLOAD, S_CRC0, S_CRC1 } rx_state_t;

static rx_state_t rs = S_IDLE;
static uint8_t    r_type, r_len, r_idx;
static uint8_t    r_buf[MAX_PAYLOAD];
static uint16_t   r_crc;

int ros_link_feed(uint8_t c)
{
    switch (rs) {
    case S_IDLE:
        if (c == HDR0) { rs = S_HDR1; return 1; }
        return 0;                                   /* byte cua console text */
    case S_HDR1:
        rs = (c == HDR1) ? S_TYPE : ((c == HDR0) ? S_HDR1 : S_IDLE);
        return 1;
    case S_TYPE:
        r_type = c; rs = S_LEN;
        return 1;
    case S_LEN:
        if (c > MAX_PAYLOAD) { rs = S_IDLE; return 1; }
        r_len = c; r_idx = 0;
        rs = r_len ? S_PAYLOAD : S_CRC0;
        return 1;
    case S_PAYLOAD:
        r_buf[r_idx++] = c;
        if (r_idx >= r_len) rs = S_CRC0;
        return 1;
    case S_CRC0:
        r_crc = c; rs = S_CRC1;
        return 1;
    case S_CRC1: {
        r_crc |= (uint16_t)c << 8;
        uint8_t hdr[2] = { r_type, r_len };
        uint16_t crc = crc16_ccitt(hdr, 2, 0xFFFFu);
        crc = crc16_ccitt(r_buf, r_len, crc);
        if (crc == r_crc) handle_frame(r_type, r_buf, r_len);
        rs = S_IDLE;
        return 1;
    }
    }
    rs = S_IDLE;
    return 1;
}

/* ===================== VONG LAP ===================== */
void ros_link_init(void)
{
    active = 0;
    enabled = 0;
    status = ROS_ST_NOT_HOMED;
    rs = S_IDLE;
    home_req = homing = home_abort = 0;
}

uint8_t ros_link_active(void)
{
    return active;
}

int ros_link_poll_blocking(void)
{
    uint8_t b;
    while (uart_rx_get(&b)) (void)ros_link_feed(b);   /* PING giu last_rx_tick, ESTOP dat home_abort */
    uint32_t now = HAL_GetTick();
    if (now - last_home_state_tick >= 100u) {
        last_home_state_tick = now;
        send_state(0);
    }
    return home_abort;
}

/* Home ca tay theo yeu cau cua Pi. Chay chan (khoang 30 s), trong luc do home.c goi ham huy
 * cua cmd_uart.c -> ros_link_poll_blocking() de van nhan goi tu Pi va gui STATE. */
static void run_home(void)
{
    home_req   = 0;
    homing     = 1;
    home_abort = 0;
    status &= (uint16_t)~(ROS_ST_ESTOP | ROS_ST_WATCHDOG | ROS_ST_LIMIT);
    last_home_state_tick = HAL_GetTick();
    send_state(0);

    (void)cmd_home_all();                 /* home.c cap nhat home_is_homed() cho tung khop */

    homing = 0;
    uint32_t now = HAL_GetTick();
    last_rx_tick = last_cmd_tick = last_cycle_tick = now;   /* khong thoat che do ROS vi home lau */
    have_new_cmd = 0;
    note_can_cycle(read_all_positions());
    update_homed_flag();
    send_state(0);
}

void ros_link_task(void)
{
    if (!active) return;
    home_sensor_update();                 /* giu bo loc cam bien chay khi o che do ROS */
    uint32_t now = HAL_GetTick();

    /* Pi im lang qua lau: tro lai console */
    if (now - last_rx_tick > ROS_MODE_TIMEOUT_MS) {
        if (enabled) stop_motion();
        active = 0;
        printf("\r\n[Thoat che do ROS: %lu ms khong nhan goi]\r\n> ", (unsigned long)ROS_MODE_TIMEOUT_MS);
        return;
    }

    /* Watchdog: dang chay ma khong con lenh */
    if (enabled && now - last_cmd_tick > ROS_WATCHDOG_MS) {
        stop_motion();
        status |= ROS_ST_WATCHDOG;
    }

    /* Pi yeu cau home */
    if (home_req) {
        run_home();
        return;
    }

    uint32_t dt = now - last_cycle_tick;
    if (dt < ROS_CYCLE_MS) return;
    last_cycle_tick = now;

    uint8_t cyc_err = 0;

    /* 1. Gui diem dich moi xuong driver */
    if (enabled && have_new_cmd) {
        have_new_cmd = 0;
        for (uint8_t n = 1; n <= JOINT_COUNT; n++) {
            joint_status_t s = joint_stream(n, cmd_x10[n - 1], dt);
            if (s == JOINT_ERR_LIMIT) {
                status |= ROS_ST_LIMIT;           /* lenh bi bo, Pi se thay tay khong toi dich */
            } else if (s == JOINT_ERR_DRIVER) {
                cyc_err = 1;                      /* dich chua cap nhat: chu ky sau gui lai */
            }
        }
#if USE_SERVO
        if (cmd_servo_us != 0 && cmd_servo_us != servo_get_us()) servo_set_us(cmd_servo_us);
#endif
    }

    /* 2. Doc vi tri thuc va co ket truc (1 khop moi chu ky de tiet kiem thoi gian bus) */
    if (read_all_positions()) cyc_err = 1;
    note_can_cycle(cyc_err);
    {
        uint8_t n = (uint8_t)(flag_rr + 1u), f;
        joint_t *j = joint_get(n);
        if (j && zdt_read_flags(j->addr, &f) == ZDT_OK) {
            uint16_t bit = (uint16_t)(ROS_ST_STALL_J1 << flag_rr);
            if (f & (ZDT_FLAG_STALL | ZDT_FLAG_STALL_PROT)) {
                status |= bit;
                if (enabled) stop_motion();
            } else {
                status &= (uint16_t)~bit;
            }
        }
        flag_rr = (uint8_t)((flag_rr + 1u) % JOINT_COUNT);
    }
    (void)zdt_poll_reached();             /* bo cac thong bao "da toi noi" con trong FIFO */

    update_homed_flag();

    /* 3. Gui STATE, kem thoi gian vong lap de Pi biet co kip 50 Hz khong */
    uint32_t used = HAL_GetTick() - now;
    send_state((uint8_t)(used > 255u ? 255u : used));
}
