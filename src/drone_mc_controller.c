/*
 * drone_mc_controller.c — 4-motor ESC control via MCPWM (ESP32-S3 / Zephyr RTOS)
 *
 * Channel → GPIO mapping:
 *   Channel 0 → GPIO 18  (Motor 1 — Front-Left)
 *   Channel 1 → GPIO 17  (Motor 2 — Front-Right)
 *   Channel 2 → GPIO 16  (Motor 3 — Rear-Left)
 *   Channel 3 → GPIO 15  (Motor 4 — Rear-Right)
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/logging/log.h>

#include "drone_mc_controller.h"

LOG_MODULE_REGISTER(esc_4motors, LOG_LEVEL_INF);

/* ── PWM device ─────────────────────────────────────────────────────── */
#define ESC_PWM_NODE  DT_NODELABEL(mcpwm0)

/* ── Channels (match overlay pin mapping) ───────────────────────────── */
#define NUM_MOTORS  4

/* ── Timing (nanoseconds) ───────────────────────────────────────────── */
#define ESC_PERIOD_NS      20000000U  /* 20 ms  = 50 Hz          */
#define ESC_PULSE_MIN_NS    1000000U  /* 1000 µs = disarmed/idle */
#define ESC_PULSE_MAX_NS    2000000U  /* 2000 µs = full throttle */
#define ESC_ARM_MS             3000U  /* arming hold time        */

/* Motor channel lookup table */
static const uint8_t motor_ch[NUM_MOTORS] = {
    MOTOR_FRONT_LEFT,   /* 0 — GPIO 18 */
    MOTOR_FRONT_RIGHT,  /* 1 — GPIO 17 */
    MOTOR_REAR_LEFT,    /* 2 — GPIO 16 */
    MOTOR_REAR_RIGHT,   /* 3 — GPIO 15 */
};

static const struct device *esc_pwm;

/* ── Low-level helper ───────────────────────────────────────────────── */

/**
 * @brief  Send a raw pulse width (µs) to one motor channel.
 *         Clamped to [1000, 2000] µs.
 */
static int motor_set_pulse_us(uint8_t motor_idx, uint32_t pulse_us)
{
    if (motor_idx >= NUM_MOTORS) {
        return -EINVAL;
    }
    if (pulse_us < 1000U) { pulse_us = 1000U; }
    if (pulse_us > 2000U) { pulse_us = 2000U; }

    int ret = pwm_set(esc_pwm,
                      motor_ch[motor_idx],
                      ESC_PERIOD_NS,
                      pulse_us * 1000U,      /* µs → ns */
                      PWM_POLARITY_NORMAL);
    if (ret) {
        LOG_ERR("Motor %u pwm_set failed: %d", motor_idx + 1, ret);
    }
    return ret;
}

/* ── Public API ─────────────────────────────────────────────────────── */

int motor_set_throttle(uint8_t motor_idx, uint8_t percent)
{
    if (percent > 100U) { percent = 100U; }

    uint32_t pulse_us = 1000U + (uint32_t)percent * 10U;
    LOG_DBG("Motor %u → %3u%%  (%u µs)", motor_idx + 1, percent, pulse_us);
    return motor_set_pulse_us(motor_idx, pulse_us);
}

int motors_set_all(uint8_t percent)
{
    int ret = 0;
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        ret |= motor_set_throttle(i, percent);
    }
    return ret;
}

/* ── Arming sequence (called by motor_esc_init) ─────────────────────── */

static int esc_arm_all(void)
{
    LOG_INF("Arming ESCs — holding 1000 µs for %u ms ...", ESC_ARM_MS);

    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        int ret = motor_set_pulse_us(i, 1000U);
        if (ret) {
            LOG_ERR("Arm failed on motor %u", i + 1);
            return ret;
        }
    }

    k_sleep(K_MSEC(ESC_ARM_MS));
    LOG_INF("All ESCs armed.");
    return 0;
}

/* ── Init ────────────────────────────────────────────────────────────── */

int motor_esc_init(void)
{
    esc_pwm = DEVICE_DT_GET(ESC_PWM_NODE);
    if (!device_is_ready(esc_pwm)) {
        LOG_ERR("PWM device not ready!");
        return -ENODEV;
    }

    LOG_INF("PWM device ready — starting ESC arming sequence.");
    return esc_arm_all();
}

/* ── RC mixing ───────────────────────────────────────────────────────── */

void motors_set_from_rc(const struct DataPackage *rc)
{
    /* Persistent state across calls */
    static int64_t last_update_ms;   /* timestamp of last motor write     */
    static int32_t throttle_smooth;  /* ramped throttle value (0-80%)     */
    static uint8_t last_pct[NUM_MOTORS]; /* last applied value per motor  */

    /* Safety: immediate disarm bypasses all rate-limiting */
    if (!rc->drone_active) {
        for (uint8_t i = 0; i < NUM_MOTORS; i++) last_pct[i] = 0;
        throttle_smooth = 0;
        last_update_ms  = 0;
        motors_set_all(0);
        return;
    }

    /* Self-rate-limit to 50 Hz to match the ESC PWM period (20 ms).
     * Calling pwm_set() more often than once per period risks updating
     * the MCPWM compare register mid-pulse → spurious partial pulses → jerks. */
    int64_t now = k_uptime_get();
    if ((now - last_update_ms) < 20) {
        return;
    }
    last_update_ms = now;

    /* Throttle ramp: limit how fast the base throttle can change.
     *   +2 %/call × 50 Hz =  100 %/s ramp-up  (smooth takeoff)
     *   −5 %/call × 50 Hz = −250 %/s ramp-down (quicker descent)
     * Attitude corrections (roll/pitch/yaw) bypass the ramp — they must
     * respond immediately for stability. */
    int32_t throttle_target = ((int32_t)rc->y_left * MOTOR_THROTTLE_MAX) / RC_MAX;
    int32_t diff = throttle_target - throttle_smooth;
    if (diff >  2) diff =  2;
    if (diff < -5) diff = -5;
    throttle_smooth += diff;

    /* Attitude mixing (unramped) */
    int32_t roll  = ((int32_t)(rc->x_right - RC_CENTER) * MOTOR_MIXING_MAX)
                    / (RC_MAX / 2);
    int32_t pitch = ((int32_t)(rc->y_right - RC_CENTER) * MOTOR_MIXING_MAX)
                    / (RC_MAX / 2);
    int32_t yaw   = ((int32_t)(rc->x_left  - RC_CENTER) * MOTOR_MIXING_MAX)
                    / (RC_MAX / 2);

    /*
     * Standard X-frame quadrotor mixing:
     *
     *          FRONT
     *   M1(CCW) │ M2(CW)
     *   ────────┼────────
     *   M3(CW)  │ M4(CCW)
     *          REAR
     *
     *   M1 Front-Left  = T + Pitch + Roll − Yaw
     *   M2 Front-Right = T + Pitch − Roll + Yaw
     *   M3 Rear-Left   = T − Pitch + Roll + Yaw
     *   M4 Rear-Right  = T − Pitch − Roll − Yaw
     */
    int32_t m[NUM_MOTORS];
    m[MOTOR_FRONT_LEFT]  = throttle_smooth + pitch + roll - yaw;
    m[MOTOR_FRONT_RIGHT] = throttle_smooth + pitch - roll + yaw;
    m[MOTOR_REAR_LEFT]   = throttle_smooth - pitch + roll + yaw;
    m[MOTOR_REAR_RIGHT]  = throttle_smooth - pitch - roll - yaw;

    /* Apply — only call pwm_set() when the value actually changes.
     * Avoids redundant register writes at 250 Hz for a constant throttle. */
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        uint8_t pct = (uint8_t)(m[i] < 0 ? 0 : m[i] > 100 ? 100 : m[i]);
        if (pct != last_pct[i]) {
            last_pct[i] = pct;
            motor_set_throttle(i, pct);
        }
    }

    LOG_DBG("T:%d R:%d P:%d Y:%d → M:%d %d %d %d",
            (int)throttle_smooth, (int)roll, (int)pitch, (int)yaw,
            (int)last_pct[0], (int)last_pct[1],
            (int)last_pct[2], (int)last_pct[3]);
}
