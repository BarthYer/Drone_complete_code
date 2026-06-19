#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "drone_fsm.h"
#include "drone_receiver.h"
#include "drone_mpu9250.h"
#include "drone_mc_controller.h"
#include "drone_webserver.h"

LOG_MODULE_REGISTER(drone_fsm, LOG_LEVEL_INF);

/* ── Tuning ────────────────────────────────────────────────────────────
 *
 * ARM_CONFIRM_MS      : pilot must hold throttle=0 + drone_active=true
 *                       for this long before the drone arms (anti-accident)
 *
 * FAILSAFE_TIMEOUT_MS : after signal loss, wait this long for recovery
 *                       before triggering auto-landing
 *
 * LANDING_STEP_MS     : interval between each throttle decrement in LANDING
 * LANDING_STEP_PCT    : % of throttle dropped each step
 *                       total landing time ≈ (100/STEP) * STEP_MS
 *
 * THROTTLE_FLY_THRESHOLD : raw y_left (0-1023) above which the drone is
 *                          considered "flying" and RC mixing is applied
 *
 * FAILSAFE_HOLD_PCT   : constant throttle (%) applied while waiting for
 *                       signal recovery in FAILSAFE — tune to approximate
 *                       the hover point of your drone
 * ──────────────────────────────────────────────────────────────────── */
#define ARM_CONFIRM_MS          500U
#define FAILSAFE_TIMEOUT_MS    2000U
#define LANDING_STEP_MS         200U
#define LANDING_STEP_PCT          2U
#define THROTTLE_FLY_THRESHOLD   50
#define FAILSAFE_HOLD_PCT        25U

/* ── Module state ─────────────────────────────────────────────────── */
static drone_state_t current_state = DRONE_STATE_INIT;
static int64_t       state_enter_ms;

/* LANDING context */
static uint8_t  landing_throttle;
static int64_t  last_landing_step_ms;

/* ── Internal helpers ─────────────────────────────────────────────── */

static void enter_state(drone_state_t next)
{
    LOG_INF("FSM: %s -> %s",
            drone_fsm_state_name(current_state),
            drone_fsm_state_name(next));
    current_state  = next;
    state_enter_ms = k_uptime_get();
}

/* Arm condition: remote is on AND throttle stick at minimum */
static bool arm_condition(const struct DataPackage *rc)
{
    return rc->drone_active && (rc->y_left <= 0);
}

/* ── State handlers ───────────────────────────────────────────────── */

static void state_idle(const struct DataPackage *rc)
{
    motors_set_all(0);

    if (arm_condition(rc)) {
        enter_state(DRONE_STATE_ARMING);
    }
}

static void state_arming(const struct DataPackage *rc)
{
    motors_set_all(0);

    if (!arm_condition(rc)) {
        /* Pilot moved throttle or signal dropped — abort */
        enter_state(DRONE_STATE_IDLE);
        return;
    }

    if ((k_uptime_get() - state_enter_ms) >= ARM_CONFIRM_MS) {
        LOG_INF("Drone ARMED — raise throttle to fly");
        enter_state(DRONE_STATE_ARMED);
    }
}

static void state_armed(const struct DataPackage *rc)
{
    motors_set_all(0);

    if (!rc->drone_active) {
        LOG_INF("RC lost while ARMED — disarming");
        enter_state(DRONE_STATE_IDLE);
        return;
    }

    if (rc->y_left > THROTTLE_FLY_THRESHOLD) {
        enter_state(DRONE_STATE_FLYING);
    }
}

static void state_flying(const struct DataPackage *rc)
{
    if (!rc->drone_active) {
        LOG_WRN("Signal lost in flight — FAILSAFE");
        enter_state(DRONE_STATE_FAILSAFE);
        return;
    }

    /* Throttle back to zero: pilot landed */
    if (rc->y_left <= 0) {
        motors_set_all(0);
        enter_state(DRONE_STATE_ARMED);
        return;
    }

    motors_set_from_rc(rc);
}

static void state_failsafe(const struct DataPackage *rc)
{
    if (rc->drone_active) {
        LOG_INF("Signal recovered — resuming FLYING");
        enter_state(DRONE_STATE_FLYING);
        return;
    }

    if ((k_uptime_get() - state_enter_ms) >= FAILSAFE_TIMEOUT_MS) {
        LOG_WRN("Failsafe timeout — auto-landing");
        landing_throttle  = FAILSAFE_HOLD_PCT;
        last_landing_step_ms = k_uptime_get();
        enter_state(DRONE_STATE_LANDING);
        return;
    }

    /* Hold a low throttle while waiting for signal recovery.
     * NOTE: without a PID loop this is open-loop — tune FAILSAFE_HOLD_PCT
     * to your drone's approximate hover point so it descends slowly. */
    motors_set_all(FAILSAFE_HOLD_PCT);
}

static void state_landing(const struct DataPackage *rc)
{
    (void)rc;

    if (landing_throttle == 0) {
        motors_set_all(0);
        LOG_INF("Landing complete");
        enter_state(DRONE_STATE_IDLE);
        return;
    }

    int64_t now = k_uptime_get();
    if ((now - last_landing_step_ms) >= LANDING_STEP_MS) {
        if (landing_throttle > LANDING_STEP_PCT) {
            landing_throttle -= LANDING_STEP_PCT;
        } else {
            landing_throttle = 0;
        }
        motors_set_all(landing_throttle);
        last_landing_step_ms = now;
    }
}

static void state_error(const struct DataPackage *rc)
{
    (void)rc;
    motors_set_all(0);
    /* Stay here until hardware reset */
}

/* ── Public API ───────────────────────────────────────────────────── */

drone_state_t drone_fsm_get_state(void)
{
    return current_state;
}

const char *drone_fsm_state_name(drone_state_t state)
{
    switch (state) {
    case DRONE_STATE_INIT:     return "INIT";
    case DRONE_STATE_IDLE:     return "IDLE";
    case DRONE_STATE_ARMING:   return "ARMING";
    case DRONE_STATE_ARMED:    return "ARMED";
    case DRONE_STATE_FLYING:   return "FLYING";
    case DRONE_STATE_FAILSAFE: return "FAILSAFE";
    case DRONE_STATE_LANDING:  return "LANDING";
    case DRONE_STATE_ERROR:    return "ERROR";
    default:                   return "UNKNOWN";
    }
}

void drone_fsm_run(void)
{
    /* ── Hardware init ───────────────────────────────────────────── */
    nrf_driver_init();

    if (motor_esc_init() < 0) {
        LOG_ERR("ESC init failed");
        enter_state(DRONE_STATE_ERROR);
        goto loop;
    }

    if (mpu9250_driver_init() < 0) {
        LOG_ERR("MPU9250 init failed");
        motors_set_all(0);
        enter_state(DRONE_STATE_ERROR);
        goto loop;
    }

    /* Start WiFi AP + HTTP control server in background */
    drone_webserver_start();

    enter_state(DRONE_STATE_IDLE);

loop:
    /* ── Main loop at MPU9250_LOOP_HZ (250 Hz) ──────────────────── */
    while (1) {
        int64_t t_start = k_uptime_get();

        if (mpu9250_update() < 0 && current_state != DRONE_STATE_ERROR) {
            LOG_ERR("IMU read error — ERROR");
            motors_set_all(0);
            enter_state(DRONE_STATE_ERROR);
        }

        /* NRF24 RC packet (last received, never cleared on signal loss) */
        struct DataPackage rc = read_data();

        /* WiFi RC overrides NRF24 when the web interface is active.
         * drone_webserver_get_rc() returns active=false if no POST
         * was received in the last 500 ms (stale guard).            */
        struct DataPackage wifi_rc = drone_webserver_get_rc();
        if (wifi_rc.drone_active) {
            rc = wifi_rc;
        }

        switch (current_state) {
        case DRONE_STATE_INIT:
        case DRONE_STATE_IDLE:     state_idle(&rc);     break;
        case DRONE_STATE_ARMING:   state_arming(&rc);   break;
        case DRONE_STATE_ARMED:    state_armed(&rc);    break;
        case DRONE_STATE_FLYING:   state_flying(&rc);   break;
        case DRONE_STATE_FAILSAFE: state_failsafe(&rc); break;
        case DRONE_STATE_LANDING:  state_landing(&rc);  break;
        case DRONE_STATE_ERROR:    state_error(&rc);    break;
        }

        /* Deadline sleep to maintain 250 Hz */
        int64_t elapsed = k_uptime_get() - t_start;
        int32_t sleep   = MPU9250_LOOP_MS - (int32_t)elapsed;
        if (sleep > 0) {
            k_sleep(K_MSEC(sleep));
        }
    }
}
