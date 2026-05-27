/*
 * drone_mc_controller.h — Public API for 4-motor ESC control (MCPWM/PWM)
 *
 * Motor layout (X-frame quadrotor):
 *
 *          FRONT
 *    M1 (CCW) | M2 (CW)
 *    ---------+---------
 *    M3 (CW)  | M4 (CCW)
 *          REAR
 *
 * Channel → GPIO mapping:
 *   Motor 1 (Front-Left)  → GPIO 18
 *   Motor 2 (Front-Right) → GPIO 17
 *   Motor 3 (Rear-Left)   → GPIO 16
 *   Motor 4 (Rear-Right)  → GPIO 15
 */

#ifndef DRONE_MC_CONTROLLER_H
#define DRONE_MC_CONTROLLER_H

#include <stdint.h>
#include "drone_receiver.h"

/* ── Motor indices ──────────────────────────────────────────────────── */
#define MOTOR_FRONT_LEFT    0   /**< Channel 0 — GPIO 18 */
#define MOTOR_FRONT_RIGHT   1   /**< Channel 1 — GPIO 17 */
#define MOTOR_REAR_LEFT     2   /**< Channel 2 — GPIO 16 */
#define MOTOR_REAR_RIGHT    3   /**< Channel 3 — GPIO 15 */

/* ── RC input range (raw int16_t from the transmitter ADC) ─────────── */
#define RC_MIN      0       /**< Joystick minimum value        */
#define RC_MAX      1023    /**< Joystick maximum value        */
#define RC_CENTER   512     /**< Neutral position (roll/pitch/yaw) */

/* ── Motor output limits ────────────────────────────────────────────── */
#define MOTOR_THROTTLE_MAX  80  /**< Max throttle % (headroom for corrections) */
#define MOTOR_MIXING_MAX    20  /**< Max ± correction per axis (%)             */

/* ──────────────────────────────────────────────────────────────────── */
/* Initialisation                                                        */
/* ──────────────────────────────────────────────────────────────────── */

/**
 * @brief  Initialise the PWM device and run the ESC arming sequence.
 *         Must be called once from main() before any motor command.
 * @return 0 on success, negative errno on failure.
 */
int motor_esc_init(void);

/* ──────────────────────────────────────────────────────────────────── */
/* Per-motor and all-motor helpers                                       */
/* ──────────────────────────────────────────────────────────────────── */

/**
 * @brief  Set the throttle of a single motor.
 * @param  motor_idx  Motor index 0–3 (use MOTOR_FRONT_LEFT … macros).
 * @param  percent    Throttle 0–100 %.  0 % → 1000 µs, 100 % → 2000 µs.
 * @return 0 on success, negative errno on failure.
 */
int motor_set_throttle(uint8_t motor_idx, uint8_t percent);

/**
 * @brief  Set the same throttle on all four motors simultaneously.
 * @param  percent  Throttle 0–100 %.
 * @return 0 on success, negative errno on failure (OR of all channels).
 */
int motors_set_all(uint8_t percent);

/* ──────────────────────────────────────────────────────────────────── */
/* RC-command mixing                                                     */
/* ──────────────────────────────────────────────────────────────────── */

/**
 * @brief  Convert RC joystick commands into motor outputs and apply them.
 *
 *         Joystick assignment:
 *           y_left  → Throttle  (0 … RC_MAX)
 *           x_left  → Yaw       (RC_CENTER = neutral)
 *           y_right → Pitch     (RC_CENTER = neutral)
 *           x_right → Roll      (RC_CENTER = neutral)
 *
 *         X-frame quadrotor mixing:
 *           M1 Front-Left  = Throttle + Pitch + Roll − Yaw
 *           M2 Front-Right = Throttle + Pitch − Roll + Yaw
 *           M3 Rear-Left   = Throttle − Pitch + Roll + Yaw
 *           M4 Rear-Right  = Throttle − Pitch − Roll − Yaw
 *
 *         If drone_active is false all motors are stopped immediately.
 *
 * @param  rc  Pointer to the latest received RC packet.
 */
void motors_set_from_rc(const struct DataPackage *rc);

#endif /* DRONE_MC_CONTROLLER_H */
