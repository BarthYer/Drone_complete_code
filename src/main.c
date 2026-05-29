#include <zephyr/kernel.h>
#include <stdio.h>
#include "drone_receiver.h"
#include "drone_mpu9250.h"
//#include "drone_mc_controller.h"


#include <zephyr/kernel.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(esc_4motors, LOG_LEVEL_INF);

/* ── PWM device ─────────────────────────────────────────────────────── */
#define ESC_PWM_NODE  DT_NODELABEL(mcpwm0)

/* ── Channels (match overlay pin mapping) ───────────────────────────── */
#define MOTOR_1_CH  0   /* GPIO 18 — Timer0 / Op0 / PWM0A */
#define MOTOR_2_CH  1   /* GPIO 17 — Timer0 / Op0 / PWM0B */
#define MOTOR_3_CH  2   /* GPIO 16 — Timer1 / Op1 / PWM1A */
#define MOTOR_4_CH  3   /* GPIO 15 — Timer1 / Op1 / PWM1B */
#define NUM_MOTORS  4

/* ── Timing (nanoseconds) ───────────────────────────────────────────── */
#define ESC_PERIOD_NS        20000000U  /* 20 ms  = 50 Hz         */
#define ESC_PULSE_MIN_NS      1000000U  /* 1000 µs = disarmed     */
#define ESC_PULSE_MAX_NS      2000000U  /* 2000 µs = full throttle*/
#define ESC_ARM_MS               3000U
#define ESC_CALIBRATE               0  /* 1 = lancer la calibration au démarrage */
#define ESC_SCAN_THRESHOLD          0  /* 1 = scanner le seuil de chaque moteur   */

//#define ESC_SCAN_THRESHOLD 1

/* Motor channel table */
static const uint8_t motor_ch[NUM_MOTORS] = {
    MOTOR_1_CH, MOTOR_2_CH, MOTOR_3_CH, MOTOR_4_CH
};

static const struct device *esc_pwm;

/* ── Low-level helpers ──────────────────────────────────────────────── */

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

/**
 * @brief  Set throttle as a percentage 0–100 % for one motor.
 *         0 % → 1000 µs,  100 % → 2000 µs
 */
int motor_set_throttle(uint8_t motor_idx, uint8_t percent)
{
    if (percent > 100U) { percent = 100U; }

    uint32_t pulse_us = 1000U + (uint32_t)percent * 10U;
    LOG_INF("Motor %u → %3u%%  (%u µs)",
            motor_idx + 1, percent, pulse_us);
    return motor_set_pulse_us(motor_idx, pulse_us);
}

/**
 * @brief  Set the same throttle on all 4 motors at once.
 */
int motors_set_all(uint8_t percent)
{
    int ret = 0;
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        ret |= motor_set_throttle(i, percent);
    }
    return ret;
}

/* ── Threshold scan ─────────────────────────────────────────────────── */

static void find_motor_threshold(uint8_t ch)
{
    LOG_INF("--- Scan seuil canal %u --- regardez le moteur", ch);
    for (uint32_t pulse = 1000U; pulse <= 1300U; pulse += 5U) {
        pwm_set(esc_pwm, ch, ESC_PERIOD_NS,
                pulse * 1000U, PWM_POLARITY_NORMAL);
        LOG_INF("Pulse: %u us", pulse);
        k_sleep(K_MSEC(500));
    }
    /* Arrêt */
    pwm_set(esc_pwm, ch, ESC_PERIOD_NS,
            ESC_PULSE_MIN_NS, PWM_POLARITY_NORMAL);
    LOG_INF("--- Fin scan canal %u ---", ch);
    k_sleep(K_MSEC(2000));
}

static void scan_all_thresholds(void)
{
    LOG_INF("=== SCAN SEUILS MOTEURS ===");
    LOG_INF("Notez la valeur 'Pulse: XXX us' au moment ou chaque moteur demarre.");
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        find_motor_threshold(motor_ch[2]);
    }
    LOG_INF("=== Scan termine ===");
}

/* ── Calibration sequence ───────────────────────────────────────────── */

static int esc_calibrate_all(void)
{
    LOG_INF("=== CALIBRATION ESC ===");
    LOG_INF("1) Coupez l'alimentation des ESC maintenant.");
    LOG_INF("   Signal MAX (2000 us) envoye — vous avez 7 secondes pour rebrancher.");

    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        pwm_set(esc_pwm, motor_ch[i], ESC_PERIOD_NS,
                ESC_PULSE_MAX_NS, PWM_POLARITY_NORMAL);
    }
    /* Fenetre pour allumer les ESC : attendez les bips de reconnaissance du MAX */
    k_sleep(K_MSEC(7000));

    LOG_INF("2) Signal MIN (1000 us) envoye — attendez les bips de confirmation.");
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        pwm_set(esc_pwm, motor_ch[i], ESC_PERIOD_NS,
                ESC_PULSE_MIN_NS, PWM_POLARITY_NORMAL);
    }
    /* Attendre les bips de fin de calibration */
    k_sleep(K_MSEC(4000));

    LOG_INF("=== Calibration terminee — tous les ESC armed ===");
    return 0;
}

/* ── Arming sequence ────────────────────────────────────────────────── */

static int esc_arm_all(void)
{
    LOG_INF("Arming all ESCs — sending 1000 µs for %u ms ...",
            ESC_ARM_MS);
       
        motor_set_pulse_us(1, 1000U);
       /* motor_set_pulse_us(2, 1000U);
        motor_set_pulse_us(3, 1000U);
        motor_set_pulse_us(0, 1000U);*/
        /*
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        int ret = motor_set_pulse_us(i, 1000U);
        if (ret) {
            LOG_ERR("Arm failed on motor %u", i + 1);
            return ret;
        }
    }*/

    k_sleep(K_MSEC(ESC_ARM_MS));
    LOG_INF("All ESCs armed.");
    return 0;
}

/* ── Demo ───────────────────────────────────────────────────────────── */

static void demo_ramp(void)
{
    LOG_INF("Demo: ramp up all motors 0 → 30 %%");
    for (uint8_t pct = 0; pct <= 30; pct += 5) {
        motors_set_all(pct);
        k_sleep(K_MSEC(400));
    }

    k_sleep(K_MSEC(1000));

    LOG_INF("Demo: ramp down all motors 30 → 0 %%");
    for (int pct = 30; pct >= 0; pct -= 5) {
        motors_set_all((uint8_t)pct);
        k_sleep(K_MSEC(400));
    }

    /* Hard stop */
    motors_set_all(0);
    LOG_INF("Demo complete — all motors stopped.");
}




int main(void)
{
    /* ---- Init NRF24L01 receiver ---- */
    nrf_driver_init();


    esc_pwm = DEVICE_DT_GET(ESC_PWM_NODE);
    if (!device_is_ready(esc_pwm)) {
        LOG_ERR("PWM device not ready!");
        return -ENODEV;
    }
#if ESC_CALIBRATE
    esc_calibrate_all();
#elif ESC_SCAN_THRESHOLD
    esc_arm_all();
    scan_all_thresholds();
#else
    esc_arm_all();
#endif


    /* ---- Init MPU9250 IMU (board must be still during calibration) ---- */
    /*if (mpu9250_driver_init() < 0) {
        printf("ERROR: MPU9250 init failed\n");
        return -1;
    }*/

    /* ---- Init ESCs and arm all 4 motors ---- */
    /*if (motor_esc_init() < 0) {
        printf("ERROR: ESC init failed\n");
        return -1;
    }*/

    uint32_t print_cnt = 0;

    //motors_set_all(0);
    
   

    while (1) {
        int64_t t_start = k_uptime_get();
        //motors_set_all(25);
        //motor_set_throttle(3, 1200);
        /* ---- Update IMU (250 Hz complementary filter) ---- */
        /*if (mpu9250_update() < 0) {
            printf("ERROR: IMU read failed\n");
        }*/
        
        /* ---- Read latest RC packet ---- */
        //struct DataPackage rc = read_data();
        for (int i = 5; i < 45; i += 5) {
            motor_set_pulse_us(MOTOR_1_CH, 1000U + i * 10U); //motor 2
            motor_set_pulse_us(MOTOR_2_CH, 1000U + i * 10U); //motor 4
            motor_set_pulse_us(MOTOR_3_CH, 1000U + i * 10U);  //motor 1
            motor_set_pulse_us(MOTOR_4_CH, 1000U + i * 10U); //mootor 3
            k_sleep(K_SECONDS(1));
        }

        /*for (int i = 95; i >= 0; i -= 5) {
            motor_set_pulse_us(MOTOR_1_CH, 1000U + i * 10U);
            motor_set_pulse_us(MOTOR_2_CH, 1000U + i * 10U);
            motor_set_pulse_us(MOTOR_3_CH, 1000U + i * 10U);
            motor_set_pulse_us(MOTOR_4_CH, 1000U + i * 10U);
            k_sleep(K_SECONDS(1));
        }*/
        /*printf("RC  x_right:%5d  y_right:%5d  x_left:%5d  y_left:%5d  active:%d\n",
                   rc.x_right, rc.y_right, rc.x_left, rc.y_left,
                   (int)rc.drone_active);*/
        /* ---- Drive all 4 motors from RC commands ---- */
        //motors_set_from_rc(&rc);

        /* ---- Print telemetry at 10 Hz ---- */
        /*if (++print_cnt >= MPU9250_LOOP_HZ / 10) {
            print_cnt = 0;

            printf("RC  x_right:%5d  y_right:%5d  x_left:%5d  y_left:%5d  active:%d\n",
                   rc.x_right, rc.y_right, rc.x_left, rc.y_left,
                   (int)rc.drone_active);

            printf("IMU Roll:%6.2f deg  Pitch:%6.2f deg  |"
                   "  Gyro(dps): %5.1f %5.1f %5.1f\n",
                   att.roll_deg,  att.pitch_deg,
                   gyro_x_dps,   gyro_y_dps,   gyro_z_dps);
        }*/

        /* ---- Deadline sleep: keep loop at exactly MPU9250_LOOP_HZ ---- */
        /*int64_t elapsed_ms = k_uptime_get() - t_start;
        int32_t sleep_ms   = MPU9250_LOOP_MS - (int32_t)elapsed_ms;*/
       /* if (sleep_ms > 0) {
            k_sleep(K_MSEC(sleep_ms));
        }*/
        //k_sleep(K_MSEC(1));
        k_sleep(K_SECONDS(1));
    }

    return 0;
}
