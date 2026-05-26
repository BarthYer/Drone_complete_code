/*
 * MPU9250 — FPV Drone IMU
 * Zephyr RTOS
 *
 * Features:
 *  - Proper sensor init (DLPF, sample rate, full-scale)
 *  - Gyro static-bias calibration at startup (keep board still!)
 *  - Exponential moving average (EMA) on accelerometer to kill vibrations
 *  - Complementary filter → stable Roll / Pitch in degrees (250 Hz)
 *  - Non-blocking timing loop (deadline based)
 *
 * NOT included yet (next steps for full flight controller):
 *  - Yaw estimation (needs AK8963 magnetometer or GPS heading)
 *  - PID controller
 *  - ESC/motor PWM output
 *  - RC receiver input
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include "drone_mpu9250.h"

/* M_PI is a POSIX extension, not guaranteed by standard C             */
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ------------------------------------------------------------------ */
/* MPU9250 register map                                                 */
/* ------------------------------------------------------------------ */
#define MPU9250_ADDR        0x68

#define REG_SMPLRT_DIV      0x19   /* Sample rate divider              */
#define REG_CONFIG          0x1A   /* DLPF config                      */
#define REG_GYRO_CONFIG     0x1B   /* Gyro full-scale                  */
#define REG_ACCEL_CONFIG    0x1C   /* Accel full-scale                 */
#define REG_ACCEL_CONFIG2   0x1D   /* Accel DLPF                       */
#define REG_ACCEL_OUT       0x3B   /* First data register (14 bytes)   */
#define REG_PWR_MGMT_1      0x6B   /* Power management                 */

/* ------------------------------------------------------------------ */
/* Sensor full-scale & conversion                                       */
/*   ACCEL_FS_SEL = 0 → ±2 g    → 16384 LSB/g                        */
/*   GYRO_FS_SEL  = 0 → ±250 °/s → 131 LSB/(°/s)                     */
/* ------------------------------------------------------------------ */
#define ACCEL_FS_SEL        0
#define GYRO_FS_SEL         0
#define ACCEL_SCALE         16384.0f
#define GYRO_SCALE          131.0f

/* ------------------------------------------------------------------ */
/* Loop timing                                                          */
/* ------------------------------------------------------------------ */
#define LOOP_HZ             250             /* control-loop frequency   */
#define LOOP_MS             (1000 / LOOP_HZ)/* 4 ms per iteration       */
#define DT                  (1.0f / LOOP_HZ)/* seconds                  */

/* ------------------------------------------------------------------ */
/* Complementary filter weight                                          */
/*   0.98 → trust gyro 98 %, correct with accel 2 % each step         */
/*   Increase toward 1.0 if you see drift; decrease if vibration noise */
/* ------------------------------------------------------------------ */
#define CF_ALPHA            0.98f

/* ------------------------------------------------------------------ */
/* Accelerometer exponential moving average (EMA / low-pass filter)    */
/*   LPF_ALPHA = 1.0 → no filter; 0.1 → heavy smoothing               */
/*   For 250 Hz with motor vibrations: 0.1 – 0.2 is a good start      */
/* ------------------------------------------------------------------ */
#define LPF_ALPHA           0.15f

/* ------------------------------------------------------------------ */
/* Gyro calibration                                                     */
/* ------------------------------------------------------------------ */
#define CALIB_SAMPLES       500    /* ~1 s at 2 ms per sample          */

/* ------------------------------------------------------------------ */
/* Print throttle: print every N control-loop cycles                   */
/*   LOOP_HZ / 10 → 10 Hz output (readable on serial terminal)        */
/* ------------------------------------------------------------------ */
#define PRINT_EVERY         (LOOP_HZ / 10)

#define I2C_NODE            DT_NODELABEL(i2c0)

/* ------------------------------------------------------------------ */
/* Globals                                                              */
/* ------------------------------------------------------------------ */
static const struct device *i2c_dev;

/* Internal calibration state */
static float gyro_bias_x = 0.0f;
static float gyro_bias_y = 0.0f;
static float gyro_bias_z = 0.0f;

/* Internal EMA state */
static float lpf_ax = 0.0f;
static float lpf_ay = 0.0f;
static float lpf_az = 0.0f;

/* Public globals — declared extern in drone_mpu9250.h */
attitude_t att        = {0.0f, 0.0f};
float      gyro_x_dps = 0.0f;
float      gyro_y_dps = 0.0f;
float      gyro_z_dps = 0.0f;

/* ================================================================== */
/* I2C helpers                                                          */
/* ================================================================== */
static int write_reg(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    return i2c_write(i2c_dev, buf, 2, MPU9250_ADDR);
}

static int read_regs(uint8_t reg, uint8_t *data, uint8_t len)
{
    return i2c_write_read(i2c_dev, MPU9250_ADDR, &reg, 1, data, len);
}

/* ================================================================== */
/* MPU9250 init                                                         */
/* ================================================================== */
static int mpu9250_init(void)
{
    /*
     * PWR_MGMT_1 = 0x01
     *   SLEEP=0, CYCLE=0, CLKSEL=1 (PLL with X-gyro reference)
     *   Using PLL gives better clock stability than internal oscillator.
     */
    if (write_reg(REG_PWR_MGMT_1, 0x01) < 0) {
        return -1;
    }
    k_sleep(K_MSEC(15)); /* wait for PLL to lock */

    /*
     * SMPLRT_DIV = 0
     *   Output data rate = Gyro rate / (1 + SMPLRT_DIV)
     *   With DLPF enabled gyro rate = 1 kHz → ODR = 1000 Hz
     *   We run our loop at 250 Hz, so the FIFO always has fresh data.
     */
    if (write_reg(REG_SMPLRT_DIV, 0x00) < 0) return -1;

    /*
     * CONFIG = 0x03
     *   DLPF_CFG = 3 → Gyro BW 41 Hz, delay 5.9 ms; Temp BW 42 Hz
     *   Removes high-frequency motor vibration noise from gyro path.
     */
    if (write_reg(REG_CONFIG, 0x03) < 0) return -1;

    /*
     * GYRO_CONFIG: GYRO_FS_SEL = 0 → ±250 °/s
     *   Good for hover / slow manoeuvres; increase to FS=1 (±500)
     *   for more aggressive flying.
     */
    if (write_reg(REG_GYRO_CONFIG, GYRO_FS_SEL << 3) < 0) return -1;

    /*
     * ACCEL_CONFIG: ACCEL_FS_SEL = 0 → ±2 g
     */
    if (write_reg(REG_ACCEL_CONFIG, ACCEL_FS_SEL << 3) < 0) return -1;

    /*
     * ACCEL_CONFIG2: A_DLPF_CFG = 3 → BW 41 Hz, delay 11.8 ms
     *   Matches gyro DLPF to keep both sensors time-aligned.
     */
    if (write_reg(REG_ACCEL_CONFIG2, 0x03) < 0) return -1;

    return 0;
}

/* ================================================================== */
/* Read raw accel + gyro (14 bytes = 6 accel + 2 temp + 6 gyro)       */
/* ================================================================== */
static int read_raw(int16_t *ax, int16_t *ay, int16_t *az,
                    int16_t *gx, int16_t *gy, int16_t *gz)
{
    uint8_t raw[14];

    if (read_regs(REG_ACCEL_OUT, raw, sizeof(raw)) < 0) {
        return -1;
    }

    *ax = (int16_t)((raw[0]  << 8) | raw[1]);
    *ay = (int16_t)((raw[2]  << 8) | raw[3]);
    *az = (int16_t)((raw[4]  << 8) | raw[5]);
    /* raw[6..7] = temperature — ignored here */
    *gx = (int16_t)((raw[8]  << 8) | raw[9]);
    *gy = (int16_t)((raw[10] << 8) | raw[11]);
    *gz = (int16_t)((raw[12] << 8) | raw[13]);

    return 0;
}

/* ================================================================== */
/* Gyro calibration                                                     */
/*   Must be called while the drone is completely stationary.          */
/*   Averages CALIB_SAMPLES readings to find the zero-rate offset.     */
/* ================================================================== */
static void calibrate_gyro(void)
{
    printf("Calibration gyro — ne pas bouger !\n");

    double sum_x = 0.0, sum_y = 0.0, sum_z = 0.0;
    int16_t ax, ay, az, gx, gy, gz;
    int good = 0;

    for (int i = 0; i < CALIB_SAMPLES; i++) {
        if (read_raw(&ax, &ay, &az, &gx, &gy, &gz) == 0) {
            sum_x += gx;
            sum_y += gy;
            sum_z += gz;
            good++;
        }
        k_sleep(K_MSEC(2));
    }

    if (good == 0) {
        printf("ERREUR calibration — aucune donnee valide\n");
        return;
    }

    gyro_bias_x = (float)(sum_x / good) / GYRO_SCALE;
    gyro_bias_y = (float)(sum_y / good) / GYRO_SCALE;
    gyro_bias_z = (float)(sum_z / good) / GYRO_SCALE;

    printf("Biais gyro (dps): X=%.4f  Y=%.4f  Z=%.4f\n",
           gyro_bias_x, gyro_bias_y, gyro_bias_z);
}

/* ================================================================== */
/* Accelerometer exponential moving average (low-pass filter)          */
/*   Attenuates high-frequency motor/frame vibrations.                 */
/*   y[n] = y[n-1] + alpha * (x[n] - y[n-1])                         */
/* ================================================================== */
static void accel_lpf(float ax, float ay, float az,
                      float *out_ax, float *out_ay, float *out_az)
{
    lpf_ax += LPF_ALPHA * (ax - lpf_ax);
    lpf_ay += LPF_ALPHA * (ay - lpf_ay);
    lpf_az += LPF_ALPHA * (az - lpf_az);

    *out_ax = lpf_ax;
    *out_ay = lpf_ay;
    *out_az = lpf_az;
}

/* ================================================================== */
/* Complementary filter                                                 */
/*                                                                      */
/*  Gyro is accurate short-term but drifts over time.                  */
/*  Accel gives absolute angle reference but is noisy under vibration. */
/*  Complementary filter blends both to get stable long-term angles    */
/*  with fast dynamic response.                                         */
/*                                                                      */
/*  roll_acc  = atan2(ay, az)                  (X-axis tilt)           */
/*  pitch_acc = atan2(-ax, sqrt(ay²+az²))      (Y-axis tilt)          */
/*                                                                      */
/*  roll  = alpha*(roll  + gx*dt) + (1-alpha)*roll_acc                 */
/*  pitch = alpha*(pitch + gy*dt) + (1-alpha)*pitch_acc                */
/* ================================================================== */
static void complementary_filter(float ax_f, float ay_f, float az_f,
                                  float gx_f, float gy_f,
                                  attitude_t *att)
{
    /* Accel-derived tilt (degrees) */
    float roll_acc  =  atan2f(ay_f, az_f)
                       * (180.0f / (float)M_PI);
    float pitch_acc =  atan2f(-ax_f, sqrtf(ay_f * ay_f + az_f * az_f))
                       * (180.0f / (float)M_PI);

    /* Gyro integration */
    float roll_gyro  = att->roll_deg  + gx_f * DT;
    float pitch_gyro = att->pitch_deg + gy_f * DT;

    /* Fuse: mostly trust the gyro, slowly correct with accel */
    att->roll_deg  = CF_ALPHA * roll_gyro  + (1.0f - CF_ALPHA) * roll_acc;
    att->pitch_deg = CF_ALPHA * pitch_gyro + (1.0f - CF_ALPHA) * pitch_acc;
}

/* ================================================================== */
/* mpu9250_driver_init                                                  */
/* ================================================================== */
int mpu9250_driver_init(void)
{
    i2c_dev = DEVICE_DT_GET(I2C_NODE);

    if (!device_is_ready(i2c_dev)) {
        printf("I2C non pret !\n");
        return -1;
    }

    if (mpu9250_init() < 0) {
        printf("MPU9250 init echouee !\n");
        return -1;
    }
    printf("MPU9250 pret !\n");

    /* Calibrate gyro (board must be stationary) */
    calibrate_gyro();

    /* Seed LPF with first accel reading so the filter starts correctly */
    {
        int16_t ax, ay, az, gx, gy, gz;
        if (read_raw(&ax, &ay, &az, &gx, &gy, &gz) == 0) {
            lpf_ax = (float)ax / ACCEL_SCALE;
            lpf_ay = (float)ay / ACCEL_SCALE;
            lpf_az = (float)az / ACCEL_SCALE;
        }
    }

    return 0;
}

/* ================================================================== */
/* mpu9250_update                                                       */
/*   Call every MPU9250_LOOP_MS ms. Updates att, gyro_*_dps globals.  */
/* ================================================================== */
int mpu9250_update(void)
{
    int16_t ax_r, ay_r, az_r, gx_r, gy_r, gz_r;

    if (read_raw(&ax_r, &ay_r, &az_r, &gx_r, &gy_r, &gz_r) < 0) {
        return -1;
    }

    /* Convert to physical units */
    float ax_f = (float)ax_r / ACCEL_SCALE;
    float ay_f = (float)ay_r / ACCEL_SCALE;
    float az_f = (float)az_r / ACCEL_SCALE;

    /* Subtract calibrated zero-rate offset */
    float gx_f = (float)gx_r / GYRO_SCALE - gyro_bias_x;
    float gy_f = (float)gy_r / GYRO_SCALE - gyro_bias_y;
    float gz_f = (float)gz_r / GYRO_SCALE - gyro_bias_z;

    /* Accel low-pass filter (removes motor vibrations) */
    float ax_lpf, ay_lpf, az_lpf;
    accel_lpf(ax_f, ay_f, az_f, &ax_lpf, &ay_lpf, &az_lpf);

    /* Complementary filter → attitude angles */
    complementary_filter(ax_lpf, ay_lpf, az_lpf, gx_f, gy_f, &att);

    /* Publish gyro rates for main.c */
    gyro_x_dps = gx_f;
    gyro_y_dps = gy_f;
    gyro_z_dps = gz_f;

    return 0;
}
