#ifndef DRONE_MPU9250_H
#define DRONE_MPU9250_H

/* ------------------------------------------------------------------ */
/* Attitude type                                                         */
/* ------------------------------------------------------------------ */
typedef struct {
    float roll_deg;    /* rotation around X axis (bank left/right) */
    float pitch_deg;   /* rotation around Y axis (nose up/down)    */
} attitude_t;

/* ------------------------------------------------------------------ */
/* Global outputs — updated every mpu9250_update() call               */
/* ------------------------------------------------------------------ */

/* Latest attitude angles from the complementary filter */
extern attitude_t att;

/* Latest calibrated gyro rates (degrees / second) */
extern float gyro_x_dps;
extern float gyro_y_dps;
extern float gyro_z_dps;

/* ------------------------------------------------------------------ */
/* Loop timing — use these in main.c to schedule mpu9250_update()     */
/* ------------------------------------------------------------------ */
#define MPU9250_LOOP_HZ   250
#define MPU9250_LOOP_MS   (1000 / MPU9250_LOOP_HZ)   /* 4 ms */

/* ------------------------------------------------------------------ */
/* Public API                                                           */
/* ------------------------------------------------------------------ */

/**
 * @brief Initialise I2C bus, MPU9250 registers, and calibrate gyro bias.
 *        The board must be completely stationary during calibration (~1 s).
 * @return  0 on success, -1 on hardware error.
 */
int mpu9250_driver_init(void);

/**
 * @brief Read sensor, apply accel LPF + complementary filter,
 *        and update the att / gyro_*_dps globals.
 *        Call every MPU9250_LOOP_MS milliseconds.
 * @return  0 on success, -1 on I2C read error.
 */
int mpu9250_update(void);

#endif /* DRONE_MPU9250_H */
