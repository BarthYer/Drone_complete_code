#include <zephyr/kernel.h>
#include <stdio.h>
#include "drone_receiver.h"
#include "drone_mpu9250.h"
#include "drone_mc_controller.h"

int main(void)
{
    /* ---- Init NRF24L01 receiver ---- */
    nrf_driver_init();

    /* ---- Init MPU9250 IMU (board must be still during calibration) ---- */
    if (mpu9250_driver_init() < 0) {
        printf("ERROR: MPU9250 init failed\n");
        return -1;
    }

    /* ---- Init ESCs and arm all 4 motors ---- */
    if (motor_esc_init() < 0) {
        printf("ERROR: ESC init failed\n");
        return -1;
    }

    uint32_t print_cnt = 0;

    while (1) {
        int64_t t_start = k_uptime_get();
        motors_set_all(25);
        /* ---- Update IMU (250 Hz complementary filter) ---- */
        if (mpu9250_update() < 0) {
            printf("ERROR: IMU read failed\n");
        }

        /* ---- Read latest RC packet ---- */
        struct DataPackage rc = read_data();

        /* ---- Drive all 4 motors from RC commands ---- */
        //motors_set_from_rc(&rc);

        /* ---- Print telemetry at 10 Hz ---- */
        if (++print_cnt >= MPU9250_LOOP_HZ / 10) {
            print_cnt = 0;

            printf("RC  x_right:%5d  y_right:%5d  x_left:%5d  y_left:%5d  active:%d\n",
                   rc.x_right, rc.y_right, rc.x_left, rc.y_left,
                   (int)rc.drone_active);

            printf("IMU Roll:%6.2f deg  Pitch:%6.2f deg  |"
                   "  Gyro(dps): %5.1f %5.1f %5.1f\n",
                   att.roll_deg,  att.pitch_deg,
                   gyro_x_dps,   gyro_y_dps,   gyro_z_dps);
        }

        /* ---- Deadline sleep: keep loop at exactly MPU9250_LOOP_HZ ---- */
        int64_t elapsed_ms = k_uptime_get() - t_start;
        int32_t sleep_ms   = MPU9250_LOOP_MS - (int32_t)elapsed_ms;
        if (sleep_ms > 0) {
            k_sleep(K_MSEC(sleep_ms));
        }
    }

    return 0;
}
