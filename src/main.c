#include <zephyr/kernel.h>
#include "drone_receiver.h"

int main(void)
{
    nrf_driver_init();

    while (1) {
       struct DataPackage data = read_data();

        printk("x_right: %d | y_right: %d | x_left: %d | y_left: %d | active: %d\n",
               data.x_right,
               data.y_right,
               data.x_left,
               data.y_left,
               (int)data.drone_active);

        k_sleep(K_MSEC(100));
    }

    return 0;
}
