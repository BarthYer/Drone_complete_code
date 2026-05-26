#ifndef DRONE_RECEIVER_H
#define DRONE_RECEIVER_H

#include <stdint.h>
#include <stdbool.h>

/* Shared data structure — must match the transmitter side exactly */
struct DataPackage {
    int16_t x_right;
    int16_t y_right;
    int16_t x_left;
    int16_t y_left;
    bool    drone_active;
} __attribute__((packed));  /* packed: sizeof = 9 on all platforms */

/* Global latest received packet — updated by read_data() */
extern struct DataPackage data;

/* Public API */
void               nrf_driver_init(void);   /* call once from main() */
struct DataPackage read_data(void);

#endif /* DRONE_RECEIVER_H */
