#include <zephyr/kernel.h>
#include "drone_fsm.h"

/* Give the FSM its own clean stack — no Zephyr pre-main overhead here.
 * 16 KB accommodates I2C/SPI driver call chains + interrupt frames. */
#define FSM_STACK_SIZE  16384
K_THREAD_STACK_DEFINE(fsm_stack, FSM_STACK_SIZE);
static struct k_thread fsm_thread;

static void fsm_entry(void *a, void *b, void *c)
{
    drone_fsm_run();
}

int main(void)
{
    k_thread_create(&fsm_thread, fsm_stack,
                    K_THREAD_STACK_SIZEOF(fsm_stack),
                    fsm_entry, NULL, NULL, NULL,
                    5, 0, K_NO_WAIT);
    k_thread_name_set(&fsm_thread, "drone_fsm");
    return 0;
}
