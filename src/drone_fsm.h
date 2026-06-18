#ifndef DRONE_FSM_H
#define DRONE_FSM_H

typedef enum {
    DRONE_STATE_INIT      = 0,
    DRONE_STATE_IDLE      = 1,
    DRONE_STATE_ARMING    = 2,
    DRONE_STATE_ARMED     = 3,
    DRONE_STATE_FLYING    = 4,
    DRONE_STATE_FAILSAFE  = 5,
    DRONE_STATE_LANDING   = 6,
    DRONE_STATE_ERROR     = 7,
} drone_state_t;

/* Start the FSM — never returns */
void drone_fsm_run(void);

/* Read current state */
drone_state_t drone_fsm_get_state(void);

/* Human-readable name for logging */
const char *drone_fsm_state_name(drone_state_t state);

#endif /* DRONE_FSM_H */
