#ifndef PUMP_CONTROL_H
#define PUMP_CONTROL_H

enum PUMP_STATES {
    PUMP_INIT,
    PUMP_PRIME,
    PUMP_ARMED,
    PUMP_FLOW,
    PUMP_HOLDING,
    PUMP_STOPPED
};

typedef struct {
    enum PUMP_STATES state;

    uint16_t target_power;
    uint16_t current_power;
    uint16_t velocity;
} pump_control_data_t;

void init_pump_control();
void request_state(enum PUMP_STATES requested_state);
void tick_pump_control();

#endif // PUMP_CONTROL_H