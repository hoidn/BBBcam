#ifndef CAMCTRL_H
#define CAMCTRL_H

#include<stdint.h> 

#define FRAMES_PER_TRANSFER 1
#define MAXVALUE 256

typedef struct{
    uint32_t threshold;
    uint32_t nexposures;
    uint32_t gain;
    uint32_t configure;
    uint32_t lower_bound;
    uint32_t upper_bound;
} clientCommands;

void exposure(uint8_t *frameptr, int framesize);
void pru_exit(void);
void pru_start(void);
int config_pru(int initial_config, uint32_t numFrames);
void wait_pru1_complete(void);
void send_pru_ready_signal();
#endif
