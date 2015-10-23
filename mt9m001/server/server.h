#ifndef SERVER_H
#define SERVER_H

typedef struct{
    uint32_t threshold;
    uint32_t nexposures;
    uint32_t gain;
    uint32_t configure;
    uint32_t lower_bound;
    uint32_t upper_bound;
} clientCommands;

#endif
