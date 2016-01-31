#ifndef I2C_H
#define I2C_H

#include<stdint.h> 



// struct to store reg address and corresponding value
typedef struct addr_val {
    uint16_t addr;
    uint8_t val;
} AddrVal;


// Function declarations
int delay_ms(unsigned int msec);
void sensors_ADC_init(uint16_t addr);  
void write8(uint16_t regAddr, uint8_t value); 
uint16_t read8(uint16_t regAddr, uint8_t dev_addr);
int delay_ms(unsigned int msec);
int i2c_writeArr(AddrVal *regStructArr);


#endif
