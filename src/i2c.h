#ifndef I2C_H
#define I2C_H

#include<stdint.h> 



// struct to store reg address and corresponding value
typedef struct addr_val {
    uint8_t addr;
    uint16_t val;
} AddrVal;


// Function declarations
int delay_ms(unsigned int msec);
void sensors_ADC_init(uint16_t addr);  
void write16(uint8_t regAddr, uint16_t value); 
uint16_t read16(char *regAddr, const char *dev_addr);  
int delay_ms(unsigned int msec);
int i2c_writeArr(AddrVal *regStructArr);


#endif
