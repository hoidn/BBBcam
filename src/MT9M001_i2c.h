#ifndef MT9M001_I2C_H
#define MT9M001_I2C_H

#include<stdint.h> 


#define MT9M001_CHIP_VERSION            0x00
#define MT9M001_ROW_START               0x01
#define MT9M001_COLUMN_START            0x02
#define MT9M001_WINDOW_HEIGHT           0x03
#define MT9M001_WINDOW_WIDTH            0x04
#define MT9M001_HORIZONTAL_BLANKING     0x05
#define MT9M001_VERTICAL_BLANKING       0x06
#define MT9M001_OUTPUT_CONTROL          0x07
#define MT9M001_SHUTTER_WIDTH           0x09
#define MT9M001_FRAME_RESTART           0x0b
#define MT9M001_SHUTTER_DELAY           0x0c
#define MT9M001_RESET                   0x0d
#define MT9M001_READ_OPTIONS1           0x1e
#define MT9M001_READ_OPTIONS2           0x20
#define MT9M001_GLOBAL_GAIN             0x35
#define MT9M001_CHIP_ENABLE             0xF1
#define MT9M001_BLACK_LEVEL             0x62

#define MT9M001_MAX_WIDTH               1280
#define MT9M001_MAX_HEIGHT              1024
#define MT9M001_MIN_WIDTH               48
#define MT9M001_MIN_HEIGHT              32
#define MT9M001_COLUMN_SKIP             20
#define MT9M001_ROW_SKIP                12


// struct to store reg address and corresponding value
typedef struct addr_val {
    uint8_t addr;
    uint16_t val;
} AddrVal;

// variable declarations
extern const AddrVal params_1280x1024_trigger[];
extern const AddrVal params_1280x1024_continuous[];

// Function declarations
void sensors_ADC_init(void);  
void write16(uint8_t regAddr, uint16_t value) ; 
void read16(uint8_t regAddr);  
// initialize i2c interface and configure sensor fr single capture mode
void init_readout(); 
int delay_ms(unsigned int msec);


#endif
