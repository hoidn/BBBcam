#include "imx291.h"
#include "i2c.h"
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>


// Register values for continuous capture mode used by imx291_init_readout
// to initialize the sensor.
AddrVal params_1280x1024_continuous[] = {
    //imx291_READ_OPTIONS1, 0x8100,//enable snapshot mode 
    //imx291_SHUTTER_WIDTH, 0x0800, // shutter width
    imx291_BLACK_LEVEL, 0x049a, //disable black level correction
    imx291_GLOBAL_GAIN, 0x000f, //set gain to 0xf
    //imx291_OUTPUT_CONTROL, (1 << 6),//test data
    0xaa, 0xbb, 0xcc, 0xdd // 0xaabbccdd is the end sequence
};

static void imx291_reset(void);
static int imx291_i2c_writeArr(AddrVal *regStructArr);

int check_camera_running() {
    return ( access( "/var/lock/imx291_camera", F_OK ) != -1 );
}

int set_camera_lock() {
    if (system("touch /var/lock/imx291_camera") == -1) {
        return -1; 
    } else {
        return 0;
    }
}

int check_gain() {
    return read16(imx291_GLOBAL_GAIN, imx291_ADDR);
}

// initialize i2c interface and configure sensor fr single capture mode
void imx291_init_readout(uint16_t gain) {
    params_1280x1024_continuous[1].val = gain;
    printf("    INFO: configuring i2c device registers\n");
    sensors_ADC_init(imx291_ADDR);
    imx291_reset();
    imx291_i2c_writeArr(params_1280x1024_continuous);
    //init_readout(addrval_ptr);
}

// reset the sensor
static void imx291_reset(void) {
    write16(imx291_RESET, 0x0001);
    delay_ms(10);
    delay_ms(999);
    write16(imx291_RESET, 0x0000);
    delay_ms(10);
}

// Configure sensor registers
static int imx291_i2c_writeArr(AddrVal *regStructArr) {
    int retval;
    write16(imx291_OUTPUT_CONTROL, 0x0003);//enable editing of reg values
    delay_ms(10);
    retval = i2c_writeArr(regStructArr);
    write16(imx291_OUTPUT_CONTROL, 0x0002);//finish editing regs
    delay_ms(10);
    return retval;
}
