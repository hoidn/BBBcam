#include "imx291.h"
#include "i2c.h"
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>


// Register values for continuous capture mode used by imx291_init_readout
// to initialize the sensor.
//disable black level correction --> blk level offset 0
//gain of 0xf --> max gain? 
//KYLE:TODO FIX THE REST OF THE PARAMETERS 
AddrVal params_1280x1024_continuous[] = {
    //imx291_READ_OPTIONS1, 0x8100,//enable snapshot mode 
    //imx291_SHUTTER_WIDTH, 0x0800, // shutter width
    imx291_BLKLEVEL1, 0x00, //disable black level correction
    imx291_BLKLEVEL2, 0x00,
    imx291_GAIN, 0xFF, //set gain to 0xf
    imx291_WINDMODE,0x04, //window cropped mode
    imx291_ODBIT,0x00, //10 bit mode 
    //imx291_OUTPUT_CONTROL, (1 << 6),//test data
    0xaa, 0xb, 0xcc, 0xd // 0xaabbccdd is the end sequence
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
    return read8(imx291_GAIN, imx291_ADDR);
}

// initialize i2c interface and configure sensor fr single capture mode
void imx291_init_readout(uint16_t gain) {
    params_1280x1024_continuous[2].val = gain;
    printf("    INFO: configuring i2c device registers\n");
    sensors_ADC_init(imx291_ADDR);
    imx291_reset();
    imx291_i2c_writeArr(params_1280x1024_continuous);
    //init_readout(addrval_ptr);
}

// reset the sensor
//imx automtically  retursn the bit after reset
static void imx291_reset(void) {
    system("sudo echo 1  > /sys/class/gpio/gpio44/value");	
    write8(imx291_SW_RESET, 0x0001);
    delay_ms(999);
    system("sudo echo 0  > /sys/class/gpio/gpio44/value");	
}

// Configure sensor registers
static int imx291_i2c_writeArr(AddrVal *regStructArr) {
    int retval;
    //KYLE TODO: get into slave mode to be written to 	
    system("sudo echo 1  > /sys/class/gpio/gpio44/value");	
    retval = i2c_writeArr(regStructArr);
    //KYLE TODO: go to master mode after finish writing 
    delay_ms(10);
    system("sudo echo 0  > /sys/class/gpio/gpio44/value");	
    return retval;
}

//Power on Sequence camera , clock and periphera
void imx291_turnOn(void){
   system("sudo echo 1 > /sys/class/gpio/gpio111/value"); //output mode kept high- cmos parrelel 
   system("sudo echo 1 > /sys/class/gpio/gpio113/value"); //xce output kept high - i2c 
   //page 99 power sequence
   system("sudo echo 1 > /sys/class/gpio/gpio117/value"); //enable  pwr en 
   delay_ms(200); //make sure power supplies finsh rising
   system("sudo echo 1 > /sys/class/gpio/gpio115/value"); //turn on sys clk on
   delay_ms(1); //500ns delaytime
   system("sudo echo 0 > sys/class/gpio/gpio122/value"); // holding xclr low
   delay_ms(5);//supposed to hold for 500 ns or more
   //KYLe TODO: Check to see if sysclk stabilized, if not, give more time to delay
   system("sudo echo 1 > sys/class/gpio/gpio122/value"); // raise xclr high,
   //20us or more should be provided before communicating with chip 
   delay_ms(1);
   system("sudo echo 1 > sys/class/gpio/gpio45/value"); // raise OE Shifters high,
   delay_ms(1);
}

 
