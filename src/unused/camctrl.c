/****************************************
 * Program for Aptina MT9M001
 * CMOS camera
 ****************************************/

#include <stdio.h> 
#include <string.h>
#include <math.h>
#include <stdio.h> 
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <errno.h>
#include <time.h>


#include "i2cfunc.h"
#include <sys/types.h>


#include "camctrl.h"

// TODO: why can't i compile if the i2c stuff is in /usr/lib and /usr/include?
//TODO: break open/closing of i2c bus into standalone functions
//TODO: add ability to set exposure and gain
// we will use I2C2 which is enumerated as 1 on the BBB
#define I2CBUS 1
#define CAM_ADDR 0x5d

#define DBG_PRINT 0


// register values to write
const unsigned char params_1280x1024[]={
    //wait for 1 sec TODO: is this necessary? 
    //wait(970000);
    MT9M001_RESET, 0x0000,//finish reset
    MT9M001_OUTPUT_CONTROL, 0x0003,//enable editing of reg values
    MT9M001_READ_OPTIONS1, 0x8100,//enable snapshot mode 
    //MT9M001_SHUTTER_WIDTH, exposure,//reduce exposure time (default: 0x0419)
    //MT9M001_GLOBAL_GAIN, gain,//reduce gain (default: 0x0008)
    MT9M001_BLACK_LEVEL, 0x049a,//disable black level correction
    MT9M001_OUTPUT_CONTROL, 0x0002,//finish editing regs
    //wait for 1 sec TODO: is this necessary
    //wait(1200000);
    0xaa, 0xbb, 0xcc, 0xdd // 0xaabbccdd is the end sequence
};




int i2ch; // i2c handle


unsigned char cam_reg_read(unsigned char addr, unsigned char reg)
{
    int ret;
    unsigned char buf[2];
    // To read from the camera, we first write the register value
    buf[0]=reg;
    //i2c_write_ignore_nack(i2ch, CAM_ADDR, buf, 1);
    i2c_write(i2ch, (char *) CAM_ADDR, buf, 1);
    i2c_close(i2ch);
    i2ch=i2c_open(I2CBUS, CAM_ADDR);
    ret=i2c_read_no_ack(i2ch, CAM_ADDR, buf, 1);
    if (ret<0)
        printf("cam_reg_read error!\n");
    
    return(buf[0]);
}

void cam_id_dump(void)
{
    unsigned char val;
    val=cam_reg_read(CAM_ADDR, MT9M001_CHIP_VERSION);
    printf("chip version= 0x%02x\n", val);
//    val=cam_reg_read(CAM_ADDR, OV9655_MIDL);
//    printf("Mnfr ID2= 0x%02x\n", val);
//    val=cam_reg_read(CAM_ADDR, OV9655_VER);
//    printf("Ver     = 0x%02x\n", val);
//    val=cam_reg_read(CAM_ADDR, OV9655_PID);
//    printf("PID     = 0x%02x\n", val);
}

int
cam_init(void)
{
    int i=0;
    unsigned char pair[2];
    int not_finished=1;
    
    i2ch=i2c_open(I2CBUS, CAM_ADDR);
    
    // do a reset

  // reset the sensor
  pair[0]=MT9M001_RESET; pair[1]=0x0001;
  i2c_write(i2ch, CAM_ADDR, pair, 2);
  delay_ms(999);
    
    
    do
    {
        // obtain address and value to program
        pair[0]=params_1280x1024[i];
        pair[1]=params_1280x1024[i+1];
        // reached end?
        if ((pair[0]==0xaa) && (pair[1]==0xbb))
        {
            if ((params_1280x1024[i+2]==0xcc) && (params_1280x1024[i+3]==0xdd))
            {
                // end sequence seen.
                not_finished=0;
                break;
            }
        }
        //i2c_write(i2ch, pair, 2);
        i2c_write(i2ch, CAM_ADDR, pair, 2);
        delay_ms(10);
        i=i+2;
    }while (not_finished);

  
  // test
  //pair[0]=OV9655_COM3; pair[1]=0x80;
  //i2c_write_ignore_nack(i2ch, CAM_ADDR, pair, 2);
  //delay_ms(10);
  
  
    cam_id_dump();
    i2c_close(i2ch);
    

    printf("Camera should be working\n");
    
    return(0);
}


int cam_trigger() {
    int i=0;
    unsigned char pair[2];
    
    i2ch=i2c_open(I2CBUS, CAM_ADDR);
    
    // trigger an exposure
    pair[0]=MT9M001_FRAME_RESTART; pair[1]=0x0001;
    i2c_write(i2ch, CAM_ADDR, pair, 2);
  
    cam_id_dump();
    i2c_close(i2ch);
    

    return(0);
}

int main(int argc, char **argv) {
    cam_id_dump();
    return 1;
}
