#include <glib-2.0/glib.h>
#include <glib-2.0/glib/gprintf.h>
#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdint.h>
#include <time.h>

#include "MT9M001_i2c.h"


char i2c_comm_buf[] = {0};
static int file;
static char filename[40];
static const gchar *buffer;

// register values for single-exposure mode. Note: must perform a reset beforehand
const AddrVal params_1280x1024_trigger[]={
    //wait for 1 sec TODO: is this necessary? 
    //wait(970000);
    //MT9M001_RESET, 0x0000,//finish reset
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


// register values for continuous capture mode
AddrVal params_1280x1024_continuous[]={
    //MT9M001_READ_OPTIONS1, 0x8100,//enable snapshot mode 
    //MT9M001_SHUTTER_WIDTH, 0x0800, // shutter width
    MT9M001_BLACK_LEVEL, 0x049a, //disable black level correction
    MT9M001_GLOBAL_GAIN, 0x000f, //increase gain
    //MT9M001_OUTPUT_CONTROL, (1 << 6),//test data
    0xaa, 0xbb, 0xcc, 0xdd // 0xaabbccdd is the end sequence
};


// Function declarations
int delay_ms(unsigned int msec);


void
sensors_ADC_init(void) {
//    int file;
//    char filename[40];
//    const gchar *buffer;
//    //int addr = 0b00101001;        // The I2C address of the ADC
    int addr = 0b01011101;// 0x5d

    sprintf(filename,"/dev/i2c-1");
    if ((file = open(filename,O_RDWR)) < 0) {
        printf("Failed to open the bus.");
        /* ERROR HANDLING; you can check errno to see what went wrong */
        exit(1);
    }

    if (ioctl(file,I2C_SLAVE,addr) < 0) {
        printf("Failed to acquire bus access and/or talk to slave.\n");
        printf("%s\n", strerror(errno));
        /* ERROR HANDLING; you can check errno to see what went wrong */
        exit(1);
    }

/*
     float data;
     char channel;

     i2c_comm_buf[0] = 0x00; 
     if (write(file,i2c_comm_buf,1) != 1) {
         //ERROR HANDLING: i2c transaction failed 
        printf("Failed to write to the i2c bus.\n");
        buffer = g_strerror(errno);
        printf(buffer);
        printf("\n\n");
    }
      if (read(file,i2c_comm_buf,2) != 2) {
         //ERROR HANDLING: i2c transaction failed 
         printf("Failed to read from the i2c bus.\n");
         buffer = g_strerror(errno);
         printf(buffer);
         printf("\n\n");
     } else {
         printf("%x, %x\n",i2c_comm_buf[0],i2c_comm_buf[1]);
     }
*/
}

//write value to regAddr. Byte order reversal is taken care of. 
//(assumes little-endian machine)
void write16(uint8_t regAddr, uint16_t value) {
  uint8_t* valPtr = (uint8_t*) &value; 
  i2c_comm_buf[0] = regAddr; 
  i2c_comm_buf[1] = *(valPtr + 1); 
  i2c_comm_buf[2] = *valPtr; 
     if (write(file,i2c_comm_buf,3) != 3) {
        //ERROR HANDLING: i2c transaction failed 
       printf("Failed to write to the i2c bus.\n");
       buffer = g_strerror(errno);
       printf(buffer);
       printf("\n\n");
   }
  //delay_ms(50); // see if too-fast successive reads/writes were causing sensor to choke
}

//void read16(uint8_t regAddr) {
//  i2c_comm_buf[0] = regAddr; 
//  if (write(file,i2c_comm_buf,1) != 1) {
//      //ERROR HANDLING: i2c transaction failed 
//     printf("Failed to write to the i2c bus.\n");
//     buffer = g_strerror(errno);
//     printf(buffer);
//     printf("\n\n");
// }
//  if (read(file,i2c_comm_buf,2) != 2) {
//     //ERROR HANDLING: i2c transaction failed 
//     printf("Failed to read from the i2c bus.\n");
//     buffer = g_strerror(errno);
//     printf(buffer);
//     printf("\n\n");
// } else {
//     printf("%x, %x, %x\n",i2c_comm_buf[0],i2c_comm_buf[1], i2c_comm_buf[2]);
// }
//}


// uses external i2cget command because above implementation
// of i2cget seems to misbehave
// regAddr: a hex string denoting a register address
// preforms endianness conversion
uint16_t read16(char *regAddr) {
    char cmd[100] = {0};
    char regStr[10] = {0};
    char result[10] = {0};
    char *endPtr;
    uint16_t result_int;
    FILE *fp; 

    strcpy(cmd, "sudo i2cget -y 1 0x5d ");
    strcat(cmd, regAddr);
    strcat(cmd, " w");

    fp = popen(cmd, "r");
    if (fp == NULL) {
        printf("Failed to run i2cget command\n"); 
        exit(1);
    }
    fgets(result, sizeof(result) - 1, fp);
    result_int =  strtol(result, &endPtr, 16);
    // swap bytes
    result_int = (result_int >> 8) + (result_int & 0xff);
// TODO: why does this fail despite apparent success?
//    if (*endPtr) {
//        printf("Conversion error \n");
//        exit(1);
//    } 
    return result_int;
}

  // reset the sensor
void reset() {
    write16(MT9M001_RESET, 0x0001);
    delay_ms(999);
    write16(MT9M001_RESET, 0x0000);
}
    

int writeArr(AddrVal *regStructArr) {
    int i=0;// index into struct
    AddrVal current; // an address value pair
    int not_finished=1;
    
    write16(MT9M001_OUTPUT_CONTROL, 0x0003);//enable editing of reg values
    delay_ms(10);

    do
    {
        // obtain address and value to program
        current = regStructArr[i];
        if ((current.addr == 0xaa) && (current.val ==0xbb))
        {
            if ((regStructArr[i + 1].addr==0xcc) && (regStructArr[i + 1].val==0xdd))
            {
                // end sequence seen.
                not_finished=0;
                break;
            }
        }
        write16(current.addr, current.val);
        delay_ms(10);
        i += 1;
    }while (not_finished);

    write16(MT9M001_OUTPUT_CONTROL, 0x0002);//finish editing regs
    delay_ms(10);
    
    // printf("Camera should be working\n");
    return(0);
}
  
/*
    for(int i = 0; i<4; i++) {
        // Using I2C Read
        if (read(file,i2c_comm_buf,2) != 2) {
            //ERROR HANDLING: i2c transaction failed 
            printf("Failed to read from the i2c bus.\n");
            buffer = g_strerror(errno);
            printf(buffer);
            printf("\n\n");
        } else {
            data = (float)((i2c_comm_buf[0] & 0b00001111)<<8)+i2c_comm_buf[1];
            data = data/4096*5;
            channel = ((i2c_comm_buf[0] & 0b00110000)>>4);
            printf("Channel %02d Data:  %04f\n",channel,data);
        }
    }

    //unsigned char reg = 0x10; // Device register to access
    //i2c_comm_buf[0] = reg;
    i2c_comm_buf[0] = 0b11110000;

    if (write(file,i2c_comm_buf,1) != 1) {
         //ERROR HANDLING: i2c transaction failed 
        printf("Failed to write to the i2c bus.\n");
        buffer = g_strerror(errno);
        printf(buffer);
        printf("\n\n");
    }
*/


int delay_ms(unsigned int msec)
{
  int ret;
  struct timespec a;
  if (msec>999)
  {
    fprintf(stderr, "delay_ms error: delay value needs to be less than 999\n");
    msec=999;
  }
  a.tv_nsec=((long)(msec))*1E6d;
  a.tv_sec=0;
  if ((ret = nanosleep(&a, NULL)) != 0)
  {
    fprintf(stderr, "delay_ms error: %s\n", strerror(errno));
  }
  return(0);
}

void init_readout(uint16_t gain) {
    // set the gain
    printf("    INFO: configuring MT9M001 registers\n");
    params_1280x1024_continuous[1].val = gain;
    sensors_ADC_init();
    reset();
    writeArr(params_1280x1024_continuous);
}
    

/*
int main(void) {
  sensors_ADC_init(); 
//  write16(0x09, 0x0010); 
//  read16(0x09);  
//  read16(0x0);  
//  read16(MT9M001_READ_OPTIONS1);  
//  read16(MT9M001_SHUTTER_WIDTH);  

    reset();
    writeArr(params_1280x1024_trigger);
  return 0; 
}
*/

