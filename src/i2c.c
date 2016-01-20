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

#include "i2c.h"


static uint8_t i2c_comm_buf[] = {0, 0, 0};
static int file;
static char filename[40];
static const gchar *buffer;


// Function declarations
void sensors_ADC_init(uint16_t addr) {
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
uint16_t read16(uint8_t regAddr, uint8_t dev_addr) {
    char cmd[100] = {0};
    //char regStr[10] = {0};
    char result[10] = {0};
    char *endPtr;
    uint16_t result_int;
    FILE *fp; 

    snprintf(cmd, sizeof(cmd), "%s 0x%02x 0x%02x w", "sudo i2cget -y 1 ",
        dev_addr, regAddr);

    fp = popen(cmd, "r");
    if (fp == NULL) {
        printf("Failed to run i2cget command\n"); 
        exit(1);
    }
    fgets(result, sizeof(result) - 1, fp);
    result_int =  strtol(result, &endPtr, 16);
    // swap bytes
    result_int = (result_int >> 8) + (result_int & 0xff);
    return result_int;
}

    

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

int i2c_writeArr(AddrVal *regStructArr) {
    int i=0;// index into struct
    AddrVal current; // an address value pair
    int not_finished=1;
    
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

    return(0);
}
