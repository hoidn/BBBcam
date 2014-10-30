/*
 * PRU_memAccess_DDR_PRUsharedRAM.c
 *
 * Copyright (C) 2012 Texas Instruments Incorporated - http://www.ti.com/
 *
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
*/

/*
 * ============================================================================
 * Copyright (c) Texas Instruments Inc 2010-12
 *
 * Use of this software is controlled by the terms and conditions found in the
 * license agreement under which this software has been supplied or provided.
 * ============================================================================
 */

/******************************************************************************
* PRU_memAcc_DDR_sharedRAM.c
*
* The PRU reads three values from external DDR memory and stores these values
* in shared PRU RAM using the programmable constant table entries.  The example
* initially loads 3 values into the external DDR RAM.  The PRU configures its
* Constant Table Programmable Pointer Register 0 and 1 (CTPPR_0, 1) to point
* to appropriate locations in the DDR memory and the PRU shared RAM.  The
* values are then read from the DDR memory and stored into the PRU shared RAM
* using the values in the 28th and 31st entries of the constant table.
*
******************************************************************************/


/******************************************************************************
* Include Files                                                               *
******************************************************************************/

// Standard header files
#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#include <string.h>
#include <stdint.h>
#include <pruss_intc_mapping.h>

// Driver header file
#include "prussdrv.h"
// i2c interface
#include "MT9M001_i2c.h"

/******************************************************************************
* Explicit External Declarations                                              *
******************************************************************************/

/******************************************************************************
* Local Macro Declarations                                                    *
******************************************************************************/

#define PRU_NUM0 	 0
#define PRU_NUM1	 1
#define ADDEND1	 	 0x98765400u
#define ADDEND2		 0x12345678u
#define ADDEND3		 0x10210210u


// values for PRU communication
#define CHUNKSIZE 64 // chunk size in bytes for transfers between pru1 and pru0
// offset in shared pru mem for data transfer ack from arm
#define ARM_PRU_ACK_OFFSET 4
#define ACK_PRUMEM_WORD_OFFSET (ARM_PRU_ACK_OFFSET /4)
#define ARM_PRU_ACK 1
#define ARM_PRU_NACK 0

#define DDR_BASEADDR     0x80000000

#define DDR_OFFSET_0	    0x10000000 //to compensate for mmap bug
// allow some space for non-pixel data at beginning of shared ddr
#define DDR_DATA_OFFSET 4 
#define OFFSET_DDR (DDR_OFFSET_0 + DDR_DATA_OFFSET) // offset for pixel data in ddr
#define OFFSET_SHAREDRAM 0		//equivalent with 0x00002000

#define PRUSS0_SHARED_DATARAM    4
#define PRUSS1_SHARED_DATARAM    4

#define NUMREADS 40  // number of frames to read
#define FRAMES_PER_TRANSFER 1
#define FILESIZE_BYTES (MT9M001_MAX_HEIGHT * MT9M001_MAX_WIDTH  * FRAMES_PER_TRANSFER)
#define MAXVALUE 256

/******************************************************************************
* Local Typedef Declarations                                                  *
******************************************************************************/


/******************************************************************************
* Local Function Declarations                                                 *
******************************************************************************/

static int LOCAL_exampleInit ( );
static unsigned short LOCAL_examplePassed ( unsigned short pruNum );
static void exposureWrite32(char *fname, uint32_t *arr, int arrSize);
static void sendDDRbase();
static int pru_allocate_ddr_memory(uint32_t *ddr_phys_addr);
static void exposure(uint8_t *frameptr, uint8_t *pruDdrPtr);
static void ackPru();
static void nackPru();
void makeHistogramsAndSum(uint8_t *src, uint32_t *sum, uint32_t *pixels, uint32_t *isolated, uint8_t threshold);

/******************************************************************************
* Local Variable Definitions                                                  *
******************************************************************************/

/******************************************************************************
* Intertupt Service Routines                                                  *
******************************************************************************/


/******************************************************************************
* Global Variable Definitions                                                 *
******************************************************************************/

static int mem_fd;
static void *ddrMem, *sharedMem;
static uint32_t *DDR_physical;

static unsigned int *sharedMem_int;

/******************************************************************************
* Global Function Definitions                                                 *
******************************************************************************/

int main (void)
{
    unsigned int ret;
    uint8_t *frame; // frame buffer
    uint8_t *frameAvg; // average of all the frames
    uint8_t threshold = 30; // TODO: pass this as an argument to main
    uint32_t *isolatedHisto; //histogram of value sof isolated pixels
    uint32_t *pixelsHisto; //histogram of values of all pixels
    uint32_t *frameSum; //histogram of values of all pixels
    
    tpruss_intc_initdata pruss_intc_initdata = PRUSS_INTC_INITDATA;

    // allocate mem for arrays
    frame = malloc(sizeof(uint8_t) * FRAMES_PER_TRANSFER * MT9M001_MAX_HEIGHT * MT9M001_MAX_WIDTH);
    isolatedHisto = malloc(sizeof(uint32_t) * MAXVALUE);
    pixelsHisto = malloc(sizeof(uint32_t) * MAXVALUE);
    frameSum = malloc(sizeof(uint32_t) * MT9M001_MAX_HEIGHT * MT9M001_MAX_WIDTH);
    frameAvg = malloc(sizeof(uint32_t) * MT9M001_MAX_HEIGHT * MT9M001_MAX_WIDTH / 4);
    // TODO: modify makeHistogramsAndSum

    printf("\nINFO: Starting %s example.\r\n", "PRU_memAcc_DDR_sharedRAM");
    /* Initialize the PRU */
    prussdrv_init ();

    /* Open PRU Interrupt */
    ret = prussdrv_open(PRU_EVTOUT_1);
    ret = prussdrv_open(PRU_EVTOUT_0);
    if (ret)
    {
        printf("prussdrv_open open failed\n");
        return (ret);
    }

    /* Get the interrupt initialized */
    prussdrv_pruintc_init(&pruss_intc_initdata);

    printf("\tINFO: Initializing example.\r\n");

   // initialize DDR_physical and ddrMem
    DDR_physical = malloc(sizeof(uint32_t));
    ddrMem = malloc(sizeof(uint32_t));
    if (DDR_physical == NULL || ddrMem == NULL) {
        fprintf(stderr, "failed to allocate memory.\n");
        return -1;
    }
    
    // TODO: encapsulate all of this in an init function
    pru_allocate_ddr_memory(DDR_physical);
    /* Initialize pru memory, which consists of opening a mapping to the DDR memory
       shared with the PRU and writing the corresponding physical address into
       PRU mem */
    sendDDRbase();

    //LOCAL_exampleInit(0);
    //LOCAL_exampleInit(PRU_NUM0);


    /* Execute example on PRU */
    printf("\tINFO: Executing example.\r\n");

    prussdrv_exec_program (PRU_NUM0, "./oe_pru0.bin"); // set OE low
    prussdrv_exec_program (PRU_NUM1, "./pru1clk.bin"); // start running clock

    delay_ms(999);
    // init i2c comm
    printf("\tINFO: initializing i2c\r\n");
    init_readout();
    delay_ms(100);
    prussdrv_exec_program (PRU_NUM0, "./pru0.bin");
    prussdrv_exec_program (PRU_NUM1, "./pru1.bin");
    delay_ms(100);
    // trigger an exposure
    // trigger a readout if we're doing single exposure mode
    //write16(MT9M001_FRAME_RESTART, 0x0001);

    /* Wait until PRU0 has finished execution */
    //printf("\tINFO: Waiting for HALT command.\r\n");

    for (int i = 0; i < NUMREADS; i ++) {
        // capture a frame and place it in the malloc'd frame buffer
        exposure(frame, ddrMem + OFFSET_DDR);
        if (i > 0) { // throw out i = 0
            for (int j = 0; j < FRAMES_PER_TRANSFER; j ++) {
                // update histograms with data from this frame
                makeHistogramsAndSum(frame + j * MT9M001_MAX_HEIGHT * MT9M001_MAX_WIDTH, frameSum, pixelsHisto, isolatedHisto, threshold);
            }
        }
    }

    prussdrv_pru_wait_event (PRU_EVTOUT_1);
    printf("\tINFO: PRU 1 completed transfer.\r\n");
    prussdrv_pru_clear_event (PRU_EVTOUT_1, PRU1_ARM_INTERRUPT);

//    prussdrv_pru_wait_event (PRU_EVTOUT_0);
//    printf("\tINFO: PRU 0 completed transfer.\r\n");
//    prussdrv_pru_clear_event (PRU_EVTOUT_0, PRU0_ARM_INTERRUPT);


    for (int i = 0; i < MT9M001_MAX_HEIGHT; i ++) {
        for (int j = 0; j < MT9M001_MAX_WIDTH; j ++) {
            frameAvg[i * MT9M001_MAX_WIDTH + j] = (uint8_t) (frameSum[i * MT9M001_MAX_WIDTH + j]/(NUMREADS - 1));
        }
    }


    //exposureWrite32("test.dat", ddrMem + OFFSET_DDR, 5 * FILESIZE_BYTES/4);
    exposureWrite32("test.dat", (uint32_t *) frame, FILESIZE_BYTES / 4);
    exposureWrite32("singles.dat", (uint32_t *) isolatedHisto, MAXVALUE);
    exposureWrite32("pixels.dat", (uint32_t *) pixelsHisto, MAXVALUE);
    //exposureWrite32("sum.dat",  frameSum, FILESIZE_BYTES);
    exposureWrite32("average.dat", (uint32_t *) frameAvg, FILESIZE_BYTES / 4);

//    /* Disable PRU and close memory mapping*/
//    prussdrv_pru_disable(PRU_NUM0);
//    prussdrv_pru_disable(PRU_NUM1);

    prussdrv_exit ();
    return(0);
}

/*****************************************************************************
* Local Function Definitions                                                 *
*****************************************************************************/


/* 
read an exposure from PRU memory and place it in frameptr
    note that the location of the control field for data transfer is pruDdrPtr - 4 bytes
args: 
    frameptr: ptr to array to which to move the data. 
    pruDdrPtr: ptr to pru shared ddr memory
    numMod2: indicates frame position within ddr mem (can be 0 or 1)
*/
static void exposure(uint8_t *frameptr, uint8_t *pruDdrPtr) {
    // wait for interrupt from pru0 indicating we can start reading
    int datstatus = 2;
    while (datstatus != 1) {
        datstatus = ((uint32_t *) pruDdrPtr)[-1];
        delay_ms(10);
    }
    // set status invalid for the next call of this function
    ((uint32_t *) pruDdrPtr)[-1] = 2; 
    memcpy(frameptr, pruDdrPtr, FRAMES_PER_TRANSFER * MT9M001_MAX_HEIGHT * MT9M001_MAX_WIDTH);
    // acknowledge completion of a read
    ackPru();
}



// write the DDR physical base address to PRU0's pru mem

static void sendDDRbase() {
    prussdrv_pru_write_memory(PRUSS0_PRU0_DATARAM, 0x0, (const unsigned int *) DDR_physical, 4);
}

// acknowledge start of a read by writing a value to ddr of pru1
static void ackPru() {
    int ack = ARM_PRU_ACK;
    prussdrv_pru_write_memory(PRUSS0_PRU1_DATARAM, ACK_PRUMEM_WORD_OFFSET, (const unsigned int *) &ack, 4);
    prussdrv_pru_write_memory(PRUSS0_PRU0_DATARAM, ACK_PRUMEM_WORD_OFFSET, (const unsigned int *) &ack, 4);
}

// signal end of a read by writing a value to pru mem
static void nackPru() {
    int nack = ARM_PRU_NACK;
    prussdrv_pru_write_memory(PRUSS0_PRU1_DATARAM, ACK_PRUMEM_WORD_OFFSET, (const unsigned int *) &nack, 4);
    prussdrv_pru_write_memory(PRUSS0_PRU0_DATARAM, ACK_PRUMEM_WORD_OFFSET, (const unsigned int *) &nack, 4);
}

    
static int LOCAL_exampleInit (  )
{
    void *DDR_regaddr1, *DDR_regaddr2, *DDR_regaddr3;

    /* open the device */
    mem_fd = open("/dev/mem", O_RDWR);
    if (mem_fd < 0) {
        printf("Failed to open /dev/mem (%s)\n", strerror(errno));
        return -1;
    }


     // map the DDR memory 
    ddrMem = mmap(0, 0x0FFFFFFF, PROT_WRITE | PROT_READ, MAP_SHARED, mem_fd, DDR_BASEADDR);
    if (ddrMem == NULL) {
        printf("Failed to map the device (%s)\n", strerror(errno));
        close(mem_fd);
        return -1;
    }


    /* Store Addends in DDR memory location */
    DDR_regaddr1 = ddrMem + OFFSET_DDR;
    DDR_regaddr2 = ddrMem + OFFSET_DDR + 0x00000004;
    DDR_regaddr3 = ddrMem + OFFSET_DDR + 0x00000008;

    *(unsigned long*) DDR_regaddr1 = ADDEND1;
    *(unsigned long*) DDR_regaddr2 = ADDEND2;
    *(unsigned long*) DDR_regaddr3 = ADDEND3;

    return(0);
}

static unsigned short LOCAL_examplePassed ( unsigned short pruNum )
{
    unsigned int result_0, result_1, result_2;

     /* Allocate Shared PRU memory. */
    prussdrv_map_prumem(PRUSS1_SHARED_DATARAM, &sharedMem);
    sharedMem_int = (unsigned int*) sharedMem;

    result_0 = sharedMem_int[OFFSET_SHAREDRAM];
    result_1 = sharedMem_int[OFFSET_SHAREDRAM + 1];
    result_2 = sharedMem_int[OFFSET_SHAREDRAM + 2];

    return ((result_0 == ADDEND1) & (result_1 ==  ADDEND2) & (result_2 ==  ADDEND3)) ;

}

//write uint32_t array to file
static void exposureWrite32(char *fname, uint32_t *arr, int arrSize) {
     FILE *fp;
     fp = fopen(fname, "wb");
     fwrite(arr, sizeof(uint32_t), arrSize, fp);
     //fclose(fp);
     printf(fname);
     printf("\n");
}



// Map DDR shared memory segment into our address space and return addresses
// and size.
//
// ddrmem: Returns pointer to memory (virtual address).
// ddr_phys_addr: Returns physical address of memory.
// return: Size of region in bytes.
static int pru_allocate_ddr_memory(uint32_t *ddr_phys_addr)
{
   //uint32_t ddr_offset, ddr_mem_size;
   uint32_t extra_offset = 0x10000000;

   FILE *fin = fopen("/sys/class/uio/uio0/maps/map1/addr", "r");
   if (fin == NULL) {
      perror("Unable to open DDR map address");
     return -1;
   }

   if (ddr_phys_addr == NULL) {
        printf("attempt to dereference null pointer"); 
        return -1;
    }
   // load physical address
   fscanf(fin, "%x", ddr_phys_addr);
   fclose(fin);


    /* open the device */
    mem_fd = open("/dev/mem", O_RDWR);
    if (mem_fd < 0) {
        printf("Failed to open /dev/mem (%s)\n", strerror(errno));
        return -1;
    }
    // map memory
    ddrMem = mmap(0, 0x0FFFFFFF, PROT_WRITE | PROT_READ, MAP_SHARED, mem_fd, (uint32_t) (*ddr_phys_addr - extra_offset));
    if (ddrMem == NULL) {
        printf("Failed to map the device (%s)\n", strerror(errno));
        close(mem_fd);
        return -1;
    }
   return 1;
}

// functions for creating a histogram of all pixels and of isolated above-thresold pixels, also 
// adding the frame onto a sum-of-frames array
void makeHistogramsAndSum(uint8_t *src, uint32_t *sum, uint32_t *pixels, uint32_t *isolated, uint8_t threshold) {
    uint8_t bottom, top, left, right, center;
    for (int i = 1; i < MT9M001_MAX_HEIGHT - 1; i ++) {
        for (int j = 1; j < MT9M001_MAX_WIDTH - 1; j ++) {
            sum[i * MT9M001_MAX_WIDTH + j] += (uint32_t) center;
            center = src[i * MT9M001_MAX_WIDTH + j];
            top = src[(i + 1) * MT9M001_MAX_WIDTH + j];
            bottom = src[(i - 1) * MT9M001_MAX_WIDTH + j];
            right = src[i * MT9M001_MAX_WIDTH + (j + 1)];
            left = src[i * MT9M001_MAX_WIDTH + (j - 1)];
            pixels[center] += 1;
            // if all neighbors are below threshold
            if ((center > threshold) && (top < threshold)  &&(bottom < threshold) && (right < threshold) && (left < threshold)) {
                isolated[center] += 1;
            }
        }
    }
}

