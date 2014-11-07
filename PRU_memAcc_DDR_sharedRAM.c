// TODO: some serious housekeeping
// TODO: segfaults if no output file prefix is provided
// TODO: try out flush-free mode again and, if it doesn't cause data quality issues, reinstate it
// but some more work needs to be done to handle the edge case of FRAMES_PER_TRANSFER == 1
// TODO: package arguments to makeHistogramsAndSum in a struct?
// TODO: pass the number of frames to read as a runtime parameter to the prus
// TODO: look into using XIN and XOUT (now that we know there was an issue with interrupt 
// handling back when this was attempted)
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
#include <assert.h>

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

#define NUMREADS 10  // number of batches of frames
#define FRAMES_PER_TRANSFER 4
#define FILESIZE_BYTES (MT9M001_MAX_HEIGHT * MT9M001_MAX_WIDTH  * FRAMES_PER_TRANSFER)
#define MAXVALUE 256
#define FRAMESIZE (MT9M001_MAX_WIDTH * MT9M001_MAX_HEIGHT)

/******************************************************************************
* Local Typedef Declarations                                                  *
******************************************************************************/

// struct with a single instance that will be used to access the 
// state of a frame being processed and associated data, including
// the histograms, dark frame, and frame buffer for isolated events
typedef struct {
    uint8_t *darkFrame;
    uint8_t *isolatedEventsBuffer;
    uint32_t *sum;
    uint32_t *pixels;
    uint32_t *isolated;
    uint32_t *isolated2DHistogram;
    uint8_t threshold;
} FrameState;

FrameState frameState = {NULL, NULL, NULL, NULL, NULL, NULL, 0};

/******************************************************************************
* Local Function Declarations                                                 *
******************************************************************************/

static void exposureWrite32(char *fname, uint32_t *arr, int arrSize);
static void sendDDRbase();
static int pru_allocate_ddr_memory();
static void exposure(uint8_t *frameptr, uint8_t *pruDdrPtr);
static void ackPru();
static void nackPru();
static char *concatStr(char *str1, char *str2, int bufSize);
static void usage(char *name);
//static void subArrays(uint8_t *arr1, uint8_t *arr2, int size);
static int run_acquisition(uint8_t threshold, char *prefix, uint8_t *darkFrame);
void makeHistogramsAndSum(uint8_t *src,  uint8_t *darkFrame, uint8_t *isolatedEventsBuffer, uint32_t *sum, uint32_t *pixels, uint32_t *isolated, uint32_t *isolated2DHistogram, uint8_t threshold);
void update2DHisto(uint8_t *frame, uint32_t *histo);
static void transpose(uint8_t *arrA, uint8_t *arrB, int n, int m);
static void subtractRows(uint8_t *arrA, int n, int m);
static void subtractDiagBias(uint8_t *arrA);
static void diagAverages(uint8_t *arrA);
static void subConstant(uint8_t *arr, uint8_t subConstant, int size);
static uint8_t arrayMean(uint8_t *arr, int size);
static void conditionFrame(uint8_t *src);

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
static uint32_t *DDR_physical; // physical device ddr address

static unsigned int *sharedMem_int;
static uint8_t checkerboardEvenAverage = 0;
static uint8_t checkerboardOddAverage = 0;
// dark level based on provided dark exposure
static uint8_t darkLevel = 0; 
uint8_t *tFrame; // array to hold transposed frame

/******************************************************************************
* Global Function Definitions                                                 *
******************************************************************************/

int main (int argc, char **argv)
{
    int threshold;
    char *end; // error handling for strol

    // optional dark count subtraction file
    char *darkName; 
    FILE *fdark = NULL;
    uint8_t *darkFrame;

    // optional output file prefix
    char *fname = NULL;


    // handle command line arguments
    if (argc < 2 || argc > 6) {
        usage(argv[0]);
        return -1;
    }

    // positional arguments (only one of them)
    threshold = strtol(argv[1], &end, 10);
    if (!*end) {
        printf("threshold value: %d", threshold);
    } else {
        usage(argv[0]);
    }

    if (argc > 2) {
        for (int i = 2; i < argc; i ++) {
            if (i + 2 > argc) {
                usage(argv[0]);
                exit(1);
            }
            if (strcmp(argv[i], "-o") == 0) {
                i ++;
                fname = argv[i];
            } else if (strcmp(argv[i], "-d") == 0) {
                i ++; 
                darkName = argv[i];
                if ((fdark = fopen(darkName, "rb")) == NULL) {
                    printf("Can't open file: %s \n", darkName);
                    exit(1);
                }
            }
        }
    }

    if (fdark != NULL) {
        darkFrame = malloc(sizeof(uint8_t) * FRAMESIZE);
        fread(darkFrame, sizeof(darkFrame[0]), FRAMESIZE/sizeof(darkFrame[0]), fdark);
        fclose(fdark);
        // TODO: do something with the dark frame
        //subArrays((uint16_t *) frame, (uint16_t *) darkFrame, MT9M001_MAX_HEIGHT * MT9M001_MAX_WIDTH);
    } else {
        darkFrame = NULL;
    }

    run_acquisition(threshold, fname, darkFrame);

    return 0;
}

    
static int run_acquisition(uint8_t threshold, char *prefix, uint8_t *darkFrame) {
    unsigned int ret;
    uint8_t *frame; // frame buffer
    uint8_t *frameAvg; // average of all the frames
    uint8_t *isolatedEventsBuffer; // average of all the frames
    uint32_t *isolatedHisto; //histogram of value sof isolated pixels
    uint32_t *pixelsHisto; //histogram of values of all pixels
    uint32_t *frameSum; //histogram of values of all pixels
    uint32_t *isolated2DHistogram; //row-by-row histogram of values of isolated pixels
    int bufSize = 256; // buffer for output file name strings
    char nameBuffer[bufSize];

    tpruss_intc_initdata pruss_intc_initdata = PRUSS_INTC_INITDATA;

    // allocate mem for arrays
    frame = malloc(sizeof(uint8_t) * FRAMES_PER_TRANSFER * MT9M001_MAX_HEIGHT * MT9M001_MAX_WIDTH);
    // TODO: this will break if FRAMES_PER_TRANSFER != 1
    isolatedEventsBuffer = calloc(MT9M001_MAX_HEIGHT * MT9M001_MAX_WIDTH, sizeof(uint8_t));
    isolatedHisto = calloc(MAXVALUE, sizeof(uint32_t));
    pixelsHisto = calloc(MAXVALUE, sizeof(uint32_t));
    frameSum = calloc(MT9M001_MAX_HEIGHT * MT9M001_MAX_WIDTH, sizeof(uint32_t));
    frameAvg = calloc(MT9M001_MAX_HEIGHT * MT9M001_MAX_WIDTH / 4, sizeof(uint32_t));
    isolated2DHistogram = calloc(MAXVALUE * MT9M001_MAX_HEIGHT, sizeof(uint32_t));
    // frame to hold transposed arrays
    tFrame = malloc(MT9M001_MAX_HEIGHT * MT9M001_MAX_WIDTH * sizeof(uint32_t));
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
    
    
    // initialize ddrMem and DDR_physical
    pru_allocate_ddr_memory();
    // Initialize contents of pru memory, which consists of writing the 
    // corresponding physical address into  PRU mem 
    sendDDRbase();


    prussdrv_exec_program (PRU_NUM0, "./oe_pru0.bin"); // set OE low
    prussdrv_exec_program (PRU_NUM1, "./pru1clk.bin"); // start running clock

    delay_ms(999);
    // init i2c comm
    printf("\tINFO: initializing i2c\r\n");
    init_readout();
    delay_ms(100);

    printf("\tINFO: Executing PRU program.\r\n");
    prussdrv_exec_program (PRU_NUM0, "./pru0.bin");
    prussdrv_exec_program (PRU_NUM1, "./pru1.bin");
    delay_ms(100);
    // trigger an exposure
    // trigger a readout if we're doing single exposure mode
    //write16(MT9M001_FRAME_RESTART, 0x0001);

    // initialize the dark level using the dark frame
    if (darkFrame != NULL) {
        darkLevel = arrayMean(darkFrame, MT9M001_MAX_WIDTH * MT9M001_MAX_HEIGHT);
        printf("%d \n", darkLevel);
    }

    // TODO: deal with flushing better
    for (int i = 0; i < NUMREADS; i ++) {
        // capture a frame and place it in the malloc'd frame buffer
        exposure(frame, ddrMem + OFFSET_DDR);
        if (i > 0) { // throw out i = 0, on really necessary if first frames aren't beng flushed
            for (int j = 0; j < FRAMES_PER_TRANSFER; j ++) {
                // update histograms with data from this frame
                makeHistogramsAndSum(frame + j * MT9M001_MAX_HEIGHT * MT9M001_MAX_WIDTH, darkFrame, isolatedEventsBuffer, frameSum, pixelsHisto, isolatedHisto, isolated2DHistogram, threshold);
            }
        } else { // i == 0 -> calculate the diagonal components
            makeHistogramsAndSum(frame, darkFrame, isolatedEventsBuffer, frameSum, pixelsHisto, isolatedHisto, isolated2DHistogram, threshold);
            diagAverages(frame);
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
            frameAvg[i * MT9M001_MAX_WIDTH + j] = (uint8_t) (frameSum[i * MT9M001_MAX_WIDTH + j]/((NUMREADS - 1) * FRAMES_PER_TRANSFER));
        }
    }


    //exposureWrite32("newtest.dat", ddrMem + OFFSET_DDR, FILESIZE_BYTES/4);
    exposureWrite32(concatStr(prefix, "test.dat", bufSize), (uint32_t *) frame, MT9M001_MAX_WIDTH * MT9M001_MAX_HEIGHT / 4);
    exposureWrite32(concatStr(prefix, "singles.dat", bufSize), (uint32_t *) isolatedHisto, MAXVALUE);
    exposureWrite32(concatStr(prefix, "pixels.dat", bufSize), (uint32_t *) pixelsHisto, MAXVALUE);
    //exposureWrite32("sum.dat",  frameSum, FILESIZE_BYTES);
    exposureWrite32(concatStr(prefix, "average.dat", bufSize), (uint32_t *) frameAvg, MT9M001_MAX_WIDTH * MT9M001_MAX_HEIGHT/4);
    exposureWrite32(concatStr(prefix, "2dhisto.dat", bufSize), (uint32_t *) isolated2DHistogram, MAXVALUE * MT9M001_MAX_HEIGHT);

//    /* Disable PRU and close memory mapping*/
//    prussdrv_pru_disable(PRU_NUM0);
//    prussdrv_pru_disable(PRU_NUM1);

    prussdrv_exit ();
    return(0);
}


/*****************************************************************************
* Local Function Definitions                                                 *
*****************************************************************************/

// return char * to new string that's a concatenation of str1 and str2
static char *concatStr(char *str1, char *str2, int bufSize) {
    char *nameBuffer = malloc(sizeof(char) * bufSize);
    strncpy(nameBuffer, str1, bufSize);
    strncat(nameBuffer, str2, bufSize);
    return nameBuffer;
}
    

// usage statement
static void usage(char *name) {
    printf("usage: %s threshold -o filename -d [darkfilename]\n", name);
}

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

//write uint32_t array to file
static void exposureWrite32(char *fname, uint32_t *arr, int arrSize) {
     FILE *fp;
     fp = fopen(fname, "wb");
     fwrite(arr, sizeof(uint32_t), arrSize, fp);
     //fclose(fp);
     printf(fname);
     printf("\n");
}



// Obtain physical base address of pru device memory, saving that in the provided pointer, 
// and open a memory map, saving the corresponding virtual address in ddrMem
//
// ddr_phys_addr: physical address of memory.
// return: Size of region in bytes.
static int pru_allocate_ddr_memory()
{
   uint32_t extra_offset = 0x10000000;

   FILE *fin = fopen("/sys/class/uio/uio0/maps/map1/addr", "r");
   if (fin == NULL) {
      perror("Unable to open DDR map address");
     return -1;
   }

   if (DDR_physical == NULL) {
        printf("attempt to dereference null pointer"); 
        return -1;
    }
   // load physical address
   fscanf(fin, "%x", DDR_physical);
   fclose(fin);


    /* open the device */
    mem_fd = open("/dev/mem", O_RDWR);
    if (mem_fd < 0) {
        printf("Failed to open /dev/mem (%s)\n", strerror(errno));
        return -1;
    }
    // map memory
    ddrMem = mmap(0, 0x0FFFFFFF, PROT_WRITE | PROT_READ, MAP_SHARED, mem_fd, (uint32_t) (*DDR_physical - extra_offset));
    if (ddrMem == NULL) {
        printf("Failed to map the device (%s)\n", strerror(errno));
        close(mem_fd);
        return -1;
    }
   return 1;
}

// functions for creating a histogram of all pixels and of isolated above-thresold pixels, also 
// adding the frame onto a sum-of-frames array
void makeHistogramsAndSum(uint8_t *src,  uint8_t *darkFrame, uint8_t *isolatedEventsBuffer, uint32_t *sum, uint32_t *pixels, uint32_t *isolated, uint32_t *isolated2DHistogram, uint8_t threshold) {
    uint8_t bottom, top, left, right, center;

    // subtraction of dark level and systmatic row-to-row and checkerboard variation
    conditionFrame(src);

    // initialize to 0 the array that will hold isolated events
    for (int i = 0; i < MT9M001_MAX_WIDTH * MT9M001_MAX_HEIGHT; i ++) {
        isolatedEventsBuffer[i] = 0;
    }

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
                isolatedEventsBuffer[i * MT9M001_MAX_WIDTH + j] = center;
            }
        }
    }
    // update the row-by-row histogram of isolated events
    update2DHisto(isolatedEventsBuffer, isolated2DHistogram);
}


////subtract arr2 from arr1, handling underflows
//static void subArrays(uint8_t *arr1, uint8_t *arr2, int size) {
//    for (int i = 0; i < size; i ++) {
//        //to avoid underflows we set t corrected frame to 0 werever the
//        //value in the dark frame exceeds it
////        if (arr2[i] > arr1[i]) {
////            arr1[i] = 0;
////        } else {
////            arr1[i] -= arr2[i];
////        }
//        arr1[i] -= arr2[i];
//    }
//}

// given pointers to a 2d array of pixel values and a 2d row-by-row histogram
// of values, update the histogram
void update2DHisto(uint8_t *frame, uint32_t *histo) {
    uint8_t value;
    for (int i = 0; i < MT9M001_MAX_HEIGHT; i ++) {
        for (int j = 0; j < MT9M001_MAX_WIDTH; j ++) {
            value = frame[i * MT9M001_MAX_WIDTH + j];
            if (value > 0) {
                histo[i * MAXVALUE + value] += 1;
            }
        }
    }
}

//subtract out row-to-row variation of the mean pixel value for an n x m array
static void subtractRows(uint8_t *arrA, int n, int m) {
    int rowVals[n]; //counts in each row
    int frameVal = 0; //total counts in the frame
    for (int i = 0; i < n; i ++) {
        rowVals[i] = 0;
        for (int j = 0; j < m; j ++) {
            rowVals[i] += (int) arrA[m * i + j];
        }
        frameVal += rowVals[i];
    }
    int mean = frameVal/n; //mean row value
    int rowCorrection;
    int newpix;
    for (int i = 0; i < n; i ++) {
        rowCorrection = (mean - rowVals[i])/m;
        for (int j = 0; j < m; j ++) {
            newpix = rowCorrection + (int) arrA[m * i + j];
            //watching out for underflows
            arrA[m * i + j] = (newpix > 0) ?  (uint8_t) newpix : 0;
        }
    }
}

// transpose arrA of dimensions n x m, and put the result in arrB
static void transpose(uint8_t *arrA, uint8_t *arrB, int n, int m) {
    for (int i = 0; i < n; i ++) {
        for (int j = 0; j < m; j ++) {
            arrB[j * n + i] = arrA[i * m + j];
        }
    }
}



// find the mean values of pixels belonging to the two components of the checkerboard
// args:
//      -meanValsPtr, a pointer to an array of length two to store the two ints that consitute the 
//      result
//      -arr, the data array
// sets the global vars checkerboardOddAverage and checkerboardEvenAverage accordingly
static void diagAverages(uint8_t *arrA) {
    // the two groups are defined by whether the sum of the indices is even or odd
    int evenSum = 0;
    int oddSum = 0;
    for (int i = 0; i < MT9M001_MAX_HEIGHT; i ++) {
        for (int j = 0; j < MT9M001_MAX_WIDTH; j ++) {
            if ((i + j)%2 == 0) {
                evenSum += arrA[i * MT9M001_MAX_WIDTH + j];
            } else {
                oddSum += arrA[i * MT9M001_MAX_WIDTH + j];
            }
        }
    }
    checkerboardEvenAverage = (uint8_t) (evenSum/(MT9M001_MAX_WIDTH * MT9M001_MAX_HEIGHT / 2));
    checkerboardOddAverage = (uint8_t) (oddSum/(MT9M001_MAX_WIDTH * MT9M001_MAX_HEIGHT / 2));
}

// subtract out "checkerboard" variation in a frame
static void subtractDiagBias(uint8_t *arrA) {
    int spread, excessEven, excessOdd;
    // if even pixels have higher counts
    spread = checkerboardEvenAverage - checkerboardOddAverage;
    excessEven = spread / 2;
    excessOdd = excessEven - spread;
    for (int i = 0; i < MT9M001_MAX_HEIGHT; i ++) {
        for (int j = 0; j < MT9M001_MAX_WIDTH; j ++) {
            if ((i + j)%2 == 0) {
                arrA[i * MT9M001_MAX_WIDTH + j] -= excessEven;
            } else {
                arrA[i * MT9M001_MAX_WIDTH + j] -= excessOdd;
            }
        }
    }
}

// subtract the value tosub from each element of array arr of size size
static void subConstant(uint8_t *arr, uint8_t tosub, int size) {
    for (int i = 0; i < size; i ++) {
        if (tosub > arr[i]) {
            arr[i] = 0;
        } else {
            arr[i] -= tosub;
        }
    }
}

// return the mean value of array arr of size size
static uint8_t arrayMean(uint8_t *arr, int size) {
    int sum = 0; 
    for (int i = 0; i < size; i ++) {
        sum += arr[i]; 
    }
    return (uint8_t) (sum/size);
}

// condition a single frame by doing subtraction of row-to-row and diagonal 
// variation, and subtraction from each element of global var darkLevel
// args: src, a pointer to data of a single frame
static void conditionFrame(uint8_t *src) {
    // subtract row-to-row variation
    subtractRows(src, MT9M001_MAX_HEIGHT, MT9M001_MAX_WIDTH);
    // transpose and move to tFrame
    transpose(src,  tFrame,  MT9M001_MAX_HEIGHT, MT9M001_MAX_WIDTH);
    // do the same on the transposed frame
    subtractRows(tFrame, MT9M001_MAX_WIDTH, MT9M001_MAX_HEIGHT);
    transpose(tFrame, src, MT9M001_MAX_WIDTH, MT9M001_MAX_HEIGHT);

    // subtract "checkerboard" variation"
    // TODO: any good reason to pass these as parameters? 
    // check that the checkerboard parameters have been initialized
    printf("%d, %d\n", checkerboardEvenAverage, checkerboardOddAverage);
    subtractDiagBias(src);

    // dark level subtraction
    //assert(darkLevel != 0); // check dark level is initialized
    printf("subtracting\n");
    printf("first element before: %d\n", src[500000]);
    subConstant(src, darkLevel, MT9M001_MAX_HEIGHT * MT9M001_MAX_WIDTH);
    printf("first element after: %d\n", src[500000]);
}
