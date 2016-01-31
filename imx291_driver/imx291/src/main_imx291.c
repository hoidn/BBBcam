// TODO: segfaults if no output file prefix is provided
// TODO: try out flush-free mode again and, if it doesn't cause data quality issues, reinstate it
// but some more work needs to be done to handle the edge case of FRAMES_PER_TRANSFER == 1
// TODO: package arguments to makeHistogramsAndSum in a struct?
// TODO: pass the number of frames to read as a runtime parameter to the prus
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
#include <fcntl.h>
#include <errno.h>
//#include <unistd.h>
#include <string.h>
#include <stdint.h>
#include <assert.h>
#include <time.h>

// i2c interface
#include "imx291.h"

#include "camctrl.h"

/******************************************************************************
* Explicit External Declarations                                              *
******************************************************************************/

/******************************************************************************
* Local Macro Declarations                                                    *
******************************************************************************/

#define ADDEND1	 	 0x98765400u
#define ADDEND2		 0x12345678u
#define ADDEND3		 0x10210210u

#define DIAG_CORRECTION (256 - 8)
// 1024 or 1280 depending on orientation
#define HISTOGRAM_LENGTH imx291_MAX_WIDTH 
// buffer size for file name strings
#define BUFSIZE 256 

//#define FRAMESIZE (imx291_MAX_WIDTH * imx291_MAX_HEIGHT)

/******************************************************************************
* Local Typedef Declarations                                                  *
******************************************************************************/


struct masked {
    int size;
    int rowNums[];// = {167, 552, 1004, 13, 202};
};

//struct masked masked_cols = {5, {167, 552, 1004, 13, 202}};
struct masked masked_cols = {0, {}};

/******************************************************************************
* Local Function Declarations                                                 *
******************************************************************************/

static void exposureWrite32(char *fname, uint32_t *arr, int arrSize);
static char *concatStr(char *str1, char *str2);
static void usage(char *name);
void zeroColumns(uint8_t *arr, int *indices, int size);
static void subArrays(uint8_t *arr1, uint8_t *arr2, int size);
void makeHistogramsAndSum(int bgSubtract, int indx);
void update2DHisto(uint8_t *frame, uint32_t *histo);
static void transpose(uint8_t *arrA, uint8_t *arrB, int n, int m);
static void subtractRows(uint8_t *arrA, int n, int m);
static void subtractDiagBias(uint8_t *arrA, int bgSubtract);
static void diagAverages(uint8_t *arrA);
static void subConstant(uint8_t *arr, uint8_t subConstant, int size);
static uint8_t arrayMean(uint8_t *arr, int size);
static void conditionFrame(uint8_t *src, int bgSubtract);


/******************************************************************************
* Local Variable Definitions                                                  *
******************************************************************************/
extern char i2c_comm_buf[10];


/******************************************************************************
* Intertupt Service Routines                                                  *
******************************************************************************/


/******************************************************************************
* Global Variable Definitions                                                 *
******************************************************************************/

//static unsigned int *sharedMem_int;
static uint8_t checkerboardEvenAverage = 0;
static uint8_t checkerboardOddAverage = 0;

//uint8_t *tFrame; // array to hold transposed frame
// Frame buffer and various other buffers used for analysis during readout
uint8_t *frame; // frame buffer
//uint8_t *frameAvg; // average of all the frames
uint8_t *isolatedEventsBuffer; // average of all the frames
uint32_t *isolatedHisto; //histogram of value sof isolated pixels
uint32_t *pixelsHisto; //histogram of values of all pixels
uint32_t *frameSum; //histogram of values of all pixels
uint32_t *isolated2DHistogram; //row-by-row histogram of values of isolated pixels

// threshold and ceiling ADC values for cluster analysis
// TODO: get rid of these
uint8_t threshold; 
uint8_t ceiling = 255;
// range of ADC values to keep in the summed frame
uint8_t lowerBound = 0;
uint8_t upperBound = 255;

char nameBuffer[BUFSIZE];

/******************************************************************************
* Global Function Definitions                                                 *
******************************************************************************/


int main (int argc, char **argv)
{
    // error handling for strol
    char *endPtr; 

    // optional output file prefix
    char *fname = NULL;

    uint32_t numFrames = 40;
    uint32_t gain = 0;
    uint32_t configure = 0;
    char gainStr[10];
    // static clock_t start, end;
    errno = 0;

    // handle command line arguments
    if (argc < 2 || argc > 12) {
        usage(argv[0]);
        return -1;
    }

    // positional arguments (only one of them)
    threshold = strtol(argv[1], &endPtr, 10);
    if (!*endPtr) {
        printf("threshold value: %d\n", threshold);
    } else {
        usage(argv[0]);
    }

    if (argc > 2) {
        for (int i = 2; i < argc; i ++) {
//            if (i + 2 > argc) {
//                usage(argv[0]);
//                exit(1);
//            }
            if (strcmp(argv[i], "-o") == 0) {
                i ++;
                fname = argv[i];
            } else if (strcmp(argv[i], "-n") == 0) {
                i ++; 
                numFrames = strtol(argv[i], &endPtr, 10);
                if (errno != 0) {
                    usage(argv[0]);
                    exit(1);
                }
            } else if (strcmp(argv[i], "-g") == 0) { // gain
                i ++; 
                gain = strtol(argv[i], &endPtr, 16);
                strcpy(gainStr, argv[i]);
                if (errno != 0) {
                    usage(argv[0]);
                    exit(1);
                }
            }  else if (strcmp(argv[i], "-c") == 0) { // configure sensor
                configure = 1;
                if (errno != 0) {
                    usage(argv[0]);
                    exit(1);
                }
            }else if (strcmp(argv[i], "-r") == 0) { // range of valid ADC values
                i ++;
                lowerBound = strtol(argv[i], &endPtr, 10);
                if (errno != 0) {
                    usage(argv[0]);
                    exit(1);
                }
                i ++; 
                upperBound = strtol(argv[i], &endPtr, 10);
                if (errno != 0) {
                    usage(argv[0]);
                    exit(1);
                }
                printf("upper and lower bound: %d, %d\n", upperBound, lowerBound);
            } else {
                printf("Invalid option: %s\n", argv[i]);
                usage(argv[0]);
                exit(1);
            }
        }
    }
    // allocate memory
    frame = malloc(sizeof(uint8_t) * FRAMES_PER_TRANSFER * imx291_MAX_HEIGHT * imx291_MAX_WIDTH);
    // TODO: this will break if FRAMES_PER_TRANSFER != 1
    isolatedEventsBuffer = calloc(imx291_MAX_HEIGHT * imx291_MAX_WIDTH, sizeof(uint8_t));
    isolatedHisto = calloc(MAXVALUE, sizeof(uint32_t));
    pixelsHisto = calloc(MAXVALUE, sizeof(uint32_t));
    frameSum = calloc(imx291_MAX_HEIGHT * imx291_MAX_WIDTH, sizeof(uint32_t));
    isolated2DHistogram = calloc(MAXVALUE * HISTOGRAM_LENGTH, sizeof(uint32_t));
    // frame to hold transposed arrays
    //tFrame = malloc(imx291_MAX_HEIGHT * imx291_MAX_WIDTH * sizeof(uint32_t));

    if ((configure == 1) || (!check_camera_running())) {
        config_pru(1, numFrames);
        pru_start();
        // Wait for sensor to initialize before starting i2c communication
        delay_ms(999);
        if (!check_camera_running()) {
            set_camera_lock();
        }
    } else {
        config_pru(0, numFrames);
        pru_start();
    }
    // only initialize the gain value if necessary
    // TODO: There's a possibly problematic assumption here that, if the gain 
    // is set, so are all the other register parameters.
    if (gain != 0) {
        if (configure && (gain != check_gain())) {
            imx291_init_readout(gain);
        }
        delay_ms(100);
    }

    // trigger readout
    send_pru_ready_signal();
    // Read out the frames

	//this control loop seems weird
    for (int i = 0; i < numFrames; i ++) {
        // capture a frame and place it in the frame buffer
        if (i == 0) {
            // initialize the timer after the readout if this is the "flush" frame
            exposure(frame, imx291_MAX_WIDTH * imx291_MAX_HEIGHT);
        } else if (i == numFrames - 1) {
            exposure(frame, imx291_MAX_WIDTH * imx291_MAX_HEIGHT);
        } else {
            exposure(frame, imx291_MAX_WIDTH * imx291_MAX_HEIGHT);
        }

        if (i > 0) { // throw out i = 0
            for (int j = 0; j < FRAMES_PER_TRANSFER; j ++) {
                // update histograms with data from this frame
                makeHistogramsAndSum(1, j);
            }
        } 
    }

    // print the total integration time
    // printf("Integration time (us): %ju\n", (uintmax_t)(clock_t)(end - start));

    wait_pru1_complete();



//    // calculate average
//    for (int i = 0; i < imx291_MAX_HEIGHT; i ++) {
//        for (int j = 0; j < imx291_MAX_WIDTH; j ++) {
//            frameAvg[i * imx291_MAX_WIDTH + j] = (uint8_t) (frameSum[i * imx291_MAX_WIDTH + j]/((numFrames - 1) * FRAMES_PER_TRANSFER));
//        }
//    }


    // Write stuff to file
    exposureWrite32(concatStr(fname, "test.dat"), (uint32_t *) frame, imx291_MAX_WIDTH * imx291_MAX_HEIGHT / 4);
    exposureWrite32(concatStr(fname, "singles.dat"), (uint32_t *) isolatedHisto, MAXVALUE);
    exposureWrite32(concatStr(fname, "pixels.dat"), (uint32_t *) pixelsHisto, MAXVALUE);
    exposureWrite32(concatStr(fname, "sum.dat"),  frameSum, imx291_MAX_WIDTH * imx291_MAX_HEIGHT);
    //exposureWrite32(concatStr(fname, "average.dat"), (uint32_t *) frameAvg, imx291_MAX_WIDTH * imx291_MAX_HEIGHT/4);
    exposureWrite32(concatStr(fname, "2dhisto.dat"), (uint32_t *) isolated2DHistogram, MAXVALUE * HISTOGRAM_LENGTH);

    pru_exit();
    return 0;
}

    

/*****************************************************************************
* Local Function Definitions                                                 *
*****************************************************************************/

// return char * to new string that's a concatenation of str1 and str2
static char *concatStr(char *str1, char *str2) {
    char *nameBuffer = malloc(sizeof(char) * BUFSIZE);
    strncpy(nameBuffer, str1, BUFSIZE);
    strncat(nameBuffer, str2, BUFSIZE);
    return nameBuffer;
}
    

// usage statement
static void usage(char *name) {
    printf("usage: %s threshold -o filename [-n number_of_exposures] [-g gain] [-c] [-r lower_bound upper_bound]\n", name);
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

// functions for creating a histogram of all pixels and of isolated above-thresold pixels, also 
// adding the frame onto a sum-of-frames array
// args: 
//  bgSubtract: 1 to subtract checkerboard pattern, 0 for the alternative
void makeHistogramsAndSum(int bgSubtract, int indx) {
    uint8_t bottom, top, left, right, center, corrected_threshold;
    uint8_t diag_correction = (uint8_t) -DIAG_CORRECTION;
    uint8_t *src = frame + indx * imx291_MAX_HEIGHT * imx291_MAX_WIDTH;

    // subtraction of dark level and systmatic row-to-row and checkerboard variation
    conditionFrame(src, bgSubtract);

    // initialize to 0 the array that will hold isolated events
    for (int i = 0; i < imx291_MAX_WIDTH * imx291_MAX_HEIGHT; i ++) {
        isolatedEventsBuffer[i] = 0;
    }

    for (int i = 1; i < imx291_MAX_HEIGHT - 1; i ++) {
        for (int j = 1; j < imx291_MAX_WIDTH - 1; j ++) {
            center = src[i * imx291_MAX_WIDTH + j];
            // gain correction for 'even' pixels
            // TODO: needed or not? 
            if (((i + j) % 2 == 0) && (threshold > diag_correction)) {
                corrected_threshold = threshold - diag_correction;
            } else {
                corrected_threshold = threshold;
//                threshold = threshold - diag_correction;
            }
            // TODO: two different modes would be helpful
//            //frameSum[i * imx291_MAX_WIDTH + j] += 1;
            if ((center >= threshold) && (center >= lowerBound) && (center <= upperBound)) {
                frameSum[i * imx291_MAX_WIDTH + j] += (uint32_t) center;
            }
            // all above-threshold pixels go into sum frame
//            if (center >= threshold) {
//                frameSum[i * imx291_MAX_WIDTH + j] += (uint32_t) center;
//            }
            pixelsHisto[center] += 1;
            // if all neighbors are below threshold
            if ((center >= threshold) && (center <= ceiling)) {
                //frameSum[i * imx291_MAX_WIDTH + j] += (uint32_t) center;
                top = src[(i + 1) * imx291_MAX_WIDTH + j];
                bottom = src[(i - 1) * imx291_MAX_WIDTH + j];
                right = src[i * imx291_MAX_WIDTH + (j + 1)];
                left = src[i * imx291_MAX_WIDTH + (j - 1)];
                if ((top < corrected_threshold) && (bottom < corrected_threshold) && (right < corrected_threshold) && (left < corrected_threshold)) {
                    isolatedHisto[center] += 1;
                    isolatedEventsBuffer[i * imx291_MAX_WIDTH + j] = center;
                }
            }
        }
    }
    // update the row-by-row histogram of isolated events
    update2DHisto(isolatedEventsBuffer, isolated2DHistogram);
}



//// given pointers to a 2d array of pixel values and a 2d row-by-row histogram
//// of values, update the histogram
//// corresponds to portrait orientation of sensor (i.e. the long side is perpendicular
//// to energy dispersion
//void update2DHisto(uint8_t *frame, uint32_t *histo) {
//    uint8_t value;
//    for (int i = 0; i < imx291_MAX_HEIGHT; i ++) {
//        for (int j = 0; j < imx291_MAX_WIDTH; j ++) {
//            value = frame[i * imx291_MAX_WIDTH + j];
//            if (value > 0) {
//                histo[i * MAXVALUE + value] += 1;
//            }
//        }
//    }
//}


// given pointers to a 2d array of pixel values and a 2d row-by-row histogram
// of values, update the histogram
// corresponds to landscape orientation of sensor (i.e. the long side is parallel
// to energy dispersion
void update2DHisto(uint8_t *frame, uint32_t *histo) {
    uint8_t value;
    for (int i = 0; i < imx291_MAX_HEIGHT; i ++) {
        for (int j = 0; j < imx291_MAX_WIDTH; j ++) {
            value = frame[i * imx291_MAX_WIDTH + j];
            if (value > 0) {
                histo[j * MAXVALUE + value] += 1;
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
    for (int i = 0; i < imx291_MAX_HEIGHT; i ++) {
        for (int j = 0; j < imx291_MAX_WIDTH; j ++) {
            if ((i + j)%2 == 0) {
                evenSum += arrA[i * imx291_MAX_WIDTH + j];
            } else {
                oddSum += arrA[i * imx291_MAX_WIDTH + j];
            }
        }
    }
    checkerboardEvenAverage = (uint8_t) (evenSum/(imx291_MAX_WIDTH * imx291_MAX_HEIGHT / 2));
    checkerboardOddAverage = (uint8_t) (oddSum/(imx291_MAX_WIDTH * imx291_MAX_HEIGHT / 2));
}

// subtract out "checkerboard" variation in a frame
// Note: this task has been offloaded to pru0
static void subtractDiagBias(uint8_t *arrA, int bgSubtract) {
    //printf("performing subtraction \n");
    int spread, excessEven, excessOdd;
    uint8_t curPix;
    // if even pixels have higher counts
    spread = checkerboardEvenAverage - checkerboardOddAverage;

    excessEven = spread / 2;
    excessOdd = excessEven - spread;
    if (bgSubtract) {
//        excessEven += darkLevel;
//        excessOdd += darkLevel;
    }
    for (int i = 0; i < imx291_MAX_HEIGHT; i ++) {
        for (int j = 0; j < imx291_MAX_WIDTH; j ++) {
            curPix = arrA[i * imx291_MAX_WIDTH + j];
            if ((i + j)%2 == 0) {
                if (excessEven < curPix) {
                    arrA[i * imx291_MAX_WIDTH + j] -= excessEven;
                } else {
                    arrA[i * imx291_MAX_WIDTH + j] = 0;
                }
            } else {
                if (excessOdd < curPix) {
                    arrA[i * imx291_MAX_WIDTH + j] -= excessOdd;
                } else {
                    arrA[i * imx291_MAX_WIDTH + j] = 0;
                }
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
static void conditionFrame(uint8_t *src, int bgSubtract) {
//    // subtract row-to-row variation
//    subtractRows(src, imx291_MAX_HEIGHT, imx291_MAX_WIDTH);
//    // transpose and move to tFrame
//    transpose(src,  tFrame,  imx291_MAX_HEIGHT, imx291_MAX_WIDTH);
//    // do the same on the transposed frame
//    subtractRows(tFrame, imx291_MAX_WIDTH, imx291_MAX_HEIGHT);
//    transpose(tFrame, src, imx291_MAX_WIDTH, imx291_MAX_HEIGHT);

//    // subtract "checkerboard" variation"
//    // check that the checkerboard parameters have been initialized
//    //printf("%d, %d\n", checkerboardEvenAverage, checkerboardOddAverage);
//    // correct for dark level and checkerboard pattern only if dark frame was provided
//    if (bgSubtract) {
//        subtractDiagBias(src, bgSubtract);
//    }

    // TODO: it would be less computationally intensive to mask out hot pixels instead
    // of doing a background subtraction
//    if (dark != NULL) {
//        subArrays(src, dark + 1, imx291_MAX_HEIGHT * imx291_MAX_WIDTH - 1);
//    }
    zeroColumns(src, masked_cols.rowNums, masked_cols.size);
}

// set values in the given columns to 0
void zeroColumns(uint8_t *arr, int *indices, int size) {
    for (int k = 0; k < size; k ++) {
        int j = indices[k];
        for (int i = 0; i < imx291_MAX_HEIGHT; i ++) {
            arr[i * imx291_MAX_WIDTH + j] = 0;
        }
    }
}

//subtract arr2 from arr1
void subArrays(uint8_t *arr1, uint8_t *arr2, int size) {
    for (int i = 0; i < size; i ++) {

        //to avoid underflows we set t corrected frame to 0 werever the
        //value in the dark frame exceeds it
        if (arr2[i] > arr1[i]) {
            arr1[i] = 0;
        } else {
            arr1[i] -= arr2[i];
        }

//        arr1[i] -= arr2[i];
    }
}


