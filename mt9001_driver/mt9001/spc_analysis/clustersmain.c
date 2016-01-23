#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include "clusters.h"

/*
//subtract arr2 from arr1
void subArrays(uint16_t arr1[], uint16_t arr2[], int size) {
    for (int i = 0; i < size; i ++) {
        //to avoid underflows we set t corrected frame to 0 werever the
        //value in the dark frame exceeds it
        if (arr2[i] > arr1[i]) {
            arr1[i] = 0;
        } else {
            arr1[i] -= arr2[i];
        }
    }
}
*/

//subtract arr2 from arr1
void subArrays(uint16_t *arr1, uint16_t *arr2, int size) {
    for (int i = 0; i < size; i ++) {
        //to avoid underflows we set t corrected frame to 0 werever the
        //value in the dark frame exceeds it
        if (arr2[i] > arr1[i]) {
            arr1[i] = 0;
        } else {
            arr1[i] -= arr2[i];
        }
    }
}


int main(int argc, char *argv[]) {
    //TODO: input validity 
    if (argc < 5 || argc > 6) {
        printf("usage: %s width height threshold filename [darkfilename]\n", argv[0]);
        return -1;
    }
    int DIMX = strtol(argv[1], NULL,  10);
    int DIMY = strtol(argv[2], NULL, 10);
    int THRESH = strtol(argv[3], NULL, 10);

    
  

    FILE *fp;
    char *fname = argv[4];
    if ((fp = fopen(fname, "rb")) == NULL) {
        printf("Can't open file: %s \n", fname);
        exit(1);
    }

    FILE *fdark; 
    if(argc == 6) {
        if ((fdark = fopen(argv[5], "rb")) == NULL) {
            printf("Can't open file: %s \n", argv[5]);
            exit(1);
        }
    } else {
        fdark = NULL;
    }

    uint16_t *frame = malloc(sizeof(uint16_t) * DIMX * DIMY);
    int frameSize = sizeof(uint16_t) * DIMX * DIMY; 
    //uint16_t frame[DIMX][DIMY]; 
    fread(frame, sizeof(frame[0]), frameSize/sizeof(frame[0]), fp);
    fclose(fp);

    if (fdark != NULL) {
        uint16_t *darkFrame = malloc(sizeof(uint16_t) * DIMX * DIMY);
        //uint16_t darkFrame[DIMX][DIMY]; 
        fread(darkFrame, sizeof(darkFrame[0]), frameSize/sizeof(darkFrame[0]), fdark);
        fclose(fdark);
        subArrays((uint16_t *) frame, (uint16_t *) darkFrame, DIMX * DIMY);
    }
    //set the 'fringe' of the frame to 0
    for (int i = 0; i < DIMX; i ++) {
        frame[i * DIMY] = 0;
        frame[i * DIMY + DIMY - 1] = 0; 
    }
    for (int i = 0; i < DIMY; i ++){
        frame[i] = 0; 
        frame[DIMY * (DIMX - 1) + i] = 0; 
    }
    int size = 8;//starting size of the Cluster array
    CCollection ccollection; 
    ccollection.maxsize = size;
    ccollection.num = 0;
    ccollection.array = (Cluster **) malloc(size * sizeof(Cluster*)); 

    //do the search
    searchFrame(&ccollection, frame, DIMX, DIMY, THRESH);

    //int histMax = 4 * pow(2, PIXDEPTH);//max counts for the histogram
    int histMax = pow(2, 16);//max counts for the histogram
    //array to store histogram of isolated pixel values
    int isolatedPixels[histMax]; 
    //array to store histogram of cluster values
    int allClusters[histMax];
    for (int i = 0; i < histMax; i ++) {
        allClusters[i] = 0; 
        isolatedPixels[i] = 0; 
    }

    //construct the histogram
    int counts; 
    for (int i = 0; i < ccollection.num; i ++) {
        counts = ccollection.array[i] -> value;
        if ((counts < histMax) && (counts > 0)) {
            allClusters[counts] += 1;
        }
        if (ccollection.array[i] -> size == 1) {
            isolatedPixels[counts] += 1;
        }
    }

    //output file name
    int bufSize = 256; 
    char out1[bufSize];
    char out2[bufSize];
    strncpy(out1, fname, bufSize); 
    strncpy(out2, fname, bufSize); 
    strncat(out1, "_clusters.hist", bufSize);
    strncat(out2, "_pixels.hist", bufSize);

    FILE* fout1;
    FILE* fout2;
    //clusters histogram

    if ((fout1 = fopen(out1, "w+")) != NULL) { 
        fwrite(allClusters, sizeof(int),histMax, fout1);
    }
    //isolated pixels histogram
    if ((fout2 = fopen(out2, "w+")) != NULL) { 
        fwrite(isolatedPixels, sizeof(int), histMax, fout2);
    }

/*
    //clusters histogram
    if (histMax != fwrite(allClusters, sizeof(int), histMax, fout1)) {
        exit(-1);
    }
    //isolated pixels histogram
    if (histMax != fwrite(isolatedPixels, sizeof(int), histMax, fout2)) {
        exit(-1);
    }
*/

}

