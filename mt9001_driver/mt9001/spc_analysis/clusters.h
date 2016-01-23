#ifndef CLUSTERS_H
#define CLUSTERS_H

#include <stdint.h>
//#include "clusters.h"

#define PIXDEPTH 10

//struct to store coordinates of a single pixel
typedef struct {
    int x; 
    int y;
    } Point;

//struct to represent a single cluster
typedef struct{
    int size; //number of pixels in cluster
    int value;//total signal in cluster
} Cluster;

//struct to store many clusters
typedef struct{
    int num; //number of Cluster structs already sotred in the array
    int maxsize; //max size of the array
    Cluster** array; 
} CCollection;

void searchFrame(CCollection *collectionptr, uint16_t *arr, int n, int m, int  threshold);

#endif 
