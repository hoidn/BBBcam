#include "clusters.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
//#include "queue.h"
#include "arrayqueue.h"



Point* createPoint(int i, int j) {
    Point* point = (Point *) malloc(sizeof(Point));
    point -> x = i; 
    point -> y = j; 
    return point; 
}

/*
insert a Cluster* into the array of a CCollection, allocating more memory
if necessary
*/
void insert(CCollection* coll, Cluster* cluster) {
    if (coll -> maxsize <= coll -> num) {
        int newsize = 2 * coll -> maxsize;
        coll -> array = (Cluster **) realloc(coll -> array, newsize * sizeof(Cluster*)); 
        coll -> maxsize = newsize; 
    }
    coll -> num += 1; 
    coll -> array[coll -> num - 1] = cluster; 
}
/*
From starting coordinates i and j in the frame, find the cluster composed of 
all connected above-threshold pixels
pre: frame[i][j] > threshold
n, m: dimensions of the array
i, j: coords of current pixel
*/
Cluster* searchClust(uint16_t *frame, uint8_t *explored, int n, int m, int i, int j, int threshold){
    uint16_t top, bottom, left, right; 
    //queue to store pixels that need to be explored
    //Queue queue = createQueue();
    Cluster* cluster = (Cluster *) malloc(sizeof(Cluster));
    cluster -> size = 0; 
    cluster -> value = 0;
    Point *point = createPoint(i, j); 
    push(point); 
    //mark point as explored
    explored[i * m + j] = 1; 
    while (size() > 0) {
        //dequeue and update our coordinates 
        point = pop();
        i = point -> x; 
        j = point -> y;
        //update cluster size and value
        cluster -> size += 1;
        cluster -> value += (int) frame[i * m  + j];
        top = frame[i * m + j + 1]; 
        bottom = frame[i * m + j - 1]; 
        left = frame[(i - 1) * m + j]; 
        right = frame[(i + 1) * m + j]; 
        //add above-threshold adjacent pixels to the queue
        if (top > threshold && explored[i * m + j + 1] == 0) {
            point = createPoint(i, j + 1); 
            push(point);
            explored[i * m + j + 1] = 1; 
       }
        if (bottom > threshold && explored[i * m + j - 1] == 0) {
            point = createPoint(i, j - 1); 
            push(point);
            explored[i * m + j - 1] = 1; 
        }   
        if (left > threshold && explored[(i - 1) * m + j] == 0) {
            point = createPoint(i - 1, j); 
            push(point);
            explored[(i - 1) * m + j] = 1; 
        }
        if (right > threshold && explored[(i + 1) * m + j] == 0) {
            point = createPoint(i + 1, j); 
            push(point);
            explored[(i + 1) * m + j] = 1; 
        }
    }
    return cluster; 
}
    
                

/*
Perform cluster search. 
collectionptr: pointer to CCollection to store the clusters
arr: the frame data
n, m: frame dimensions
threshold: noise threshold
*/
void searchFrame(CCollection *collectionptr, uint16_t *arr, int n, int m, int threshold) {
    //TODO: set pixels on the 'rim' equal to 0 (in main)
    //boolean array that record which pixels have been explored by the BFS. 
    Cluster* clusterptr; 
    //Queue queue = createQueue();
    uint8_t *explored = calloc(n * m * sizeof(uint8_t));
//    for(int i = 0; i < n; i ++) {
//        for (int j = 0; j < m; j ++) {
//            explored[i][j] = 0; 
//        }
//    }
    for (int i = 1; i < n - 1; i ++) {
        for (int j = 1; j < m - 1; j ++) {
            if (explored[i][j] == 0 && arr[i * m + j] > (uint16_t) threshold) {
                clusterptr = searchClust(arr, &explored[0][0], n, m, i, j, threshold); 
                insert(collectionptr, clusterptr); 
            }
        }
    }
}

// given a CCollection, initialize its fields and allocate memory for its array
void init_CCollection(CCollection ccollection) {
    ccollection.maxsize = 8;
    ccollection.num = 0;
    ccollection.array = (Cluster **) malloc(size * sizeof(Cluster*)); 
}


// given a CCollection, reinitalize its fields
void init_CCollection(CCollection ccollection) {
    ccollection.maxsize = 8;
    ccollection.num = 0;
}
