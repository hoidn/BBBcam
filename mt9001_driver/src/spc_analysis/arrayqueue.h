#ifndef ARRAY_QUEUE_H
#define ARRAY_QUEUE_H

#include <stdio.h>
#include "clusters.h"

// Max length of the queue = max number of clusters in a 1024 x 1280 frame
#define MAX_SIZE 655360       

// List Function Prototypes
void InitQueue();             // Initialize the queue
void ClearQueue();            // Remove all items from the queue
int size();			//current size of the queue
int push(Point *pt);         // Enter an item in the queue
Point* pop();               // Remove an item from the queue
int isEmpty();                // Return true if queue is empty
int isFull();                 // Return true if queue is full

// Define TRUE and FALSE if they have not already been defined
#ifndef FALSE
#define FALSE (0)
#endif
#ifndef TRUE
#define TRUE (!FALSE)
#endif

#endif // End of queue header
