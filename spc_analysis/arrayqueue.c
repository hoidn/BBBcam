//implementation of a variable-sized array queue

//---------------------------------------------------------------
// File: Code130_Queue.c
// Purpose: Implementation file for a demonstration of a queue  
//      implemented as an array.    Data type: Character
// Programming Language: C
// Author: Dr. Rick Coleman
// Date: February 11, 2002
//
//Modified for use in clusters.c by Oliver Hoidn, 3/20/2014
//---------------------------------------------------------------
#include "arrayqueue.h"
#include "clusters.h"

// Declare these as static so no code outside of this source
// can access them.
static int head, tail;  // Declare global indices to head and tail of queue
static Point theQueue[MAX_SIZE]; // The queue

//--------------------------------------------
// Function: InitQueue()
// Purpose: Initialize queue to empty.
// Returns: void
//--------------------------------------------
void InitQueue()
{
    head = tail = -1;
}

//--------------------------------------------
// Function: ClearQueue()
// Purpose: Remove all items from the queue
// Returns: void
//--------------------------------------------
void ClearQueue()
{
    head = tail = -1; // Reset indices to start over
}

int size(){
    return tail - head; 
}
//--------------------------------------------
// Function: Enqueue()
// Purpose: Enqueue an item into the queue.
// Returns: TRUE if enqueue was successful
//      or FALSE if the enqueue failed.
// Note: We let head and tail continuing 
//      increasing and use [head % MAX_SIZE] 
//      and [tail % MAX_SIZE] to get the real
//      indices.  This automatically handles
//      wrap-around when the end of the array
//      is reached.
//--------------------------------------------
int push(Point *pt)
{
    // Check to see if the Queue is full
    if(isFull()) return FALSE;

    // Increment tail index
    tail++;
    // Add the item to the Queue
    theQueue[tail % MAX_SIZE].x = pt -> x;
    theQueue[tail % MAX_SIZE].y = pt -> y;
    return TRUE;
}

//--------------------------------------------
// Function: Dequeue()
// Purpose: Dequeue an item from the Queue.
// Returns: TRUE if dequeue was successful
//      or FALSE if the dequeue failed.
//--------------------------------------------
Point* pop()
{
    Point *pt;

    // Check for empty Queue
    if(isEmpty()) return '\0';  // Return null character if queue is empty
    else
    {
        head++;
        pt = (Point *) &theQueue[head % MAX_SIZE];     // Get character to return
        return pt;              // Return popped character
    }
}

//--------------------------------------------
// Function: isEmpty()
// Purpose: Return true if the queue is empty
// Returns: TRUE if empty, otherwise FALSE
// Note: C has no boolean data type so we use
//  the defined int values for TRUE and FALSE
//  instead.
//--------------------------------------------
int isEmpty()
{
    return (head == tail);
}

//--------------------------------------------
// Function: isFull()
// Purpose: Return true if the queue is full.
// Returns: TRUE if full, otherwise FALSE
// Note: C has no boolean data type so we use
//  the defined int values for TRUE and FALSE
//  instead.
//--------------------------------------------
int isFull()
{
    // Queue is full if tail has wrapped around
    //  to location of the head.  See note in
    //  Enqueue() function.
    return ((tail - MAX_SIZE) == head);
}
