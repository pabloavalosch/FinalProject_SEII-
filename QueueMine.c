/*
 * Queue.c
 *
 *  Created on: 26 sep. 2023
 *      Author: pablo
 */

#include <QueueMine.h>

uint8_t Queue_Init(Queue_type myQueue)
{
    myQueue.front = 0;
    myQueue.rear = 0;

    return TRUE;
}

uint8_t Queue_Enqueue(Queue_type * myQueue, uint8_t * pStr)
{
    if( QUEUE_SIZE <= myQueue->rear )
    {
        // Not enough space in the queue
        return FALSE;
    }

    // Enqueue the string into the queue of pointers
    myQueue->queue[myQueue->rear++] = pStr;

    if( QUEUE_SIZE == myQueue->rear)
    {
    	myQueue->rear--;
    }
    return TRUE;
}

uint8_t Queue_Dequeue(Queue_type * myQueue, uint8_t * pStr)
{
	if( 0 == myQueue->count )
	{
		// Queue is empty, nothing to Dequeue
		return FALSE;
	}

	// Dequeue the string from the queue of pointers
	myQueue->queue[myQueue->front] = 0;
	myQueue->count--;

	return TRUE;
}

uint8_t Queue_is_empty(Queue_type* myQueue)
{
	return (myQueue->count);
}

Queue_type get_myQueue(Queue_type myQueue)
{
	return (myQueue);
}
