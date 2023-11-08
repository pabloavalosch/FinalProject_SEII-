/*
 * Queue.h
 *
 *  Created on: 26 sep. 2023
 *      Author: pablo
 */

#ifndef QUEUEMINE_H_
#define QUEUEMINE_H_

#include <stdint.h>

#define QUEUE_SIZE		16U

enum {FALSE, TRUE};

typedef struct {
    uint8_t *queue[QUEUE_SIZE];
    uint8_t front;
    uint8_t rear;
    uint8_t count;
} Queue_type;

uint8_t Queue_Init(Queue_type myQueue);
uint8_t Queue_Enqueue(Queue_type * myQueue, uint8_t * pStr);
uint8_t Queue_Dequeue(Queue_type * myQueue, uint8_t * pStr);
uint8_t Queue_is_empty(Queue_type * myQueue);
Queue_type get_myQueue(Queue_type myQueue);

#endif /* QUEUEMINE_H_ */
