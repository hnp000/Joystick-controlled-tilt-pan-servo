#ifndef QUEUE_H_
#define QUEUE_H_


#include <stdint.h>
#include <stdbool.h>


typedef struct
{
    
    uint32_t nq;      
    uint32_t dq;      
    uint32_t mask;    
    uint8_t *buffer;  
    
} CircularQueue;

void circular_queue_create( CircularQueue*q, uint8_t *Buffer,uint32_t size );

bool CQUEUE_Enqueue( CircularQueue *q, uint8_t byte );
bool CQUEUE_Dequeue( CircularQueue *q, uint8_t *ptr2data );


#endif