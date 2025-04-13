#include "circular_queue.h"



void circular_queue_create( CircularQueue*q, uint8_t *Buffer,uint32_t size )
{
    
    q->nq =0;      
    q->dq =0;      
    q->mask = size-1;    
    q->buffer = Buffer;

}

bool CQUEUE_Enqueue( CircularQueue *q, uint8_t byte )
{   bool status = false;
    uint32_t write_index = q->nq+1 & q->mask;

    if(write_index != q->dq)
    {
    
        q->buffer[q->nq] = byte;
        q->nq = write_index;
        status=true;
        
    }

    
    
    return status;

}

bool CQUEUE_Dequeue( CircularQueue *q, uint8_t *ptr2data )
{
    
    bool status = false;
    uint32_t read_index = q->dq;
    uint32_t write_index = q->nq;

    if(write_index != read_index)
    {
        
        *ptr2data = q->buffer[read_index];
        read_index= (read_index+1) & q->mask;
        status=true;
    }

q->dq = read_index;
return status;
}

    


