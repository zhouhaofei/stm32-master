#ifndef _MQUEUE_H_
#define _MQUEUE_H_

#include "stm32f4xx.h"

typedef struct{
  u8 *pointer;  //指向队列元素的指针
	u16 front;  //队列第一个元素
	u16 rear;  //队列最后一个元素的下一个元素
	u16 count;  //队列元素个数
	u16 maxSize;  //循环队列最大存储空间
}ByteQueue;

u8 byteQueueCreate(ByteQueue *queue, u16 size);
u8 byteQueueIsFull(ByteQueue *queue); 
u8 byteQueueIsEmpty(ByteQueue *queue);
u8 byteQueueEnter(ByteQueue *queue, u8 dat);
u8 byteQueueExit(ByteQueue *queue);
u8 byteQueueIndex(ByteQueue *queue, u16 index);
u16 byteQueueCopyIn(ByteQueue *queue, u16 len, u8 buf[]);
u16 byteQueueCopyOut(ByteQueue *queue, u16 len, u8 dat[]);
void byteQueueClear(ByteQueue *queue);
void byteQueueDestroy(ByteQueue *queue);

#endif



















