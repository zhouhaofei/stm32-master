#ifndef _MQUEUE_H_
#define _MQUEUE_H_

#include "stm32f4xx.h"

typedef struct{
  u8 *pointer;  //ָ�����Ԫ�ص�ָ��
	u16 front;  //���е�һ��Ԫ��
	u16 rear;  //�������һ��Ԫ�ص���һ��Ԫ��
	u16 count;  //����Ԫ�ظ���
	u16 maxSize;  //ѭ���������洢�ռ�
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



















