#include "mQueue.h"
#include "stdlib.h"
#include "string.h"

u8 byteQueueCreate(ByteQueue *queue, u16 size)
{
	queue->pointer = (u8 *)malloc(sizeof(u8) * size);
	if(queue->pointer == NULL)
	{
		return 0;
	}
	queue->front = 0;
	queue->rear = 0;
	queue->count = 0;
	queue->maxSize = size;  
	return 1;
}

u8 byteQueueIsFull(ByteQueue *queue)
{
  if(queue->front == (queue->rear + 1) % (queue->maxSize))  //��������Ԥ��һ���ռ䲻��
	{
		return 1;
	}		
	else
		return 0;
}

u8 byteQueueIsEmpty(ByteQueue *queue)
{
  if(queue->front == queue->rear)  //����Ϊ��
	{
		return 1;
	}
	else
		return 0;
}

u8 byteQueueEnter(ByteQueue *queue, u8 dat)  
{
  if(queue->front == (queue->rear + 1) % (queue->maxSize))  //��������Ԥ��һ���ռ䲻��
	{
		return 0;
	}
	else
	{
	  queue->pointer[queue->rear] = dat;
		queue->rear = (queue->rear + 1) % (queue->maxSize);
		queue->count++;
		return 1;
	}
}

u8 byteQueueExit(ByteQueue *queue)
{
	u8 temp = 0;
  if(queue->front == queue->rear)  //����Ϊ��
	{
		return temp;
	}
	else
	{
	  temp = queue->pointer[queue->front];
		queue->front = (queue->front + 1) % (queue->maxSize);
		queue->count--;
		return temp;
	}
}

u8 byteQueueIndex(ByteQueue *queue, u16 index)
{
	if(index >= queue->count)
	{
	  return 0;
	}
	if(queue->front == queue->rear)  //ͷ����β
	{
		return 0;
	}
	return queue->pointer[(queue->front + index) % queue->maxSize];
}

u16 byteQueueCopyIn(ByteQueue *queue, u16 len, u8 buf[])
{
	if(len > (queue->maxSize - queue->count - 1))  //����Ĳ��ܴ���ʣ��ռ��һ
	{
	  len = queue->maxSize - queue->count - 1;
	}
	if(queue->rear < queue->front)  //βС��ͷ
	{
	  memcpy(queue->pointer + queue->rear, buf, len);
		queue->rear += len;
		queue->count += len;
	}
	else  //β���ڵ���ͷ
	{
		if((queue->rear + len) <= queue->maxSize)  //ֱ�Ӹ���
		{
		  memcpy(queue->pointer + queue->rear, buf, len);
			queue->rear += len;
			queue->rear %= queue->maxSize;
			queue->count += len;
		}
	  else  //��Ҫ�����θ���
		{
			u16 num = queue->maxSize - queue->rear;
		  memcpy(queue->pointer + queue->rear, buf, num);
			memcpy(queue->pointer, buf + num, len - num);
			queue->rear = (len - num);
			queue->count += len;
		}
	}
	return len;
}

u16 byteQueueCopyOut(ByteQueue *queue, u16 len, u8 dat[])
{
  if((queue->front == queue->rear) || (len == 0))  //�ն��л�Ҫ�����ĳ���Ϊ0
	{
		return 0;
	}
	if(len > queue->count)  //�����Ĳ��ܴ��ڴ��ڵ�
	{
	  len = queue->count;
	}
	if(queue->front < queue->rear)  //ͷС��β
	{
	  memcpy(dat, queue->pointer + queue->front, len);
		queue->front += len;
		queue->count -= len;
	}
	else  //ͷ����β
	{
	  if((queue->front + len) <= queue->maxSize)
		{
		  memcpy(dat, queue->pointer + queue->front, len);
			queue->front += len;
			queue->front %= queue->maxSize;
			queue->count -= len;
		}
		else
		{
		  u16 num = queue->maxSize - queue->front;
			memcpy(dat, queue->pointer + queue->front, num);
			memcpy(dat + num, queue->pointer, len - num);
			queue->front = (len - num);
			queue->count -= len;
		}
	}
	return len;
}

void byteQueueClear(ByteQueue *queue)
{
	memset(queue->pointer, 0, queue->maxSize);
  queue->front = 0;
	queue->rear = 0;
	queue->count = 0;
}

void byteQueueDestroy(ByteQueue *queue)
{
	memset(queue->pointer, 0, queue->maxSize);
	free(queue->pointer);
	queue->front = 0;
	queue->rear = 0;
	queue->count = 0;
}



















