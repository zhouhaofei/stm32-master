/********************************************************
Created by hange_v;
FreeRTOS��һЩ��Ҫ�û�ʵ�ֵĺ������Է��������棻
********************************************************/

#include "stm32f4xx.h"
#include "public.h"
#include "FreeRTOS.h"
#include "task.h"

//���������Ӻ�������������ι��������
//��������Բ��ܹ��𣡣���
void vApplicationIdleHook(void)
{
  mIWDG_feed();  //ι���Ź�
}

//�ڴ���䲻�����Ӻ���
void vApplicationMallocFailedHook(void)
{
	taskENTER_CRITICAL();  //���ٽ�״̬���ر������ж�
	mPrintf("OS�ڴ�Ѳ�������\r\n");
  STM32_SYSTEM_RESET;
}

//��ջ�����⹳�Ӻ���
void vApplicationStackOverflowHook(TaskHandle_t  xTaskHandle, char*  pcTaskName)
{
  mPrintf("\"%s\"�����ջ���\r\n", pcTaskName);
}



















