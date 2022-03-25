/********************************************************
Created by hange_v;
FreeRTOS中一些需要用户实现的函数可以放在这里面；
********************************************************/

#include "stm32f4xx.h"
#include "public.h"
#include "FreeRTOS.h"
#include "task.h"

//空闲任务钩子函数，可以用来喂狗。。。
//这里面绝对不能挂起！！！
void vApplicationIdleHook(void)
{
  mIWDG_feed();  //喂看门狗
}

//内存分配不够钩子函数
void vApplicationMallocFailedHook(void)
{
	taskENTER_CRITICAL();  //进临界状态，关闭所有中断
	mPrintf("OS内存堆不够分配\r\n");
  STM32_SYSTEM_RESET;
}

//堆栈溢出检测钩子函数
void vApplicationStackOverflowHook(TaskHandle_t  xTaskHandle, char*  pcTaskName)
{
  mPrintf("\"%s\"任务堆栈溢出\r\n", pcTaskName);
}



















