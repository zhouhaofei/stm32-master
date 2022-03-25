/*
 * FreeRTOS Kernel V10.2.1
 * Copyright (C) 2019 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://www.FreeRTOS.org
 * http://aws.amazon.com/freertos
 *
 * 1 tab == 4 spaces!
 */

#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

/*-----------------------------------------------------------
 * Application specific definitions.
 *
 * These definitions should be adjusted for your particular hardware and
 * application requirements.
 *
 * THESE PARAMETERS ARE DESCRIBED WITHIN THE 'CONFIGURATION' SECTION OF THE
 * FreeRTOS API DOCUMENTATION AVAILABLE ON THE FreeRTOS.org WEB SITE. 
 *
 * See http://www.freertos.org/a00110.html
 *----------------------------------------------------------*/

/* Co-routine definitions. */
#define configUSE_CO_ROUTINES 		       0  //不使用协程
#define configMAX_CO_ROUTINE_PRIORITIES  ( 2 )

/* 功能使能定义。 */
#define configUSE_COUNTING_SEMAPHORES    1  //启用计数型信号量
#define configUSE_MUTEXES                1  //启用互斥型信号量
#define configUSE_RECURSIVE_MUTEXES      1  //启用递归互斥信号量
#define configUSE_QUEUE_SETS             1  //启用队列集功能
#define configUSE_TASK_NOTIFICATIONS     1  //使用任务通知功能

#define configUSE_PREEMPTION	  1  //使能时间片调度，和下者必须同时为1
#define configUSE_TIME_SLICING  1  //使能时间片调度，和上者必须同时为1

#define configUSE_PORT_OPTIMISED_TASK_SELECTION  1  //硬件计算前导零

#define configIDLE_SHOULD_YIELD		 1  //空闲任务为处于同优先级的用户任务让出CPU使用权
#define configUSE_TICKLESS_IDLE    0  //不使能低功耗tickless模式

/* 调试功能定义。 */
#define configUSE_TRACE_FACILITY	 0  //不使用可视化跟踪调试
#define configQUEUE_REGISTRY_SIZE  0  //不使用内核调试器设为0即可

/* Memory allocation related definitions. */
#define configSUPPORT_STATIC_ALLOCATION   0  //使用动态方法创建内核对象
#define configSUPPORT_DYNAMIC_ALLOCATION  1  //创建内核对象所需RAM从OS堆中分配
/* 在heap_5.c的92行，使用RAM2空间的64K所有和RAM1中的64K */
#define configAPPLICATION_ALLOCATED_HEAP  0  //由编译器分配内存堆(分配到了RAM2)
#define configTOTAL_HEAP_SIZE  0  //OS可用内存堆大小(字节)，使用heap_5无效

/* Hook function related definitions. */
#define configUSE_IDLE_HOOK			            1  //使能空闲任务钩子函数
#define configUSE_MALLOC_FAILED_HOOK        1  //使能内存分配不够钩子函数
#define configCHECK_FOR_STACK_OVERFLOW      2  //使用堆栈溢出检测方法2并使能堆栈溢出钩子函数
#define configUSE_TICK_HOOK			            0  //不使能时间片钩子函数
#define configUSE_DAEMON_TASK_STARTUP_HOOK  0

#define configCPU_CLOCK_HZ			       ( ( unsigned long ) 168000000 )  //系统时钟
#define configTICK_RATE_HZ			       ( ( unsigned int ) 100 )  //滴答定时器频率
#define configUSE_16_BIT_TICKS		     0  //32位滴答定时器
#define configOVERRIDE_DEFAULT_TICK_CONFIGURATION  1  //自己初始化滴答定时器，没实现低功耗部分函数（hange_v）
#define configGENERATE_RUN_TIME_STATE  0  //不开启时间统计功能
	
/*和UCOS相反，FreeRTOS中最高优先级为configMAX_PRIORITIES-1，最低优先级为0*/
#define configMAX_PRIORITIES		      ( 5 )  //最大任务优先级（不要大于32），设置为一个满足应用的最小值
#define configMINIMAL_STACK_SIZE	    ( ( unsigned short ) 128 )  //空闲任务最小堆栈
#define configMAX_TASK_NAME_LEN		    ( 16 )  //任务名最大长度

/* Software timer related definitions. */
#define configUSE_TIMERS               0  //不使用软件定时器
#define configTIMER_TASK_PRIORITY      3
#define configTIMER_QUEUE_LENGTH       10
#define configTIMER_TASK_STACK_DEPTH   configMINIMAL_STACK_SIZE

/* Set the following definitions to 1 to include the API function, or zero
to exclude the API function. */
#define INCLUDE_xSemaphoreGetMutexHolder      1  //xQueueGetMutexHolder()
#define INCLUDE_xTaskAbortDelay               1  //xTaskAbortDelay()
#define INCLUDE_vTaskDelay                    1  //vTaskDelay()
#define INCLUDE_vTaskDelayUntil               1  //vTaskDelayUntil()
#define INCLUDE_vTaskDelete                   1  //vTaskDelete()
#define INCLUDE_xTaskGetCurrentTaskHandle     1  //xTaskGetCurrentTaskHandle()
#define INCLUDE_xTaskGetHandle                1  //xTaskGetHandle()
#define INCLUDE_xTaskGetIdleTaskHandle        1  //xTaskGetIdleTaskHandle()
#define INCLUDE_xTaskGetSchedulerState        1  //xTaskGetSchedulerState()
#define INCLUDE_uxTaskGetStackHighWaterMark   1  //uxTaskGetStackHighWaterMark()
#define INCLUDE_vTaskPrioritySet              1  //vTaskPrioritySet()
#define INCLUDE_uxTaskPriorityGet             1  //vTaskPrioritySet()
#define INCLUDE_xResumeFromISR                1  //xResumeFromISR()
#define INCLUDE_eTaskGetState                 1  //eTaskGetState()
#define INCLUDE_vTaskSuspend                  1  //vTaskSuspend()
#define INCLUDE_xTimerPendFunctionCall        0  //xTimerPendFunctionCall()

/* 针对处理器配置，配置处理器优先级位数和OS可管理的处理器最高优先级中断 */
#define configPRIO_BITS                       4     //MCU使用几位优先级
#define configKERNEL_INTERRUPT_PRIORITY 		  0xF0  //0xF<<4, STM32使用高4位做优先级
#define configMAX_SYSCALL_INTERRUPT_PRIORITY  0x50  //0x5<<4, OS可管理的最大优先级

#endif /* FREERTOS_CONFIG_H */



















