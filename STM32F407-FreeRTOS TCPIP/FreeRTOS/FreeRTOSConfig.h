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
#define configUSE_CO_ROUTINES 		       0  //��ʹ��Э��
#define configMAX_CO_ROUTINE_PRIORITIES  ( 2 )

/* ����ʹ�ܶ��塣 */
#define configUSE_COUNTING_SEMAPHORES    1  //���ü������ź���
#define configUSE_MUTEXES                1  //���û������ź���
#define configUSE_RECURSIVE_MUTEXES      1  //���õݹ黥���ź���
#define configUSE_QUEUE_SETS             1  //���ö��м�����
#define configUSE_TASK_NOTIFICATIONS     1  //ʹ������֪ͨ����

#define configUSE_PREEMPTION	  1  //ʹ��ʱ��Ƭ���ȣ������߱���ͬʱΪ1
#define configUSE_TIME_SLICING  1  //ʹ��ʱ��Ƭ���ȣ������߱���ͬʱΪ1

#define configUSE_PORT_OPTIMISED_TASK_SELECTION  1  //Ӳ������ǰ����

#define configIDLE_SHOULD_YIELD		 1  //��������Ϊ����ͬ���ȼ����û������ó�CPUʹ��Ȩ
#define configUSE_TICKLESS_IDLE    0  //��ʹ�ܵ͹���ticklessģʽ

/* ���Թ��ܶ��塣 */
#define configUSE_TRACE_FACILITY	 0  //��ʹ�ÿ��ӻ����ٵ���
#define configQUEUE_REGISTRY_SIZE  0  //��ʹ���ں˵�������Ϊ0����

/* Memory allocation related definitions. */
#define configSUPPORT_STATIC_ALLOCATION   0  //ʹ�ö�̬���������ں˶���
#define configSUPPORT_DYNAMIC_ALLOCATION  1  //�����ں˶�������RAM��OS���з���
/* ��heap_5.c��92�У�ʹ��RAM2�ռ��64K���к�RAM1�е�64K */
#define configAPPLICATION_ALLOCATED_HEAP  0  //�ɱ����������ڴ��(���䵽��RAM2)
#define configTOTAL_HEAP_SIZE  0  //OS�����ڴ�Ѵ�С(�ֽ�)��ʹ��heap_5��Ч

/* Hook function related definitions. */
#define configUSE_IDLE_HOOK			            1  //ʹ�ܿ��������Ӻ���
#define configUSE_MALLOC_FAILED_HOOK        1  //ʹ���ڴ���䲻�����Ӻ���
#define configCHECK_FOR_STACK_OVERFLOW      2  //ʹ�ö�ջ�����ⷽ��2��ʹ�ܶ�ջ������Ӻ���
#define configUSE_TICK_HOOK			            0  //��ʹ��ʱ��Ƭ���Ӻ���
#define configUSE_DAEMON_TASK_STARTUP_HOOK  0

#define configCPU_CLOCK_HZ			       ( ( unsigned long ) 168000000 )  //ϵͳʱ��
#define configTICK_RATE_HZ			       ( ( unsigned int ) 100 )  //�δ�ʱ��Ƶ��
#define configUSE_16_BIT_TICKS		     0  //32λ�δ�ʱ��
#define configOVERRIDE_DEFAULT_TICK_CONFIGURATION  1  //�Լ���ʼ���δ�ʱ����ûʵ�ֵ͹��Ĳ��ֺ�����hange_v��
#define configGENERATE_RUN_TIME_STATE  0  //������ʱ��ͳ�ƹ���
	
/*��UCOS�෴��FreeRTOS��������ȼ�ΪconfigMAX_PRIORITIES-1��������ȼ�Ϊ0*/
#define configMAX_PRIORITIES		      ( 5 )  //����������ȼ�����Ҫ����32��������Ϊһ������Ӧ�õ���Сֵ
#define configMINIMAL_STACK_SIZE	    ( ( unsigned short ) 128 )  //����������С��ջ
#define configMAX_TASK_NAME_LEN		    ( 16 )  //��������󳤶�

/* Software timer related definitions. */
#define configUSE_TIMERS               0  //��ʹ�������ʱ��
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

/* ��Դ��������ã����ô��������ȼ�λ����OS�ɹ���Ĵ�����������ȼ��ж� */
#define configPRIO_BITS                       4     //MCUʹ�ü�λ���ȼ�
#define configKERNEL_INTERRUPT_PRIORITY 		  0xF0  //0xF<<4, STM32ʹ�ø�4λ�����ȼ�
#define configMAX_SYSCALL_INTERRUPT_PRIORITY  0x50  //0x5<<4, OS�ɹ����������ȼ�

#endif /* FREERTOS_CONFIG_H */



















