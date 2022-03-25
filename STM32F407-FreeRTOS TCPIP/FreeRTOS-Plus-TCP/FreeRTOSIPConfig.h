/*
 * FreeRTOS Kernel V10.2.1
 * Copyright (C) 2017 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
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


/*****************************************************************************
 *
 * See the following URL for configuration information.
 * http://www.freertos.org/FreeRTOS-Plus/FreeRTOS_Plus_TCP/TCP_IP_Configuration.html
 *
 *****************************************************************************/

#ifndef FREERTOS_IP_CONFIG_H
#define FREERTOS_IP_CONFIG_H

#include "FreeRTOS.h"
#include "list.h"
#include "semphr.h"

//影响TCP/IP堆栈任务执行行为的常量
#define ipconfigIP_TASK_PRIORITY          ( configMAX_PRIORITIES - 1 )  //TCP/IP任务优先级
#define ipconfigIP_TASK_STACK_SIZE_WORDS	( configMINIMAL_STACK_SIZE * 5 )  //TCP/IP任务的堆栈大小
#define ipconfigUSE_NETWORK_EVENT_HOOK    1  //调用 网络事件 钩子函数
#define ipconfigEVENT_QUEUE_LENGTH		    ( ipconfigNUM_NETWORK_BUFFER_DESCRIPTORS + 5 )  //事件队列必须至少比网络缓冲区的总数大5
                                          //将事件从应用程序任务发送到IP堆栈，可以一次排队等待处理的最大事件数

//调试，跟踪和记录设置
#define vLoggingPrintf  printf  //定义为printf

#define ipconfigHAS_DEBUG_PRINTF	  0
#if( ipconfigHAS_DEBUG_PRINTF == 1 )
	#define FreeRTOS_debug_printf(X)	vLoggingPrintf X
#endif

#define ipconfigHAS_PRINTF			    0
#if( ipconfigHAS_PRINTF == 1 )
	#define FreeRTOS_printf(X)			  vLoggingPrintf X
#endif

#define ipconfigTCP_MAY_LOG_PORT(xPort)  (xPort != 80)  //举例：不为80端口生成日志
#define ipconfigCHECK_IP_QUEUE_SPACE     1  //应用程序任务发送到IP堆栈，一次排队等待处理的最大事件数
#define ipconfigWATCHDOG_TIMER()            //IP任务每次迭代调用的宏，可以做喂狗

//硬件和驱动程序特定设置
#define ipconfigBYTE_ORDER    pdFREERTOS_LITTLE_ENDIAN   //处理器小端
/* FreeRTOSTCPIP只支持硬件生成IP校验和，不支持TCP/UDP/ICMP校验和 */
#define ipconfigDRIVER_INCLUDED_TX_IP_CHECKSUM       1   //硬件计算ipv4校验
#define ipconfigDRIVER_INCLUDED_RX_IP_CHECKSUM       1
#define ipconfigETHERNET_DRIVER_FILTERS_FRAME_TYPES	 1   //硬件过滤MAC
//#define ipconfigETHERNET_DRIVER_FILTERS_PACKETS
#define ipconfigNETWORK_MTU                          1500
#define ipconfigNUM_NETWORK_BUFFER_DESCRIPTORS		   10  //可用于TCP/IP堆栈的网络缓冲区的总数
#define ipconfigFILTER_OUT_NON_ETHERNET_II_FRAMES    1   //非以太网II格式的以太网帧将被丢弃
//#define ipconfigUSE_LINKED_RX_MESSAGES
#define ipconfigZERO_COPY_TX_DRIVER                  0   //发送不使用0复制方案
#define ipconfigZERO_COPY_RX_DRIVER                  0   //接收不使用0复制方案
//#define ipconfigBUFFER_PADDING
//#define ipconfigPACKET_FILLER_SIZE

//TCP特定常数
#define ipconfigUSE_TCP				            1    //启用TCP
#define ipconfigTCP_TIME_TO_LIVE		      128  //TCP TTL
#define ipconfigTCP_TX_BUFFER_LENGTH      (2 * ipconfigTCP_MSS)  //每个TCP套接字都有一个用于接收的缓冲区和一个用于传输的单独缓冲区，
#define ipconfigTCP_RX_BUFFER_LENGTH      (2 * ipconfigTCP_MSS)
#define ipconfigUSE_TCP_WIN			          0    //不使用滑动窗口(单位为ipconfigTCP_MSS)
#define ipconfigTCP_WIN_SEG_COUNT		      32  //池中描述符的数量
#define ipconfigUSE_TCP_TIMESTAMPS        0    //不使用TCP时间戳
#define ipconfigTCP_MSS                   400  //所有TCP数据包的MSS值（以字节为单位）
#define ipconfigTCP_KEEP_ALIVE				    1    //开启保活
#define ipconfigTCP_KEEP_ALIVE_INTERVAL		20   //启动保活时间(单位是 秒)
#define ipconfigTCP_HANG_PROTECTION			  1    //指定的时间内套接字上没有状态更改，则将套接字标记为已关闭
#define ipconfigTCP_HANG_PROTECTION_TIME	30   //上者的时间(单位是 秒)
#define ipconfigIGNORE_UNKNOWN_PACKETS    0    //先设为0吧

//UDP特定常数
#define ipconfigUDP_TIME_TO_LIVE		           128  //UDP TTL
#define ipconfigUDP_MAX_SEND_BLOCK_TIME_TICKS  ( 5000 / portTICK_PERIOD_MS )  //可理解为发送超时时间
#define ipconfigUDP_MAX_RX_PACKETS             5    //UDP套接字的Rx队列中可以存在的最大数据包数

//其他影响套接字行为的常量
#define ipconfigINCLUDE_FULL_INET_ADDR	         1  //先设为1
#define ipconfigALLOW_SOCKET_SEND_WITHOUT_BIND   1  //客户端自动绑定端口
#define ipconfigSOCK_DEFAULT_RECEIVE_BLOCK_TIME  ( 5000 )
#define	ipconfigSOCK_DEFAULT_SEND_BLOCK_TIME	   ( 5000 )
#define ipconfigSOCKET_HAS_USER_SEMAPHORE        1
#define ipconfigSUPPORT_SIGNALS                  1
#define ipconfigSUPPORT_SELECT_FUNCTION				   1  //使能FreeRTOS_select函数

//影响ARP行为的常数
#define ipconfigARP_CACHE_ENTRIES		         16   //ARP表中可以存在的最大条目数
#define ipconfigMAX_ARP_RETRANSMISSIONS      8    //ARP最大请求次数
#define ipconfigMAX_ARP_AGE			             180  //ARP最长保存时间（单位10s）
//#define ipconfigARP_REVERSED_LOOKUP
//#define ipconfigARP_STORES_REMOTE_ADDRESSES
//#define ipconfigUSE_ARP_REVERSED_LOOKUP
//#define ipconfigUSE_ARP_REMOVE_ENTRY

//影响DHCP和名称服务行为的常量
#define ipconfigUSE_DNS                     0   //不启用DNS
#define ipconfigDNS_REQUEST_ATTEMPTS		    20  //DNS请求次数
#define ipconfigUSE_DNS_CACHE				        1   //启动DNS缓存
#define ipconfigDNS_CACHE_ENTRIES			      4   //DNS缓存条目
#define ipconfigDNS_CACHE_NAME_LENGTH		    23  //DNS长度
#define ipconfigUSE_LLMNR                   0
#define ipconfigUSE_NBNS                    0
#define ipconfigUSE_DHCP                    1  /* 见下者 */
extern  int hange_vUSE_DHCP;  /* 见hange_vUSE_DHCP定义 */
#define ipconfigMAXIMUM_DISCOVER_TX_PERIOD  ( 120000 / portTICK_PERIOD_MS )  //DHCP间隔
#define ipconfigUSE_DHCP_HOOK               0   //DHCP钩子函数
#define ipconfigDHCP_REGISTER_HOSTNAME      0   //DHCP服务器名称

//影响IP和ICMP行为的常数
#define ipconfigCAN_FRAGMENT_OUTGOING_PACKETS  1  
#define ipconfigREPLY_TO_INCOMING_PINGS				 1  //ping应答
#define ipconfigSUPPORT_OUTGOING_PINGS				 0  //不启用FreeRTOS_SendPingRequest函数

//提供目标支持的常量
#define ipconfigRAND32()  getRNG_value()  //随机数，在public.c里面

#define ipconfigHAS_INLINE_FUNCTIONS  1  //定义了内联函数设为1（下者）

#ifndef portINLIN  //在portmacro.h里面已经定义了
  #define portINLINE __inline  //内联函数
#endif

//其它
#define ipconfigIS_VALID_PROG_ADDRESS(x) ( (x) != NULL )

//以太网接收任务
#define ethCardRxPrio  configMAX_PRIORITIES - 1
#define ethCardRxStackSize  384
extern TaskHandle_t  ethCardRxHandler;
extern void ethCardRxTask(void *pvParameters);

#endif /* FREERTOS_IP_CONFIG_H */



















