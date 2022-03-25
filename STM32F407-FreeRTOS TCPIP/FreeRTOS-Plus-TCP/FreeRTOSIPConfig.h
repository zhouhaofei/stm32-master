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

//Ӱ��TCP/IP��ջ����ִ����Ϊ�ĳ���
#define ipconfigIP_TASK_PRIORITY          ( configMAX_PRIORITIES - 1 )  //TCP/IP�������ȼ�
#define ipconfigIP_TASK_STACK_SIZE_WORDS	( configMINIMAL_STACK_SIZE * 5 )  //TCP/IP����Ķ�ջ��С
#define ipconfigUSE_NETWORK_EVENT_HOOK    1  //���� �����¼� ���Ӻ���
#define ipconfigEVENT_QUEUE_LENGTH		    ( ipconfigNUM_NETWORK_BUFFER_DESCRIPTORS + 5 )  //�¼����б������ٱ����绺������������5
                                          //���¼���Ӧ�ó��������͵�IP��ջ������һ���Ŷӵȴ����������¼���

//���ԣ����ٺͼ�¼����
#define vLoggingPrintf  printf  //����Ϊprintf

#define ipconfigHAS_DEBUG_PRINTF	  0
#if( ipconfigHAS_DEBUG_PRINTF == 1 )
	#define FreeRTOS_debug_printf(X)	vLoggingPrintf X
#endif

#define ipconfigHAS_PRINTF			    0
#if( ipconfigHAS_PRINTF == 1 )
	#define FreeRTOS_printf(X)			  vLoggingPrintf X
#endif

#define ipconfigTCP_MAY_LOG_PORT(xPort)  (xPort != 80)  //��������Ϊ80�˿�������־
#define ipconfigCHECK_IP_QUEUE_SPACE     1  //Ӧ�ó��������͵�IP��ջ��һ���Ŷӵȴ����������¼���
#define ipconfigWATCHDOG_TIMER()            //IP����ÿ�ε������õĺ꣬������ι��

//Ӳ�������������ض�����
#define ipconfigBYTE_ORDER    pdFREERTOS_LITTLE_ENDIAN   //������С��
/* FreeRTOSTCPIPֻ֧��Ӳ������IPУ��ͣ���֧��TCP/UDP/ICMPУ��� */
#define ipconfigDRIVER_INCLUDED_TX_IP_CHECKSUM       1   //Ӳ������ipv4У��
#define ipconfigDRIVER_INCLUDED_RX_IP_CHECKSUM       1
#define ipconfigETHERNET_DRIVER_FILTERS_FRAME_TYPES	 1   //Ӳ������MAC
//#define ipconfigETHERNET_DRIVER_FILTERS_PACKETS
#define ipconfigNETWORK_MTU                          1500
#define ipconfigNUM_NETWORK_BUFFER_DESCRIPTORS		   10  //������TCP/IP��ջ�����绺����������
#define ipconfigFILTER_OUT_NON_ETHERNET_II_FRAMES    1   //����̫��II��ʽ����̫��֡��������
//#define ipconfigUSE_LINKED_RX_MESSAGES
#define ipconfigZERO_COPY_TX_DRIVER                  0   //���Ͳ�ʹ��0���Ʒ���
#define ipconfigZERO_COPY_RX_DRIVER                  0   //���ղ�ʹ��0���Ʒ���
//#define ipconfigBUFFER_PADDING
//#define ipconfigPACKET_FILLER_SIZE

//TCP�ض�����
#define ipconfigUSE_TCP				            1    //����TCP
#define ipconfigTCP_TIME_TO_LIVE		      128  //TCP TTL
#define ipconfigTCP_TX_BUFFER_LENGTH      (2 * ipconfigTCP_MSS)  //ÿ��TCP�׽��ֶ���һ�����ڽ��յĻ�������һ�����ڴ���ĵ�����������
#define ipconfigTCP_RX_BUFFER_LENGTH      (2 * ipconfigTCP_MSS)
#define ipconfigUSE_TCP_WIN			          0    //��ʹ�û�������(��λΪipconfigTCP_MSS)
#define ipconfigTCP_WIN_SEG_COUNT		      32  //����������������
#define ipconfigUSE_TCP_TIMESTAMPS        0    //��ʹ��TCPʱ���
#define ipconfigTCP_MSS                   400  //����TCP���ݰ���MSSֵ�����ֽ�Ϊ��λ��
#define ipconfigTCP_KEEP_ALIVE				    1    //��������
#define ipconfigTCP_KEEP_ALIVE_INTERVAL		20   //��������ʱ��(��λ�� ��)
#define ipconfigTCP_HANG_PROTECTION			  1    //ָ����ʱ�����׽�����û��״̬���ģ����׽��ֱ��Ϊ�ѹر�
#define ipconfigTCP_HANG_PROTECTION_TIME	30   //���ߵ�ʱ��(��λ�� ��)
#define ipconfigIGNORE_UNKNOWN_PACKETS    0    //����Ϊ0��

//UDP�ض�����
#define ipconfigUDP_TIME_TO_LIVE		           128  //UDP TTL
#define ipconfigUDP_MAX_SEND_BLOCK_TIME_TICKS  ( 5000 / portTICK_PERIOD_MS )  //�����Ϊ���ͳ�ʱʱ��
#define ipconfigUDP_MAX_RX_PACKETS             5    //UDP�׽��ֵ�Rx�����п��Դ��ڵ�������ݰ���

//����Ӱ���׽�����Ϊ�ĳ���
#define ipconfigINCLUDE_FULL_INET_ADDR	         1  //����Ϊ1
#define ipconfigALLOW_SOCKET_SEND_WITHOUT_BIND   1  //�ͻ����Զ��󶨶˿�
#define ipconfigSOCK_DEFAULT_RECEIVE_BLOCK_TIME  ( 5000 )
#define	ipconfigSOCK_DEFAULT_SEND_BLOCK_TIME	   ( 5000 )
#define ipconfigSOCKET_HAS_USER_SEMAPHORE        1
#define ipconfigSUPPORT_SIGNALS                  1
#define ipconfigSUPPORT_SELECT_FUNCTION				   1  //ʹ��FreeRTOS_select����

//Ӱ��ARP��Ϊ�ĳ���
#define ipconfigARP_CACHE_ENTRIES		         16   //ARP���п��Դ��ڵ������Ŀ��
#define ipconfigMAX_ARP_RETRANSMISSIONS      8    //ARP����������
#define ipconfigMAX_ARP_AGE			             180  //ARP�����ʱ�䣨��λ10s��
//#define ipconfigARP_REVERSED_LOOKUP
//#define ipconfigARP_STORES_REMOTE_ADDRESSES
//#define ipconfigUSE_ARP_REVERSED_LOOKUP
//#define ipconfigUSE_ARP_REMOVE_ENTRY

//Ӱ��DHCP�����Ʒ�����Ϊ�ĳ���
#define ipconfigUSE_DNS                     0   //������DNS
#define ipconfigDNS_REQUEST_ATTEMPTS		    20  //DNS�������
#define ipconfigUSE_DNS_CACHE				        1   //����DNS����
#define ipconfigDNS_CACHE_ENTRIES			      4   //DNS������Ŀ
#define ipconfigDNS_CACHE_NAME_LENGTH		    23  //DNS����
#define ipconfigUSE_LLMNR                   0
#define ipconfigUSE_NBNS                    0
#define ipconfigUSE_DHCP                    1  /* ������ */
extern  int hange_vUSE_DHCP;  /* ��hange_vUSE_DHCP���� */
#define ipconfigMAXIMUM_DISCOVER_TX_PERIOD  ( 120000 / portTICK_PERIOD_MS )  //DHCP���
#define ipconfigUSE_DHCP_HOOK               0   //DHCP���Ӻ���
#define ipconfigDHCP_REGISTER_HOSTNAME      0   //DHCP����������

//Ӱ��IP��ICMP��Ϊ�ĳ���
#define ipconfigCAN_FRAGMENT_OUTGOING_PACKETS  1  
#define ipconfigREPLY_TO_INCOMING_PINGS				 1  //pingӦ��
#define ipconfigSUPPORT_OUTGOING_PINGS				 0  //������FreeRTOS_SendPingRequest����

//�ṩĿ��֧�ֵĳ���
#define ipconfigRAND32()  getRNG_value()  //���������public.c����

#define ipconfigHAS_INLINE_FUNCTIONS  1  //����������������Ϊ1�����ߣ�

#ifndef portINLIN  //��portmacro.h�����Ѿ�������
  #define portINLINE __inline  //��������
#endif

//����
#define ipconfigIS_VALID_PROG_ADDRESS(x) ( (x) != NULL )

//��̫����������
#define ethCardRxPrio  configMAX_PRIORITIES - 1
#define ethCardRxStackSize  384
extern TaskHandle_t  ethCardRxHandler;
extern void ethCardRxTask(void *pvParameters);

#endif /* FREERTOS_IP_CONFIG_H */



















