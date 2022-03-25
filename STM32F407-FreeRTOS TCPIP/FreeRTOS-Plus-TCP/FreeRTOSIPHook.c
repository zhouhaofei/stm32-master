/********************************************************
Created by hange_v;
FreeRTOS-TCPIP中一些需要用户实现的函数可以放在这里面；
********************************************************/

#include "stm32f4xx.h"
#include "public.h"
#include "FreeRTOS.h"
#include "FreeRTOS_IP.h"
#include "task.h"
#include "FreeRTOS_IP_Private.h"
#include "LAN8720.h"

/* FreeRTOSTCPIP的DHCP是通过宏开启的，不能在运行时判断，
   我把宏控制改成了变量hange_vUSE_DHCP控制，
	 同时把ipconfigUSE_DHCP设为1即可，
   所以协议栈在运行之前需要先给hange_vUSE_DHCP赋值 */
int hange_vUSE_DHCP = 0;

//在stm32f4x7_eth.c
extern ETH_DMADESCTypeDef     *DMATxDescToSet;      //DMA发送描述符追踪指针
extern ETH_DMADESCTypeDef     *DMARxDescToGet;      //DMA接收描述符追踪指针 
extern ETH_DMA_Rx_Frame_infos *DMA_RX_FRAME_infos;  //DMA最后接收到的帧信息指针

//vLoggingPrintf在FreeRTOSIPConfig.h中被定义为printf
//#define vLoggingPrintf  printf  //定义为printf

/* Generate a randomized TCP Initial Sequence Number per RFC. */
uint32_t ulApplicationGetNextSequenceNumber( uint32_t ulSourceAddress,
													uint16_t usSourcePort,
													uint32_t ulDestinationAddress,
													uint16_t usDestinationPort )
{
	return getRNG_value();
}

//网络准备就绪时 钩子函数
void vApplicationIPNetworkEventHook(eIPCallbackEvent_t eNetworkEvent)
{
  mTcpIpDebug("vApplicationIPNetworkEventHook\r\n");
}

/**********************************************************************/
/*下面几个函数应该放在NetworkInterface.c，为了方便也都放在这个.c文件了*/
/**********************************************************************/

//BufferAllocation_2.c
extern void vReleaseNetworkBufferAndDescriptor( NetworkBufferDescriptor_t * const pxNetworkBuffer );
extern NetworkBufferDescriptor_t *pxGetNetworkBufferWithDescriptor( size_t xRequestedSizeBytes, 
	                                                                  TickType_t xBlockTimeTicks );

//初始化网卡 钩子函数
BaseType_t xNetworkInterfaceInitialise(void)
{
	return pdPASS;
}

//调用网卡发送函数  非零复制方案
BaseType_t xNetworkInterfaceOutput(NetworkBufferDescriptor_t * const pxDescriptor,
                                   BaseType_t xReleaseAfterSend)
{
	u8 *buffer = (u8 *)ETH_GetCurrentTxBuffer();
	memcpy(buffer, pxDescriptor->pucEthernetBuffer, pxDescriptor->xDataLength);
	if(ETH_Tx_Packet(pxDescriptor->xDataLength) == ETH_ERROR)
  {
    mPrintf("xNetworkInterfaceOutput()\r\n");
  }
	if(xReleaseAfterSend != pdFALSE)  //需要手动释放
	{
	  vReleaseNetworkBufferAndDescriptor(pxDescriptor); 
	}
	return pdTRUE;
}

/* 被以太网接收调用  非零复制方案 */
void ETH_IRQHandlerHook(void)
{
	FrameTypeDef frame = ETH_Rx_Packet();
	if(frame.length <= 0)  //没有数据包就直接return了
	{
	  return;
	}
	
  NetworkBufferDescriptor_t* pxBufferDescriptor = pxGetNetworkBufferWithDescriptor(frame.length, 0);
	
	if(pxBufferDescriptor != NULL)
	{
		memcpy(pxBufferDescriptor->pucEthernetBuffer, (u8 *)frame.buffer, frame.length);
		if(eConsiderFrameForProcessing(pxBufferDescriptor->pucEthernetBuffer) == eProcessBuffer)
		{
			IPStackEvent_t xRxEvent;
			xRxEvent.eEventType = eNetworkRxEvent;
			xRxEvent.pvData = (void*)pxBufferDescriptor;
			if(xSendEventStructToIPTask(&xRxEvent, 0) == pdFALSE)  //将数据发送到TCP/IP堆栈
			{
				vReleaseNetworkBufferAndDescriptor(pxBufferDescriptor);  //无法发送到堆栈，释放缓冲区
			}
			else
			{
		    mTcpIpDebug("发送网卡数据到TCP/IP堆栈成功\r\n");
			}
		}
		else
		{
		  mTcpIpDebug("不处理的以太网数据\r\n");
			vReleaseNetworkBufferAndDescriptor(pxBufferDescriptor);  //无法发送到堆栈，释放缓冲区
		}
	}
	else
	{
		mPrintf("网卡接收分配缓冲区失败\r\n");
	}
	frame.descriptor->Status = ETH_DMARxDesc_OWN;  //设置Rx描述符OWN位，buffer重归ETHDMA
	//当RxBuffer不可用位(RBUS)被设置的时候，重置它，恢复传输
	if((ETH->DMASR & ETH_DMASR_RBUS) != (u32)RESET)
	{
		ETH->DMASR = ETH_DMASR_RBUS;  //重置ETH_DMARBUS位
		ETH->DMARPDR = 0;  //恢复DMA接收
	}
}

//如果使用BufferAllocation_1则必须实现
void vNetworkInterfaceAllocateRAMToBuffers( NetworkBufferDescriptor_t 
	     pxNetworkBuffers[ ipconfigNUM_NETWORK_BUFFER_DESCRIPTORS ] )
{
  
}

//这个函数不是必须实现的，可以作为phy link丢失重启
BaseType_t xGetPhyLinkStatus( void )
{
	return pdTRUE;
}

/*********************************************************************/
/**************************以太网接收任务和中断***********************/
/*********************************************************************/
TaskHandle_t  ethCardRxHandler;
void ethCardRxTask(void *pvParameters)
{
	u32 notifyVal = 0;
  while(1)
	{
		notifyVal = ulTaskNotifyTake(pdFALSE, portMAX_DELAY);
		while(ETH_GetRxPktSize(DMARxDescToGet) != 0) 	//检测是否收到数据包
		{
			ETH_IRQHandlerHook();
		}
	}
}

void ETH_IRQHandler(void)  //以太网DMA接收中断服务函数
{
	ETH_DMAClearITPendingBit(ETH_DMA_IT_R);  //清除DMA中断标志位
	ETH_DMAClearITPendingBit(ETH_DMA_IT_NIS);  //清除DMA接收中断标志位
	
	/* 向ethCardRxTask()任务发送通知 */
	BaseType_t  pxHigherPriorityTaskWoken;
	vTaskNotifyGiveFromISR(ethCardRxHandler, &pxHigherPriorityTaskWoken);
	portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
}



















