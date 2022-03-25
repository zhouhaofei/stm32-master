/********************************************************
Created by hange_v;
FreeRTOS-TCPIP��һЩ��Ҫ�û�ʵ�ֵĺ������Է��������棻
********************************************************/

#include "stm32f4xx.h"
#include "public.h"
#include "FreeRTOS.h"
#include "FreeRTOS_IP.h"
#include "task.h"
#include "FreeRTOS_IP_Private.h"
#include "LAN8720.h"

/* FreeRTOSTCPIP��DHCP��ͨ���꿪���ģ�����������ʱ�жϣ�
   �ҰѺ���Ƹĳ��˱���hange_vUSE_DHCP���ƣ�
	 ͬʱ��ipconfigUSE_DHCP��Ϊ1���ɣ�
   ����Э��ջ������֮ǰ��Ҫ�ȸ�hange_vUSE_DHCP��ֵ */
int hange_vUSE_DHCP = 0;

//��stm32f4x7_eth.c
extern ETH_DMADESCTypeDef     *DMATxDescToSet;      //DMA����������׷��ָ��
extern ETH_DMADESCTypeDef     *DMARxDescToGet;      //DMA����������׷��ָ�� 
extern ETH_DMA_Rx_Frame_infos *DMA_RX_FRAME_infos;  //DMA�����յ���֡��Ϣָ��

//vLoggingPrintf��FreeRTOSIPConfig.h�б�����Ϊprintf
//#define vLoggingPrintf  printf  //����Ϊprintf

/* Generate a randomized TCP Initial Sequence Number per RFC. */
uint32_t ulApplicationGetNextSequenceNumber( uint32_t ulSourceAddress,
													uint16_t usSourcePort,
													uint32_t ulDestinationAddress,
													uint16_t usDestinationPort )
{
	return getRNG_value();
}

//����׼������ʱ ���Ӻ���
void vApplicationIPNetworkEventHook(eIPCallbackEvent_t eNetworkEvent)
{
  mTcpIpDebug("vApplicationIPNetworkEventHook\r\n");
}

/**********************************************************************/
/*���漸������Ӧ�÷���NetworkInterface.c��Ϊ�˷���Ҳ���������.c�ļ���*/
/**********************************************************************/

//BufferAllocation_2.c
extern void vReleaseNetworkBufferAndDescriptor( NetworkBufferDescriptor_t * const pxNetworkBuffer );
extern NetworkBufferDescriptor_t *pxGetNetworkBufferWithDescriptor( size_t xRequestedSizeBytes, 
	                                                                  TickType_t xBlockTimeTicks );

//��ʼ������ ���Ӻ���
BaseType_t xNetworkInterfaceInitialise(void)
{
	return pdPASS;
}

//�����������ͺ���  ���㸴�Ʒ���
BaseType_t xNetworkInterfaceOutput(NetworkBufferDescriptor_t * const pxDescriptor,
                                   BaseType_t xReleaseAfterSend)
{
	u8 *buffer = (u8 *)ETH_GetCurrentTxBuffer();
	memcpy(buffer, pxDescriptor->pucEthernetBuffer, pxDescriptor->xDataLength);
	if(ETH_Tx_Packet(pxDescriptor->xDataLength) == ETH_ERROR)
  {
    mPrintf("xNetworkInterfaceOutput()\r\n");
  }
	if(xReleaseAfterSend != pdFALSE)  //��Ҫ�ֶ��ͷ�
	{
	  vReleaseNetworkBufferAndDescriptor(pxDescriptor); 
	}
	return pdTRUE;
}

/* ����̫�����յ���  ���㸴�Ʒ��� */
void ETH_IRQHandlerHook(void)
{
	FrameTypeDef frame = ETH_Rx_Packet();
	if(frame.length <= 0)  //û�����ݰ���ֱ��return��
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
			if(xSendEventStructToIPTask(&xRxEvent, 0) == pdFALSE)  //�����ݷ��͵�TCP/IP��ջ
			{
				vReleaseNetworkBufferAndDescriptor(pxBufferDescriptor);  //�޷����͵���ջ���ͷŻ�����
			}
			else
			{
		    mTcpIpDebug("�����������ݵ�TCP/IP��ջ�ɹ�\r\n");
			}
		}
		else
		{
		  mTcpIpDebug("���������̫������\r\n");
			vReleaseNetworkBufferAndDescriptor(pxBufferDescriptor);  //�޷����͵���ջ���ͷŻ�����
		}
	}
	else
	{
		mPrintf("�������շ��仺����ʧ��\r\n");
	}
	frame.descriptor->Status = ETH_DMARxDesc_OWN;  //����Rx������OWNλ��buffer�ع�ETHDMA
	//��RxBuffer������λ(RBUS)�����õ�ʱ�����������ָ�����
	if((ETH->DMASR & ETH_DMASR_RBUS) != (u32)RESET)
	{
		ETH->DMASR = ETH_DMASR_RBUS;  //����ETH_DMARBUSλ
		ETH->DMARPDR = 0;  //�ָ�DMA����
	}
}

//���ʹ��BufferAllocation_1�����ʵ��
void vNetworkInterfaceAllocateRAMToBuffers( NetworkBufferDescriptor_t 
	     pxNetworkBuffers[ ipconfigNUM_NETWORK_BUFFER_DESCRIPTORS ] )
{
  
}

//����������Ǳ���ʵ�ֵģ�������Ϊphy link��ʧ����
BaseType_t xGetPhyLinkStatus( void )
{
	return pdTRUE;
}

/*********************************************************************/
/**************************��̫������������ж�***********************/
/*********************************************************************/
TaskHandle_t  ethCardRxHandler;
void ethCardRxTask(void *pvParameters)
{
	u32 notifyVal = 0;
  while(1)
	{
		notifyVal = ulTaskNotifyTake(pdFALSE, portMAX_DELAY);
		while(ETH_GetRxPktSize(DMARxDescToGet) != 0) 	//����Ƿ��յ����ݰ�
		{
			ETH_IRQHandlerHook();
		}
	}
}

void ETH_IRQHandler(void)  //��̫��DMA�����жϷ�����
{
	ETH_DMAClearITPendingBit(ETH_DMA_IT_R);  //���DMA�жϱ�־λ
	ETH_DMAClearITPendingBit(ETH_DMA_IT_NIS);  //���DMA�����жϱ�־λ
	
	/* ��ethCardRxTask()������֪ͨ */
	BaseType_t  pxHigherPriorityTaskWoken;
	vTaskNotifyGiveFromISR(ethCardRxHandler, &pxHigherPriorityTaskWoken);
	portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
}



















