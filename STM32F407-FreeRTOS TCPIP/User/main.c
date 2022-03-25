#include "stm32f4xx.h"
#include "public.h"
#include "mQueue.h"
#include "FreeRTOS.h"
#include "task.h"
#include "FreeRTOS_IP.h"
#include "LAN8720.h"

/* �������ڴ����˵��.txt */
extern const HeapRegion_t xHeapRegions[];  //��heap_5.c

extern ETH_DMADESCTypeDef DMARxDscrTab[ETH_RXBUFNB];	//stm32f4x7_eth.c
extern ETH_DMADESCTypeDef DMATxDscrTab[ETH_TXBUFNB];
extern uint8_t Rx_Buff[ETH_RX_BUF_SIZE * ETH_RXBUFNB];
extern uint8_t Tx_Buff[ETH_TX_BUF_SIZE * ETH_TXBUFNB];

extern NetworkParm networkParm;  //LAN8720.c

int main(void)
{
	delay_ms(100);
	
	//��ʼ��FreeRTOS�ڴ�ѣ���Ϊ�õ�heap_5������
	vPortDefineHeapRegions((const HeapRegion_t *)xHeapRegions);
	
	mNVIC_config();
	mIWDG_config(6, 625);  //4s���Ź�
	mUSART1_config(921600);  //��ӡʹ��
	mPrintf("start...\r\n");

	mRNG_Init();  //��ʼ�������
	
	LAN8720_Init();  //��ʼ��LAN8720
	ETH_MACDMA_Config();  //����STM32��̫��������
	ETH_MACAddressConfig(ETH_MAC_Address0, networkParm.mac);  //��STM32F4��MAC��ַ�Ĵ�����д��MAC��ַ
	ETH_DMATxDescChainInit(DMATxDscrTab, Tx_Buff, ETH_TXBUFNB);
	ETH_DMARxDescChainInit(DMARxDscrTab, Rx_Buff, ETH_RXBUFNB);
  for(u8 i = 0; i < ETH_TXBUFNB; i++)	 //ʹ��ipv4����֡У�������
	{
    /* FreeRTOSTCPIPֻ֧��Ӳ������ipv4У��ͣ���֧��TCP/UDP/ICMPУ��� */
		ETH_DMATxDescChecksumInsertionConfig(&DMATxDscrTab[i], ETH_DMATxDesc_ChecksumIPV4Header);
	}
	ETH_Start();  //����MAC��DMA
	
	hange_vUSE_DHCP = networkParm.dhcp;  /* ��hange_vUSE_DHCP���� */
	FreeRTOS_IPInit(networkParm.ip, 
									networkParm.sub, 
									networkParm.gw,
	                networkParm.domainS1,
									networkParm.mac);
	
	xTaskCreate(ethCardRxTask, 
	            "ethCardRxTask",
	            ethCardRxStackSize,
	            (void* )NULL,
	            ethCardRxPrio,
	            &ethCardRxHandler);

	vTaskStartScheduler();  //�����������

  while(1)  //��ִ������Ҳ�ͱ���
	{
		taskENTER_CRITICAL();  //���ٽ�״̬���ر������ж�
	  mPrintf("OSû���������\r\n");
    STM32_SYSTEM_RESET;
	}
//	return 0;
}



















