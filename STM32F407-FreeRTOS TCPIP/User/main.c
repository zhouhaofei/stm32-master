#include "stm32f4xx.h"
#include "public.h"
#include "mQueue.h"
#include "FreeRTOS.h"
#include "task.h"
#include "FreeRTOS_IP.h"
#include "LAN8720.h"

/* 见五种内存分配说明.txt */
extern const HeapRegion_t xHeapRegions[];  //在heap_5.c

extern ETH_DMADESCTypeDef DMARxDscrTab[ETH_RXBUFNB];	//stm32f4x7_eth.c
extern ETH_DMADESCTypeDef DMATxDscrTab[ETH_TXBUFNB];
extern uint8_t Rx_Buff[ETH_RX_BUF_SIZE * ETH_RXBUFNB];
extern uint8_t Tx_Buff[ETH_TX_BUF_SIZE * ETH_TXBUFNB];

extern NetworkParm networkParm;  //LAN8720.c

int main(void)
{
	delay_ms(100);
	
	//初始化FreeRTOS内存堆，因为用的heap_5！！！
	vPortDefineHeapRegions((const HeapRegion_t *)xHeapRegions);
	
	mNVIC_config();
	mIWDG_config(6, 625);  //4s看门狗
	mUSART1_config(921600);  //打印使用
	mPrintf("start...\r\n");

	mRNG_Init();  //初始化随机数
	
	LAN8720_Init();  //初始化LAN8720
	ETH_MACDMA_Config();  //配置STM32以太网控制器
	ETH_MACAddressConfig(ETH_MAC_Address0, networkParm.mac);  //向STM32F4的MAC地址寄存器中写入MAC地址
	ETH_DMATxDescChainInit(DMATxDscrTab, Tx_Buff, ETH_TXBUFNB);
	ETH_DMARxDescChainInit(DMARxDscrTab, Rx_Buff, ETH_RXBUFNB);
  for(u8 i = 0; i < ETH_TXBUFNB; i++)	 //使能ipv4发送帧校验和生成
	{
    /* FreeRTOSTCPIP只支持硬件生成ipv4校验和，不支持TCP/UDP/ICMP校验和 */
		ETH_DMATxDescChecksumInsertionConfig(&DMATxDscrTab[i], ETH_DMATxDesc_ChecksumIPV4Header);
	}
	ETH_Start();  //开启MAC和DMA
	
	hange_vUSE_DHCP = networkParm.dhcp;  /* 见hange_vUSE_DHCP定义 */
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

	vTaskStartScheduler();  //启动任务调度

  while(1)  //真执行这里也就崩了
	{
		taskENTER_CRITICAL();  //进临界状态，关闭所有中断
	  mPrintf("OS没启动或崩了\r\n");
    STM32_SYSTEM_RESET;
	}
//	return 0;
}



















