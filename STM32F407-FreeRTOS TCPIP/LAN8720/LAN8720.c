#include "LAN8720.h"

//NetworkParm networkParm;
const NetworkParm defNetworkParm = {  //mac后四字节从STM32 ID获取
  .dhcp = 0,
  .mac  = {0x02, 0x00, 0x00, 0x00, 0x04, 0x08},
  .ip   = {192, 168, 1, 37},  //192.168.1.37
	.sub  = {255, 255, 255, 0},
	.gw   = {192, 168, 1, 1},
	.domainS1 = {192, 168, 1, 1},
	.domainS2 = {8, 8, 8, 8},
  .mtu  = 1500 };

NetworkParm networkParm = {  //测试用
  .dhcp = 0,
  .mac  = {0x02, 0x00, 0x00, 0x00, 0x04, 0x08},
  .ip   = {192, 168, 1, 37},  //192.168.1.37
	.sub  = {255, 255, 255, 0},
	.gw   = {192, 168, 1, 1},
	.domainS1 = {192, 168, 1, 1},
	.domainS2 = {8, 8, 8, 8},
  .mtu  = 1500 };

//在stm32f4x7_eth.c
extern ETH_DMADESCTypeDef     *DMATxDescToSet;      //DMA发送描述符追踪指针
extern ETH_DMADESCTypeDef     *DMARxDescToGet;      //DMA接收描述符追踪指针 
extern ETH_DMA_Rx_Frame_infos *DMA_RX_FRAME_infos;  //DMA最后接收到的帧信息指针

//以太网中断分组配置
static void ETHERNET_NVICConfiguration(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	
	NVIC_InitStructure.NVIC_IRQChannel = ETH_IRQn;  //以太网中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

//初始化LAN8720
void LAN8720_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
  
	//使能GPIO时钟 RMII接口
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOG | RCC_AHB1Periph_GPIOD, ENABLE);  //144Pin
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOD, ENABLE);  //100Pin
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);  //使能SYSCFG时钟
  
	SYSCFG_ETH_MediaInterfaceConfig(SYSCFG_ETH_MediaInterface_RMII);  //MAC和PHY之间使用RMII接口
   
	/* 网络引脚设置 RMII接口
	   ETH_MDIO -------------------------> PA2
	   ETH_MDC --------------------------> PC1
	   ETH_RMII_REF_CLK------------------> PA1
	   ETH_RMII_CRS_DV ------------------> PA7
	   ETH_RMII_RXD0 --------------------> PC4
	   ETH_RMII_RXD1 --------------------> PC5
	   ETH_RMII_TX_EN -------------------> PG11---PB11
	   ETH_RMII_TXD0 --------------------> PG13---PB12
	   ETH_RMII_TXD1 --------------------> PG14---PB13
	   ETH_RESET-------------------------> PD3  */
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;  
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_ETH);  //引脚复用到网络接口上
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_ETH);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_ETH);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource1, GPIO_AF_ETH);  //引脚复用到网络接口上
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource4, GPIO_AF_ETH);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource5, GPIO_AF_ETH);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_13 | GPIO_Pin_14;  //144Pin
	GPIO_Init(GPIOG, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOG, GPIO_PinSource11, GPIO_AF_ETH);
	GPIO_PinAFConfig(GPIOG, GPIO_PinSource13, GPIO_AF_ETH);
	GPIO_PinAFConfig(GPIOG, GPIO_PinSource14, GPIO_AF_ETH);
	
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13;  //100Pin
//	GPIO_Init(GPIOB, &GPIO_InitStructure);
//	GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_ETH);  //引脚复用到网络接口上
//	GPIO_PinAFConfig(GPIOB, GPIO_PinSource12, GPIO_AF_ETH);
//	GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_ETH);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	//推完输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;  
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	GPIO_ResetBits(GPIOD, GPIO_Pin_3);  //硬件复位LAN8720
	delay_ms(50);	
	GPIO_SetBits(GPIOD, GPIO_Pin_3); 	//复位结束
	
	/* 接收中断 */
  ETHERNET_NVICConfiguration();	 //设置中断优先级
}

//001:10M半双工，101:10M全双工，010:100M半双工，110:100M全双工，其他:错误.
u8 LAN8720_Get_Speed(void)
{
	u8 speed = ((ETH_ReadPHYRegister(0x00,31) & 0x1C) >> 2);  //从LAN8720的31号寄存器中读取网络速度和双工模式
	return speed;
}

//初始化ETH MAC层及DMA配置
u8 ETH_MACDMA_Config(void)
{
	u8 rval;
	ETH_InitTypeDef ETH_InitStructure; 
	
	//使能以太网MAC以及MAC接收和发送时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_ETH_MAC | RCC_AHB1Periph_ETH_MAC_Tx | RCC_AHB1Periph_ETH_MAC_Rx, ENABLE);
                        
	ETH_DeInit();  //AHB总线重启以太网
	ETH_SoftwareReset();  //软件重启网络
	while(ETH_GetSoftwareResetStatus() == SET);  //等待软件重启网络完成 
	ETH_StructInit(&ETH_InitStructure);  //初始化网络为默认值  

	//网络MAC参数设置 
	ETH_InitStructure.ETH_AutoNegotiation = ETH_AutoNegotiation_Enable;  //开启网络自适应功能
	ETH_InitStructure.ETH_LoopbackMode = ETH_LoopbackMode_Disable;  //关闭反馈
	ETH_InitStructure.ETH_RetryTransmission = ETH_RetryTransmission_Disable;  //关闭重传功能
	ETH_InitStructure.ETH_AutomaticPadCRCStrip = ETH_AutomaticPadCRCStrip_Disable;  //关闭自动去除PDA/CRC功能 
	ETH_InitStructure.ETH_ReceiveAll = ETH_ReceiveAll_Disable;  //不接收所有类型的帧
	ETH_InitStructure.ETH_BroadcastFramesReception = ETH_BroadcastFramesReception_Enable;  //允许接收所有广播帧
	ETH_InitStructure.ETH_PromiscuousMode = ETH_PromiscuousMode_Disable;  //关闭混合模式的地址过滤  
	ETH_InitStructure.ETH_MulticastFramesFilter = ETH_MulticastFramesFilter_Perfect;  //对于组播地址使用完美地址过滤   
	ETH_InitStructure.ETH_UnicastFramesFilter = ETH_UnicastFramesFilter_Perfect;  //对单播地址使用完美地址过滤 

  /* FreeRTOSTCPIP只支持硬件生成ipv4校验和，不支持TCP/UDP/ICMP校验和 */
	ETH_InitStructure.ETH_ChecksumOffload = ETH_ChecksumOffload_Disable;  //不开启ipv4的TCP/UDP/ICMP的帧校验和卸载
	//当我们使用帧校验和卸载功能的时候，一定要使能存储转发模式,存储转发模式中要保证整个帧存储在FIFO中,
	//这样MAC能插入/识别出帧校验值,当真校验正确的时候DMA就可以处理帧,否则就丢弃掉该帧
	ETH_InitStructure.ETH_DropTCPIPChecksumErrorFrame = ETH_DropTCPIPChecksumErrorFrame_Enable;  //开启丢弃TCP/IP错误帧
	ETH_InitStructure.ETH_ReceiveStoreForward = ETH_ReceiveStoreForward_Enable;  //开启接收数据的存储转发模式    
	ETH_InitStructure.ETH_TransmitStoreForward = ETH_TransmitStoreForward_Enable;  //开启发送数据的存储转发模式  

	ETH_InitStructure.ETH_ForwardErrorFrames = ETH_ForwardErrorFrames_Disable;  //禁止转发错误帧  
	ETH_InitStructure.ETH_ForwardUndersizedGoodFrames = ETH_ForwardUndersizedGoodFrames_Disable;  //不转发过小的好帧 
	ETH_InitStructure.ETH_SecondFrameOperate = ETH_SecondFrameOperate_Enable;  //打开处理第二帧功能
	ETH_InitStructure.ETH_AddressAlignedBeats = ETH_AddressAlignedBeats_Enable;  //开启DMA传输的地址对齐功能
	ETH_InitStructure.ETH_FixedBurst = ETH_FixedBurst_Enable;  //开启固定突发功能    
	ETH_InitStructure.ETH_RxDMABurstLength = ETH_RxDMABurstLength_32Beat;  //DMA发送的最大突发长度为32个节拍   
	ETH_InitStructure.ETH_TxDMABurstLength = ETH_TxDMABurstLength_32Beat;  //DMA接收的最大突发长度为32个节拍
	ETH_InitStructure.ETH_DMAArbitration = ETH_DMAArbitration_RoundRobin_RxTx_2_1;
	rval = ETH_Init(&ETH_InitStructure, LAN8720_PHY_ADDRESS);  //配置ETH

  /* 接收中断 */
	if(rval == ETH_SUCCESS)  //配置成功
	{
		ETH_DMAITConfig(ETH_DMA_IT_NIS | ETH_DMA_IT_R, ENABLE);  //使能以太网接收中断
	}

	return rval;
}

//放到FreeRTOSIPHook.c里面了
//void ETH_IRQHandler(void)  //以太网DMA接收中断服务函数
//{
//	while(ETH_GetRxPktSize(DMARxDescToGet) != 0) 	//检测是否收到数据包
//	{
//		
//	}
//	//发通知给任务
//	ETH_DMAClearITPendingBit(ETH_DMA_IT_R);  //清除DMA中断标志位
//	ETH_DMAClearITPendingBit(ETH_DMA_IT_NIS);  //清除DMA接收中断标志位
//}

//接收以太网数据包
FrameTypeDef ETH_Rx_Packet(void)
{
	FrameTypeDef frame = {0, 0, NULL};   
	//检查当前描述符,是否属于ETHERNET DMA(设置的时候)/CPU(复位的时候)
	if((DMARxDescToGet->Status & ETH_DMARxDesc_OWN) != (u32)RESET)
	{
		frame.length = ETH_ERROR;  /* ETH_ERROR值为0 */
		if((ETH->DMASR & ETH_DMASR_RBUS) != (u32)RESET)  
		{
			ETH->DMASR = ETH_DMASR_RBUS;  //清除ETH DMA的RBUS位 
			ETH->DMARPDR = 0;  //恢复DMA接收
		}
		return frame;  //错误，OWN位被设置了
	}
	if(((DMARxDescToGet->Status & ETH_DMARxDesc_ES) == (u32)RESET) && \
	   ((DMARxDescToGet->Status & ETH_DMARxDesc_LS) != (u32)RESET) && \
	   ((DMARxDescToGet->Status & ETH_DMARxDesc_FS) != (u32)RESET))  
	{
		frame.length = ((DMARxDescToGet->Status & ETH_DMARxDesc_FL) >> ETH_DMARxDesc_FrameLengthShift) - 4;  //得到接收包帧长度(不包含4字节CRC)
 		frame.buffer = DMARxDescToGet->Buffer1Addr;  //得到包数据所在的位置
	}
	else
	{
	  frame.length = ETH_ERROR;  //错误
	}
	frame.descriptor = DMARxDescToGet;  
	//更新ETH DMA全局Rx描述符为下一个Rx描述符
	//为下一次buffer读取设置下一个DMA Rx描述符
	DMARxDescToGet = (ETH_DMADESCTypeDef*)(DMARxDescToGet->Buffer2NextDescAddr);   
	return frame;  
}

//发送以太网数据包
u8 ETH_Tx_Packet(u16 FrameLength)
{
	//检查当前描述符,是否属于ETHERNET DMA(设置的时候)/CPU(复位的时候)
	if((DMATxDescToSet->Status & ETH_DMATxDesc_OWN) != (u32)RESET)
	{
	  return ETH_ERROR;  //错误，OWN位被设置了
	}
 	DMATxDescToSet->ControlBufferSize = (FrameLength & ETH_DMATxDesc_TBS1);  //设置帧长度,bits[12:0]
	DMATxDescToSet->Status |= ETH_DMATxDesc_LS | ETH_DMATxDesc_FS;  //设置最后一个和第一个位段置位(1个描述符传输一帧)
  DMATxDescToSet->Status |= ETH_DMATxDesc_OWN;  //设置Tx描述符的OWN位,buffer重归ETH DMA
	if((ETH->DMASR & ETH_DMASR_TBUS) != (u32)RESET)  //当Tx Buffer不可用位(TBUS)被设置的时候,重置它恢复传输
	{
		ETH->DMASR = ETH_DMASR_TBUS;  //重置ETH DMA TBUS位 
		ETH->DMATPDR = 0;  //恢复DMA发送
	}
	//更新ETH DMA全局Tx描述符为下一个Tx描述符
	//为下一次buffer发送设置下一个DMA Tx描述符 
	DMATxDescToSet = (ETH_DMADESCTypeDef*)(DMATxDescToSet->Buffer2NextDescAddr);    
	return ETH_SUCCESS;   
}

//得到当前描述符的Tx buffer地址
u32 ETH_GetCurrentTxBuffer(void)
{
  return DMATxDescToSet->Buffer1Addr;  //返回Tx buffer地址  
}



















