#include "LAN8720.h"

//NetworkParm networkParm;
const NetworkParm defNetworkParm = {  //mac�����ֽڴ�STM32 ID��ȡ
  .dhcp = 0,
  .mac  = {0x02, 0x00, 0x00, 0x00, 0x04, 0x08},
  .ip   = {192, 168, 1, 37},  //192.168.1.37
	.sub  = {255, 255, 255, 0},
	.gw   = {192, 168, 1, 1},
	.domainS1 = {192, 168, 1, 1},
	.domainS2 = {8, 8, 8, 8},
  .mtu  = 1500 };

NetworkParm networkParm = {  //������
  .dhcp = 0,
  .mac  = {0x02, 0x00, 0x00, 0x00, 0x04, 0x08},
  .ip   = {192, 168, 1, 37},  //192.168.1.37
	.sub  = {255, 255, 255, 0},
	.gw   = {192, 168, 1, 1},
	.domainS1 = {192, 168, 1, 1},
	.domainS2 = {8, 8, 8, 8},
  .mtu  = 1500 };

//��stm32f4x7_eth.c
extern ETH_DMADESCTypeDef     *DMATxDescToSet;      //DMA����������׷��ָ��
extern ETH_DMADESCTypeDef     *DMARxDescToGet;      //DMA����������׷��ָ�� 
extern ETH_DMA_Rx_Frame_infos *DMA_RX_FRAME_infos;  //DMA�����յ���֡��Ϣָ��

//��̫���жϷ�������
static void ETHERNET_NVICConfiguration(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	
	NVIC_InitStructure.NVIC_IRQChannel = ETH_IRQn;  //��̫���ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

//��ʼ��LAN8720
void LAN8720_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
  
	//ʹ��GPIOʱ�� RMII�ӿ�
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOG | RCC_AHB1Periph_GPIOD, ENABLE);  //144Pin
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOD, ENABLE);  //100Pin
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);  //ʹ��SYSCFGʱ��
  
	SYSCFG_ETH_MediaInterfaceConfig(SYSCFG_ETH_MediaInterface_RMII);  //MAC��PHY֮��ʹ��RMII�ӿ�
   
	/* ������������ RMII�ӿ�
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
	
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_ETH);  //���Ÿ��õ�����ӿ���
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_ETH);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_ETH);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource1, GPIO_AF_ETH);  //���Ÿ��õ�����ӿ���
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource4, GPIO_AF_ETH);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource5, GPIO_AF_ETH);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_13 | GPIO_Pin_14;  //144Pin
	GPIO_Init(GPIOG, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOG, GPIO_PinSource11, GPIO_AF_ETH);
	GPIO_PinAFConfig(GPIOG, GPIO_PinSource13, GPIO_AF_ETH);
	GPIO_PinAFConfig(GPIOG, GPIO_PinSource14, GPIO_AF_ETH);
	
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13;  //100Pin
//	GPIO_Init(GPIOB, &GPIO_InitStructure);
//	GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_ETH);  //���Ÿ��õ�����ӿ���
//	GPIO_PinAFConfig(GPIOB, GPIO_PinSource12, GPIO_AF_ETH);
//	GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_ETH);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	//�������
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;  
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	GPIO_ResetBits(GPIOD, GPIO_Pin_3);  //Ӳ����λLAN8720
	delay_ms(50);	
	GPIO_SetBits(GPIOD, GPIO_Pin_3); 	//��λ����
	
	/* �����ж� */
  ETHERNET_NVICConfiguration();	 //�����ж����ȼ�
}

//001:10M��˫����101:10Mȫ˫����010:100M��˫����110:100Mȫ˫��������:����.
u8 LAN8720_Get_Speed(void)
{
	u8 speed = ((ETH_ReadPHYRegister(0x00,31) & 0x1C) >> 2);  //��LAN8720��31�żĴ����ж�ȡ�����ٶȺ�˫��ģʽ
	return speed;
}

//��ʼ��ETH MAC�㼰DMA����
u8 ETH_MACDMA_Config(void)
{
	u8 rval;
	ETH_InitTypeDef ETH_InitStructure; 
	
	//ʹ����̫��MAC�Լ�MAC���պͷ���ʱ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_ETH_MAC | RCC_AHB1Periph_ETH_MAC_Tx | RCC_AHB1Periph_ETH_MAC_Rx, ENABLE);
                        
	ETH_DeInit();  //AHB����������̫��
	ETH_SoftwareReset();  //�����������
	while(ETH_GetSoftwareResetStatus() == SET);  //�ȴ��������������� 
	ETH_StructInit(&ETH_InitStructure);  //��ʼ������ΪĬ��ֵ  

	//����MAC�������� 
	ETH_InitStructure.ETH_AutoNegotiation = ETH_AutoNegotiation_Enable;  //������������Ӧ����
	ETH_InitStructure.ETH_LoopbackMode = ETH_LoopbackMode_Disable;  //�رշ���
	ETH_InitStructure.ETH_RetryTransmission = ETH_RetryTransmission_Disable;  //�ر��ش�����
	ETH_InitStructure.ETH_AutomaticPadCRCStrip = ETH_AutomaticPadCRCStrip_Disable;  //�ر��Զ�ȥ��PDA/CRC���� 
	ETH_InitStructure.ETH_ReceiveAll = ETH_ReceiveAll_Disable;  //�������������͵�֡
	ETH_InitStructure.ETH_BroadcastFramesReception = ETH_BroadcastFramesReception_Enable;  //����������й㲥֡
	ETH_InitStructure.ETH_PromiscuousMode = ETH_PromiscuousMode_Disable;  //�رջ��ģʽ�ĵ�ַ����  
	ETH_InitStructure.ETH_MulticastFramesFilter = ETH_MulticastFramesFilter_Perfect;  //�����鲥��ַʹ��������ַ����   
	ETH_InitStructure.ETH_UnicastFramesFilter = ETH_UnicastFramesFilter_Perfect;  //�Ե�����ַʹ��������ַ���� 

  /* FreeRTOSTCPIPֻ֧��Ӳ������ipv4У��ͣ���֧��TCP/UDP/ICMPУ��� */
	ETH_InitStructure.ETH_ChecksumOffload = ETH_ChecksumOffload_Disable;  //������ipv4��TCP/UDP/ICMP��֡У���ж��
	//������ʹ��֡У���ж�ع��ܵ�ʱ��һ��Ҫʹ�ܴ洢ת��ģʽ,�洢ת��ģʽ��Ҫ��֤����֡�洢��FIFO��,
	//����MAC�ܲ���/ʶ���֡У��ֵ,����У����ȷ��ʱ��DMA�Ϳ��Դ���֡,����Ͷ�������֡
	ETH_InitStructure.ETH_DropTCPIPChecksumErrorFrame = ETH_DropTCPIPChecksumErrorFrame_Enable;  //��������TCP/IP����֡
	ETH_InitStructure.ETH_ReceiveStoreForward = ETH_ReceiveStoreForward_Enable;  //�����������ݵĴ洢ת��ģʽ    
	ETH_InitStructure.ETH_TransmitStoreForward = ETH_TransmitStoreForward_Enable;  //�����������ݵĴ洢ת��ģʽ  

	ETH_InitStructure.ETH_ForwardErrorFrames = ETH_ForwardErrorFrames_Disable;  //��ֹת������֡  
	ETH_InitStructure.ETH_ForwardUndersizedGoodFrames = ETH_ForwardUndersizedGoodFrames_Disable;  //��ת����С�ĺ�֡ 
	ETH_InitStructure.ETH_SecondFrameOperate = ETH_SecondFrameOperate_Enable;  //�򿪴���ڶ�֡����
	ETH_InitStructure.ETH_AddressAlignedBeats = ETH_AddressAlignedBeats_Enable;  //����DMA����ĵ�ַ���빦��
	ETH_InitStructure.ETH_FixedBurst = ETH_FixedBurst_Enable;  //�����̶�ͻ������    
	ETH_InitStructure.ETH_RxDMABurstLength = ETH_RxDMABurstLength_32Beat;  //DMA���͵����ͻ������Ϊ32������   
	ETH_InitStructure.ETH_TxDMABurstLength = ETH_TxDMABurstLength_32Beat;  //DMA���յ����ͻ������Ϊ32������
	ETH_InitStructure.ETH_DMAArbitration = ETH_DMAArbitration_RoundRobin_RxTx_2_1;
	rval = ETH_Init(&ETH_InitStructure, LAN8720_PHY_ADDRESS);  //����ETH

  /* �����ж� */
	if(rval == ETH_SUCCESS)  //���óɹ�
	{
		ETH_DMAITConfig(ETH_DMA_IT_NIS | ETH_DMA_IT_R, ENABLE);  //ʹ����̫�������ж�
	}

	return rval;
}

//�ŵ�FreeRTOSIPHook.c������
//void ETH_IRQHandler(void)  //��̫��DMA�����жϷ�����
//{
//	while(ETH_GetRxPktSize(DMARxDescToGet) != 0) 	//����Ƿ��յ����ݰ�
//	{
//		
//	}
//	//��֪ͨ������
//	ETH_DMAClearITPendingBit(ETH_DMA_IT_R);  //���DMA�жϱ�־λ
//	ETH_DMAClearITPendingBit(ETH_DMA_IT_NIS);  //���DMA�����жϱ�־λ
//}

//������̫�����ݰ�
FrameTypeDef ETH_Rx_Packet(void)
{
	FrameTypeDef frame = {0, 0, NULL};   
	//��鵱ǰ������,�Ƿ�����ETHERNET DMA(���õ�ʱ��)/CPU(��λ��ʱ��)
	if((DMARxDescToGet->Status & ETH_DMARxDesc_OWN) != (u32)RESET)
	{
		frame.length = ETH_ERROR;  /* ETH_ERRORֵΪ0 */
		if((ETH->DMASR & ETH_DMASR_RBUS) != (u32)RESET)  
		{
			ETH->DMASR = ETH_DMASR_RBUS;  //���ETH DMA��RBUSλ 
			ETH->DMARPDR = 0;  //�ָ�DMA����
		}
		return frame;  //����OWNλ��������
	}
	if(((DMARxDescToGet->Status & ETH_DMARxDesc_ES) == (u32)RESET) && \
	   ((DMARxDescToGet->Status & ETH_DMARxDesc_LS) != (u32)RESET) && \
	   ((DMARxDescToGet->Status & ETH_DMARxDesc_FS) != (u32)RESET))  
	{
		frame.length = ((DMARxDescToGet->Status & ETH_DMARxDesc_FL) >> ETH_DMARxDesc_FrameLengthShift) - 4;  //�õ����հ�֡����(������4�ֽ�CRC)
 		frame.buffer = DMARxDescToGet->Buffer1Addr;  //�õ����������ڵ�λ��
	}
	else
	{
	  frame.length = ETH_ERROR;  //����
	}
	frame.descriptor = DMARxDescToGet;  
	//����ETH DMAȫ��Rx������Ϊ��һ��Rx������
	//Ϊ��һ��buffer��ȡ������һ��DMA Rx������
	DMARxDescToGet = (ETH_DMADESCTypeDef*)(DMARxDescToGet->Buffer2NextDescAddr);   
	return frame;  
}

//������̫�����ݰ�
u8 ETH_Tx_Packet(u16 FrameLength)
{
	//��鵱ǰ������,�Ƿ�����ETHERNET DMA(���õ�ʱ��)/CPU(��λ��ʱ��)
	if((DMATxDescToSet->Status & ETH_DMATxDesc_OWN) != (u32)RESET)
	{
	  return ETH_ERROR;  //����OWNλ��������
	}
 	DMATxDescToSet->ControlBufferSize = (FrameLength & ETH_DMATxDesc_TBS1);  //����֡����,bits[12:0]
	DMATxDescToSet->Status |= ETH_DMATxDesc_LS | ETH_DMATxDesc_FS;  //�������һ���͵�һ��λ����λ(1������������һ֡)
  DMATxDescToSet->Status |= ETH_DMATxDesc_OWN;  //����Tx��������OWNλ,buffer�ع�ETH DMA
	if((ETH->DMASR & ETH_DMASR_TBUS) != (u32)RESET)  //��Tx Buffer������λ(TBUS)�����õ�ʱ��,�������ָ�����
	{
		ETH->DMASR = ETH_DMASR_TBUS;  //����ETH DMA TBUSλ 
		ETH->DMATPDR = 0;  //�ָ�DMA����
	}
	//����ETH DMAȫ��Tx������Ϊ��һ��Tx������
	//Ϊ��һ��buffer����������һ��DMA Tx������ 
	DMATxDescToSet = (ETH_DMADESCTypeDef*)(DMATxDescToSet->Buffer2NextDescAddr);    
	return ETH_SUCCESS;   
}

//�õ���ǰ��������Tx buffer��ַ
u32 ETH_GetCurrentTxBuffer(void)
{
  return DMATxDescToSet->Buffer1Addr;  //����Tx buffer��ַ  
}



















