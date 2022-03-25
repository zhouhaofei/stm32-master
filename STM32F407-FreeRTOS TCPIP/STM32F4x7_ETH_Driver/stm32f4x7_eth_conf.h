#ifndef _STM32F4x7_ETH_CONF_H_
#define _STM32F4x7_ETH_CONF_H_

#include "stm32f4xx.h"

//ʹ��ǿ����������У��ͼ��������ʹ��ǿ����������
#define USE_ENHANCED_DMA_DESCRIPTORS

//#define USE_Delay  //ʹ��Ĭ����ʱ���������ע����
#ifdef USE_Delay
	#include "public.h"               
	#define _eth_delay_    delay      //DelayΪ�û��Լ��ṩ�ĸ߾�����ʱ����
#else
	#define _eth_delay_    ETH_Delay  //Ĭ�ϵ�_eth_delay���ܺ�����ʱ���Ȳ�
#endif

/* �ض�����̫�����պͷ��ͻ������Ĵ�С��������Ĭ��Ϊstm32f4x7_eth.h�е�ֵ
   ����ԭ�Ӱ�stm32f4x7_eth.c�л�����ע�͵��ˣ�ʹ��������RAM */
#define  CUSTOM_DRIVER_BUFFERS_CONFIG
#ifdef  CUSTOM_DRIVER_BUFFERS_CONFIG
	#define ETH_RX_BUF_SIZE    ETH_MAX_PACKET_SIZE  //���ջ������Ĵ�С
	#define ETH_TX_BUF_SIZE    ETH_MAX_PACKET_SIZE  //���ͻ������Ĵ�С
	#define ETH_RXBUFNB        5                    //���ջ���������
	#define ETH_TXBUFNB        5                    //���ͻ���������
#endif

//PHY���ÿ�
#ifdef USE_Delay
	#define PHY_RESET_DELAY      ((uint32_t)0x000000FF)  //PHY��λ��ʱ
	#define PHY_CONFIG_DELAY     ((uint32_t)0x00000FFF)  //PHY������ʱ
	#define ETH_REG_WRITE_DELAY  ((uint32_t)0x00000001)	 //����̫���Ĵ���д����ʱ����ʱ
#else
	#define PHY_RESET_DELAY      ((uint32_t)0x000FFFFF)  //PHY��λ��ʱ
	#define PHY_CONFIG_DELAY     ((uint32_t)0x00FFFFFF)  //PHY������ʱ
	#define ETH_REG_WRITE_DELAY  ((uint32_t)0x0000FFFF)	 //����̫���Ĵ���д����ʱ����ʱ
#endif

//LAN8720 PHYоƬ��״̬�Ĵ���
#define PHY_SR				     ((uint16_t)31)       //LAN8720 PHY״̬�Ĵ�����ַ
#define PHY_SPEED_STATUS   ((uint16_t)0x0004) 	//LAN8720 PHY�ٶ�ֵ����
#define PHY_DUPLEX_STATUS  ((uint16_t)0x00010)  //LAN8720 PHY����״ֵ̬����

#endif 



















