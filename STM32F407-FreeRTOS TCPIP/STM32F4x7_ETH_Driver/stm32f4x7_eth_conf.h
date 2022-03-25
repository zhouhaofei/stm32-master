#ifndef _STM32F4x7_ETH_CONF_H_
#define _STM32F4x7_ETH_CONF_H_

#include "stm32f4xx.h"

//使用强制描述符（校验和激活则必须使用强制描述符）
#define USE_ENHANCED_DMA_DESCRIPTORS

//#define USE_Delay  //使用默认延时函数，因此注销掉
#ifdef USE_Delay
	#include "public.h"               
	#define _eth_delay_    delay      //Delay为用户自己提供的高精度延时函数
#else
	#define _eth_delay_    ETH_Delay  //默认的_eth_delay功能函数延时精度差
#endif

/* 重定义以太网接收和发送缓冲区的大小和数量，默认为stm32f4x7_eth.h中的值
   正点原子把stm32f4x7_eth.c中缓冲区注释掉了，使用了外扩RAM */
#define  CUSTOM_DRIVER_BUFFERS_CONFIG
#ifdef  CUSTOM_DRIVER_BUFFERS_CONFIG
	#define ETH_RX_BUF_SIZE    ETH_MAX_PACKET_SIZE  //接收缓冲区的大小
	#define ETH_TX_BUF_SIZE    ETH_MAX_PACKET_SIZE  //发送缓冲区的大小
	#define ETH_RXBUFNB        5                    //接收缓冲区数量
	#define ETH_TXBUFNB        5                    //发送缓冲区数量
#endif

//PHY配置块
#ifdef USE_Delay
	#define PHY_RESET_DELAY      ((uint32_t)0x000000FF)  //PHY复位延时
	#define PHY_CONFIG_DELAY     ((uint32_t)0x00000FFF)  //PHY配置延时
	#define ETH_REG_WRITE_DELAY  ((uint32_t)0x00000001)	 //向以太网寄存器写数据时的延时
#else
	#define PHY_RESET_DELAY      ((uint32_t)0x000FFFFF)  //PHY复位延时
	#define PHY_CONFIG_DELAY     ((uint32_t)0x00FFFFFF)  //PHY配置延时
	#define ETH_REG_WRITE_DELAY  ((uint32_t)0x0000FFFF)	 //向以太网寄存器写数据时的延时
#endif

//LAN8720 PHY芯片的状态寄存器
#define PHY_SR				     ((uint16_t)31)       //LAN8720 PHY状态寄存器地址
#define PHY_SPEED_STATUS   ((uint16_t)0x0004) 	//LAN8720 PHY速度值掩码
#define PHY_DUPLEX_STATUS  ((uint16_t)0x00010)  //LAN8720 PHY连接状态值掩码

#endif 



















