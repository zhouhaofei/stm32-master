#ifndef _LAN8720_H_
#define _LAN8720_H_

#include "stm32f4xx.h"
#include "stm32f4x7_eth.h"
#include "public.h"

typedef struct {
  u8 phy;  //PHY工作模式，保留
  u8 dhcp;
  u8 mac[6];
	u8 ip[4];  //IPv4
	u8 sub[4];
	u8 gw[4];
	u8 domainS1[4];
	u8 domainS2[4];
  u16 mtu;  //MTU值，保留
  u16 nc;  //占位，为了使结构体大小为4字节整数倍数
} NetworkParm;

#define LAN8720_PHY_ADDRESS  0x00  //LAN8720 PHY芯片地址

void LAN8720_Init(void);
u8 LAN8720_Get_Speed(void);
u8 ETH_MACDMA_Config(void);
FrameTypeDef ETH_Rx_Packet(void);
u8 ETH_Tx_Packet(u16 FrameLength);
u32 ETH_GetCurrentTxBuffer(void);

#endif 



















