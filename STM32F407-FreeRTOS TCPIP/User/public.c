#include "public.h"

#if SYSTEM_SUPPORT_OS == USE_OS_UCOSII  //�����Ҫ֧��uCOS-II
#include "os_cfg.h"
#endif

static const u8 modbusAuchCRCHi[256] = {
  0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
  0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
  0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
  0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
  0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
  0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
  0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
  0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
  0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
  0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
  0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
  0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
  0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
  0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
  0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
  0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
  0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
  0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
  0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
  0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
  0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
  0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
  0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
  0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
  0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
  0x80, 0x41, 0x00, 0xC1, 0x81, 0x40 };

static const u8 modbusAuchCRCLo[256] = {
  0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06,
  0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD,
  0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
  0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A,
  0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC, 0x14, 0xD4,
  0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
  0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3,
  0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4,
  0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
  0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29,
  0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED,
  0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
  0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60,
  0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67,
  0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
  0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68,
  0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E,
  0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
  0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71,
  0x70, 0xB0, 0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92,
  0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
  0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B,
  0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B,
  0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
  0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42,
  0x43, 0x83, 0x41, 0x81, 0x80, 0x40 };

u16 getModbusCRC16(uint8_t *puchMsg, uint8_t usDataLen)
{
  uint8_t uchCRCHi = 0xFF;  /* ��CRC�ֽڳ�ʼ�� */
  uint8_t uchCRCLo = 0xFF;  /* ��CRC �ֽڳ�ʼ�� */
  uint16_t uIndex = 0;  /* CRCѭ���е����� */
  while(usDataLen--)  /* ������Ϣ������ */
  {
    uIndex = uchCRCHi ^ *(puchMsg++);  /* ����CRC */
    uchCRCHi = uchCRCLo ^ modbusAuchCRCHi[uIndex];
    uchCRCLo = modbusAuchCRCLo[uIndex];
  }
  return (uint16_t)((uint16_t)uchCRCHi << 8 | uchCRCLo);
}

#if SYSTEM_SUPPORT_OS == USE_OS_UCOSII  //�����Ҫ֧��uCOS-II
void mOSTickTimerCfg(void)
{
  SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);	 //�δ�ʱ��ʱ��ѡ���ⲿʱ��8��Ƶ
  u32 reload = SystemCoreClock / 8;  //ÿ���Ӽ�������
  reload /= OS_TICKS_PER_SEC;  //����OS_TICKS_PER_SEC�趨���ʱ�䣬reloadΪ24λ�Ĵ���,���ֵ:16777216
  SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;  //����SYSTICK�ж�
  SysTick->LOAD = reload;  //ÿ1/OS_CFG_TICK_RATE_HZ���ж�һ��
  SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;  //����SYSTICK
}
#endif

void delay(u32 time)
{
  while(time--);
}

#if SYSTEM_SUPPORT_OS != USE_OS_NONE  //�����Ҫ֧��OS
void delay_us(u16 us)
{
  u8 i = 0;
  while(us--)
  {
    i = 24;
    while(i--);
  }
}

void delay_ms(u16 ms)
{
  u16 i = 0;
  while(ms--)
  {
    i = 28000;
    while(i--);
  }
}
#else
#define SYSCLK 168  //ϵͳʱ��
static u8  fac_us = 0;  //us��ʱ������			   
static u16 fac_ms = 0;  //ms��ʱ����������ucos�´���ÿ�����ĵ�ms��

void delayInit(void)
{
  SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);
	fac_us = SYSCLK / 8;	 
	fac_ms = (u16)fac_us * 1000; //ÿ��ms��Ҫ��systickʱ���� 
}

void delay_us(u16 us)
{
  u32 midtime = 0;	    	 
	SysTick->LOAD = us * fac_us;  //ʱ�����(SysTick->LOADΪ24bit)		 
	SysTick->VAL = 0x00;  //��ռ�����
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;  //��ʼ���� 
	do{
		midtime = SysTick->CTRL;
	}while((midtime & 0x01) && !(midtime & (1 << 16)));  //�ȴ�ʱ�䵽��   
	SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;  //�رռ�����
	SysTick->VAL = 0x00;  //��ռ�����	 
}

void delay_ms(u16 ms)
{
  u32 midtime = 0;		   
	SysTick->LOAD = (u32)ms * fac_ms;  //ʱ�����(SysTick->LOADΪ24bit)
	SysTick->VAL = 0x00;  //��ռ�����
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk ;  //��ʼ����  
	do{
		midtime = SysTick->CTRL;
	}while((midtime & 0x01) && !(midtime & (1 << 16)));  //�ȴ�ʱ�䵽��   
	SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;  //�رռ�����
	SysTick->VAL = 0x00;  //��ռ�����
}

void delay_s(u16 s)
{
  while(s--)
	{
	  delay_ms(250);  //��ʱms���ܴ���798
		mIWDG_feed();  //ιһ�ι�
		delay_ms(250);  //��ʱms���ܴ���798
		mIWDG_feed();  //ιһ�ι�
		delay_ms(250);  //��ʱms���ܴ���798
		mIWDG_feed();  //ιһ�ι�
		delay_ms(250);  //��ʱms���ܴ���798
		mIWDG_feed();  //ιһ�ι�
	}
}
#endif

void mNVIC_config(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;  //����NVIC��ʼ���ṹ��

  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);  //��ռ���ȼ��ʹ����ȼ�����λ
	
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;  //����1�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //��ռ���ȼ�0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  //�����ȼ�2
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;  //IRQ ͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);  //����ָ���Ĳ�����ʼ�� NVIC �Ĵ���
}

//Ԥ��Ƶ��4������ֵ500�����ʱ��Ϊ1s
void mIWDG_config(u8 prer, u16 rlr)
{
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);  //ʹ�ܶ�IWDG->PR IWDG->RLR��д	
	IWDG_SetPrescaler(prer);  //����IWDG��Ƶϵ������Ƶ����=4*2^prer�����Ϊ256
	IWDG_SetReload(rlr);  //����IWDGװ��ֵ
	IWDG_ReloadCounter();  //reload
	IWDG_Enable();  
}

//ι�������Ź�
void mIWDG_feed(void)
{
	IWDG_ReloadCounter();  //reload
}

//��ʼ�������������
void mRNG_Init(void)
{
  RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_RNG, ENABLE);  //ʹ��RNGʱ��
  RNG_Cmd(ENABLE);  //ʹ��RNG����
}

//��ȡӲ�������
u32 getRNG_value(void)
{
  while(RNG_GetFlagStatus(RNG_FLAG_DRDY) == RESET);  //�ȴ�ת������
	return RNG_GetRandomNumber();  //��ȡ�����
}

u8 mFLASH_readByte(u32 add)
{
  if((add < FLASH_START_ADDRESS) || (add > FLASH_END_ADDRESS))
	{
	  return 0;
	}
	return *(__IO uint8_t*)add;
}

void mFLASH_readMoreByte(u32 add, u32 length, u8 buf[])
{
  if((add < FLASH_START_ADDRESS) || ((add + length- 1) > FLASH_END_ADDRESS))
	{
	  return;
	}
	while(length--)
	{
	  *(buf++) = *(__IO uint8_t*)(add++);
	}
}

void mFLASH_writeMoreByte(u32 add, u32 length, u8 dat[])
{
  if((add < FLASH_START_ADDRESS) || ((add + length - 1) > FLASH_END_ADDRESS))
	{
	  return;
	}
	FLASH_Unlock(); 
  FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | 
                  FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR); 
	FLASH_EraseSector(FLASH_Sector_11, VoltageRange_3);  //���֣�32bit��Ϊ��λ���в���
	while(length--)
	{
		FLASH_ProgramByte(add++, *(dat++));
	}
	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | 
                  FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
	FLASH_Lock();
}

u16 mFLASH_readHalf(u32 add)
{
  if((add < FLASH_START_ADDRESS) || ((add + 1) > FLASH_END_ADDRESS))
	{
	  return 0;
	}
  return *(__IO uint16_t*)add;
}

void mFLASH_readMoreHalf(u32 add, u32 length, u16 buf[])
{
  if((add < FLASH_START_ADDRESS) || ((add + length * 2 - 1) > FLASH_END_ADDRESS))
	{
	  return;
	}
	while(length--)
	{
	  *(buf++) = *(__IO uint16_t*)add;
		add += 2;
	}
}

void mFLASH_writeMoreHalf(u32 add, u32 length, u16 dat[])
{
  if((add < FLASH_START_ADDRESS) || ((add + length * 2 - 1) > FLASH_END_ADDRESS))
	{
	  return;
	}
	FLASH_Unlock(); 
  FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | 
                  FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR); 
	FLASH_EraseSector(FLASH_Sector_11, VoltageRange_3);  //���֣�32bit��Ϊ��λ���в���
	while(length--)
	{
		FLASH_ProgramHalfWord(add, *(dat++));
		add += 2;
	}
	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | 
                  FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
	FLASH_Lock();
}

u32 mFLASH_readWord(u32 add)
{
	if((add < FLASH_START_ADDRESS) || ((add + 3) > FLASH_END_ADDRESS))
	{
	  return 0;
	}
  return *(__IO uint32_t*)add;
}

void mFLASH_readMoreWord(u32 add, u32 length, u32 buf[])
{
  if((add < FLASH_START_ADDRESS) || ((add + length * 4 - 1) > FLASH_END_ADDRESS))
	{
	  return;
	}
	while(length--)
	{
	  *(buf++) = *(__IO uint32_t*)add;
		add += 4;
	}
}

void mFLASH_writeMoreWord(u32 add, u32 length, u32 dat[])
{
  if((add < FLASH_START_ADDRESS) || ((add + length * 4 - 1) > FLASH_END_ADDRESS))
	{
	  return;
	}
	FLASH_Unlock(); 
  FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | 
                  FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR); 
	FLASH_EraseSector(FLASH_Sector_11, VoltageRange_3);  //���֣�32bit��Ϊ��λ���в���
	while(length--)
	{
	  FLASH_ProgramWord(add, *(dat++));
		add += 4;
	}
	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | 
                  FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
	FLASH_Lock();
}

int fputc(int ch, FILE *f)
{
  USART_ClearFlag(USART1, USART_FLAG_TC);
  USART_SendData(USART1, (uint8_t)ch);
  while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
  return ch;
}

int GetKey(void)
{
  while(!(USART1->SR & USART_FLAG_RXNE));
  return((int)(USART1->DR & 0x1FF));
}

void mUSART1_config(u32 baudRate)
{
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;  //GPIOA9 �� GPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;  //���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  //�ٶ� 50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;  //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;  //����
	GPIO_Init(GPIOA, &GPIO_InitStructure);  //��ʼ�� PA9��PA10
	
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);  //PA9 ����Ϊ USART1
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);  //PA10 ����Ϊ USART1
	
	USART_InitStructure.USART_BaudRate = baudRate;  //һ������Ϊ 9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;  //�ֳ�Ϊ 8 λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;  //һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;  //����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;  //�շ�ģʽ
	USART_Init(USART1, &USART_InitStructure);  //��ʼ������
	
	USART_Cmd(USART1, ENABLE);  //ʹ�ܴ���
	USART_ClearFlag(USART1, USART_FLAG_TC);  //���USART�Ĵ������־λ
//  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);  //ʹ��USART1�����ж�
//	USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);  //ʹ��USART1 IDLE�ж�
}

void USART1_IRQHandler(void)  //�жϺ���������������
{
  u8 clear = clear;
  if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //���������һ���ֽ�
  {
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);

  }
	if(USART_GetITStatus(USART1, USART_IT_IDLE) != RESET)  //����յ�һ֡����
	{
	  clear = USART1->SR;  //��SR�Ĵ���
		clear = USART1->DR;  //��DR�Ĵ���(�ȶ�SR�ٶ�DR�����IDLE�ж�)

	}
}

void mUSART2_config(u32 baudRate)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;  //GPIOA2 �� GPIOA3
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;  //���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  //�ٶ� 50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;  //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;  //����
	GPIO_Init(GPIOA, &GPIO_InitStructure);  //��ʼ�� PA2��PA3
	
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);  //PA2 ����Ϊ USART2
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);  //PA3 ����Ϊ USART2
	
	USART_InitStructure.USART_BaudRate = baudRate;  //һ������Ϊ 9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;  //�ֳ�Ϊ 8 λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;  //һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;  //����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;  //�շ�ģʽ
	USART_Init(USART2, &USART_InitStructure);  //��ʼ������
	
	USART_Cmd(USART2, ENABLE);  //ʹ�ܴ���
	USART_ClearFlag(USART2, USART_FLAG_TC);  //���USART�Ĵ������־λ
//  USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);  //ʹ��USART2�����ж�
//	USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);  //ʹ��USART2 IDLE�ж�
}

void USART2_IRQHandler(void)  //�жϺ���������������
{
  u8 clear = clear;
  if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)  //���������һ���ֽ�
  {
		USART_ClearITPendingBit(USART2, USART_IT_RXNE);

  }
	if(USART_GetITStatus(USART2, USART_IT_IDLE) != RESET)  //����յ�һ֡����
	{
	  clear = USART2->SR;  //��SR�Ĵ���
		clear = USART2->DR;  //��DR�Ĵ���(�ȶ�SR�ٶ�DR�����IDLE�ж�)

	}
}

void mUSART3_config(u32 baudRate)
{
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;  //GPIOB10 �� GPIOB11
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;  //���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  //�ٶ� 50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;  //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;  //����
	GPIO_Init(GPIOB, &GPIO_InitStructure);  //��ʼ�� PB10��PB11
	
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);  //PB10 ����Ϊ USART3
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3);  //PB11 ����Ϊ USART3
	
	USART_InitStructure.USART_BaudRate = baudRate;  //һ������Ϊ 9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;  //�ֳ�Ϊ 8 λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;  //һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;  //����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;  //�շ�ģʽ
	USART_Init(USART3, &USART_InitStructure);  //��ʼ������
	
	USART_Cmd(USART3, ENABLE);  //ʹ�ܴ���
	USART_ClearFlag(USART3, USART_FLAG_TC);  //���USART�Ĵ������־λ
//  USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);  //ʹ��USART3�����ж�
//	USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);  //ʹ��USART3 IDLE�ж�
}

void USART3_IRQHandler(void)  //�жϺ���������������
{
  u8 clear = clear;
  if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)  //���������һ���ֽ�
  {
		USART_ClearITPendingBit(USART3, USART_IT_RXNE);

  }
	if(USART_GetITStatus(USART3, USART_IT_IDLE) != RESET)  //����յ�һ֡����
	{
	  clear = USART3->SR;  //��SR�Ĵ���
		clear = USART3->DR;  //��DR�Ĵ���(�ȶ�SR�ٶ�DR�����IDLE�ж�)

	}
}

void mUART4_config(u32 baudRate)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;  //GPIOC10 �� GPIOC11
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;  //���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  //�ٶ� 50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;  //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;  //����
	GPIO_Init(GPIOC, &GPIO_InitStructure);  //��ʼ�� PC10��PC11
	
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_UART4);  //PC10 ����Ϊ UART4
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_UART4);  //PC11 ����Ϊ UART4
	
	USART_InitStructure.USART_BaudRate = baudRate;  //һ������Ϊ 9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;  //�ֳ�Ϊ 8 λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;  //һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;  //����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;  //�շ�ģʽ
	USART_Init(UART4, &USART_InitStructure);  //��ʼ������
	
	USART_Cmd(UART4, ENABLE);  //ʹ�ܴ���
	USART_ClearFlag(UART4, USART_FLAG_TC);  //���USART�Ĵ������־λ
//  USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);  //ʹ��UART4�����ж�
//	USART_ITConfig(UART4, USART_IT_IDLE, ENABLE);  //ʹ��UART4 IDLE�ж�
}

void UART4_IRQHandler(void)  //�жϺ���������������
{
  u8 clear = clear;
  if(USART_GetITStatus(UART4, USART_IT_RXNE) != RESET)  //���������һ���ֽ�
  {
		USART_ClearITPendingBit(UART4, USART_IT_RXNE);

  }
	if(USART_GetITStatus(UART4, USART_IT_IDLE) != RESET)  //����յ�һ֡����
	{
	  clear = UART4->SR;  //��SR�Ĵ���
		clear = UART4->DR;  //��DR�Ĵ���(�ȶ�SR�ٶ�DR�����IDLE�ж�)

	}
}

void mUART5_config(u32 baudRate)
{
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;  //GPIOC12
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;  //���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  //�ٶ� 50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;  //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;  //����
	GPIO_Init(GPIOC, &GPIO_InitStructure);  //��ʼ�� PC12
	
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_UART5);  //PC12 ����Ϊ UART5
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;  //GPIOD2
	GPIO_Init(GPIOD, &GPIO_InitStructure);  //��ʼ�� PD2
	
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource2, GPIO_AF_UART5);  //PD2 ����Ϊ UART5
	
	USART_InitStructure.USART_BaudRate = baudRate;  //һ������Ϊ 9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;  //�ֳ�Ϊ 8 λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;  //һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;  //����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;  //�շ�ģʽ
	USART_Init(UART5, &USART_InitStructure);  //��ʼ������
	
	USART_Cmd(UART5, ENABLE);  //ʹ�ܴ���
	USART_ClearFlag(UART5, USART_FLAG_TC);  //���USART�Ĵ������־λ
//  USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);  //ʹ��UART5�����ж�
//	USART_ITConfig(UART5, USART_IT_IDLE, ENABLE);  //ʹ��UART5 IDLE�ж�
}

void UART5_IRQHandler(void)  //�жϺ���������������
{
  u8 clear = clear;
  if(USART_GetITStatus(UART5, USART_IT_RXNE) != RESET)  //���������һ���ֽ�
  {
		USART_ClearITPendingBit(UART5, USART_IT_RXNE);

  }
	if(USART_GetITStatus(UART5, USART_IT_IDLE) != RESET)  //����յ�һ֡����
	{
	  clear = UART5->SR;  //��SR�Ĵ���
		clear = UART5->DR;  //��DR�Ĵ���(�ȶ�SR�ٶ�DR�����IDLE�ж�)

	}
}

void mUSART6_config(u32 baudRate)
{
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;  //GPIOC6 �� GPIOC7
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;  //���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  //�ٶ� 50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;  //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;  //����
	GPIO_Init(GPIOC, &GPIO_InitStructure);  //��ʼ�� PC6��PC7
	
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_USART6);  //PC6 ����Ϊ USART6
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_USART6);  //PC7 ����Ϊ USART6
	
	USART_InitStructure.USART_BaudRate = baudRate;  //һ������Ϊ 9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;  //�ֳ�Ϊ 8 λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;  //һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;  //����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;  //�շ�ģʽ
	USART_Init(USART6, &USART_InitStructure);  //��ʼ������
	
	USART_Cmd(USART6, ENABLE);  //ʹ�ܴ���
	USART_ClearFlag(USART6, USART_FLAG_TC);  //���USART�Ĵ������־λ
//  USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);  //ʹ��USART6�����ж�
//	USART_ITConfig(USART6, USART_IT_IDLE, ENABLE);  //ʹ��USART6 IDLE�ж�
}

void USART6_IRQHandler(void)  //�жϺ���������������
{
  u8 clear = clear;
  if(USART_GetITStatus(USART6, USART_IT_RXNE) != RESET)  //���������һ���ֽ�
  {
		USART_ClearITPendingBit(USART6, USART_IT_RXNE);

  }
	if(USART_GetITStatus(USART6, USART_IT_IDLE) != RESET)  //����յ�һ֡����
	{
	  clear = USART6->SR;  //��SR�Ĵ���
		clear = USART6->DR;  //��DR�Ĵ���(�ȶ�SR�ٶ�DR�����IDLE�ж�)

	}
}

//��ʱʱ��Tout=(TIM_period+1)*(TIM_prescaler+1)/(84M/TIM_clockDivision)
void mTIMER2_Config(u16 TIM_period, u16 TIM_prescaler, u8 TIM_clockDivision)
{
  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

  TIM_TimeBaseInitStructure.TIM_Period = TIM_period;  //�Զ���װ��ֵ
  TIM_TimeBaseInitStructure.TIM_Prescaler = TIM_prescaler;  //��ʱ����Ƶ
  TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_clockDivision;  
  TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;  //���ϼ���ģʽ
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);

  TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
  TIM_Cmd(TIM2, ENABLE);  //ʹ��TIMx����
//  TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);  //ʹ��ָ����TIM�ж�
}

void TIM2_IRQHandler(void)   //��ʱ��2�жϺ���������������
{
  if(TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
  {
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);  //���жϱ�־
		
  }
}

//��ʱʱ��Tout=(TIM_period+1)*(TIM_prescaler+1)/(84M/TIM_clockDivision)
void mTIMER3_Config(u16 TIM_period, u16 TIM_prescaler, u8 TIM_clockDivision)
{
  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

  TIM_TimeBaseInitStructure.TIM_Period = TIM_period;  //�Զ���װ��ֵ
  TIM_TimeBaseInitStructure.TIM_Prescaler = TIM_prescaler;  //��ʱ����Ƶ
  TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_clockDivision;  
  TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;  //���ϼ���ģʽ
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStructure);

  TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
  TIM_Cmd(TIM3, ENABLE);  //ʹ��TIMx����
//  TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);  //ʹ��ָ����TIM�ж�
}

void TIM3_IRQHandler(void)   //��ʱ��3�жϺ���������������
{
  if(TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
  {
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);  //���жϱ�־
		
  }
}

//��ʱʱ��Tout=(TIM_period+1)*(TIM_prescaler+1)/(84M/TIM_clockDivision)
void mTIMER4_Config(u16 TIM_period, u16 TIM_prescaler, u8 TIM_clockDivision)
{
  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

  TIM_TimeBaseInitStructure.TIM_Period = TIM_period;  //�Զ���װ��ֵ
  TIM_TimeBaseInitStructure.TIM_Prescaler = TIM_prescaler;  //��ʱ����Ƶ
  TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_clockDivision;  
  TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;  //���ϼ���ģʽ
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStructure);

  TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
  TIM_Cmd(TIM4, ENABLE);  //ʹ��TIMx����
//  TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);  //ʹ��ָ����TIM�ж�
}

void TIM4_IRQHandler(void)   //��ʱ��4�жϺ���������������
{
  if(TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)
  {
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update);  //���жϱ�־
		
  }
}

//��ʱʱ��Tout=(TIM_period+1)*(TIM_prescaler+1)/(84M/TIM_clockDivision)
void mTIMER5_Config(u16 TIM_period, u16 TIM_prescaler, u8 TIM_clockDivision)
{
  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);

  TIM_TimeBaseInitStructure.TIM_Period = TIM_period;  //�Զ���װ��ֵ
  TIM_TimeBaseInitStructure.TIM_Prescaler = TIM_prescaler;  //��ʱ����Ƶ
  TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_clockDivision;  
  TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;  //���ϼ���ģʽ
  TIM_TimeBaseInit(TIM5, &TIM_TimeBaseInitStructure);

  TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
  TIM_Cmd(TIM5, ENABLE);  //ʹ��TIMx����
//  TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);  //ʹ��ָ����TIM�ж�
}

void TIM5_IRQHandler(void)   //��ʱ��5�жϺ���������������
{
  if(TIM_GetITStatus(TIM5, TIM_IT_Update) != RESET)
  {
		TIM_ClearITPendingBit(TIM5, TIM_IT_Update);  //���жϱ�־
		
  }
}

static float reference = 0.0f;  //��׼��ѹ����λV��
void mDAC_CH1_config(float refer)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	DAC_InitTypeDef DAC_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);  //ʹ��GPIOAʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);  //ʹ��DACʱ��

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;  //ģ��
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;  //����
	GPIO_Init(GPIOA, &GPIO_InitStructure);  //��ʼ��

	DAC_InitStructure.DAC_Trigger = DAC_Trigger_None;	 //��ʹ�ô������� TEN1=0
	DAC_InitStructure.DAC_WaveGeneration = DAC_WaveGeneration_None;  //��ʹ�ò��η���
	DAC_InitStructure.DAC_LFSRUnmask_TriangleAmplitude = DAC_LFSRUnmask_Bit0;  //���Ρ���ֵ����
	DAC_InitStructure.DAC_OutputBuffer = DAC_OutputBuffer_Disable;  //DAC1�������ر� BOFF1=1
	DAC_Init(DAC_Channel_2, &DAC_InitStructure);  //��ʼ��DACͨ��2

	DAC_Cmd(DAC_Channel_2, ENABLE);  //ʹ��DACͨ��2

	DAC_SetChannel2Data(DAC_Align_12b_R, 0);  //12λ�Ҷ������ݸ�ʽ����DACֵ	
	reference = refer;  //���û�׼Դ
}

void mDAC_CH1_output(float mV)
{
	DAC_SetChannel2Data(DAC_Align_12b_R, (double)mV * 4.096 / reference);
}



















