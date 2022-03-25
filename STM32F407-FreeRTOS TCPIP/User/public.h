#ifndef _PUBLIC_H_
#define _PUBLIC_H_

#include "stm32f4xx.h"
#include "stdio.h"

#define mPrintf  printf  //������ӡ
#define mTcpIpDebug  //printf  //�������� FreeRTOS TCPIP
#define mDebug  //printf  //��������
#define mTest   //printf  //��������

#define USE_OS_NONE        0
#define USE_OS_FREERTOS    1
#define USE_OS_UCOSII      2
#define USE_OS_UCOSIII     3
#define SYSTEM_SUPPORT_OS  USE_OS_FREERTOS  //ʹ��FreeRTOS

#define STM32_INT_DIS       __set_FAULTMASK(1);  //�ر�STM32�����ж�
#define STM32_INT_EN        __set_FAULTMASK(0);  //�����ж�
#define STM32_SYSTEM_RESET  __set_FAULTMASK(1); NVIC_SystemReset(); while(1);  //STM32�����λ

//ModbusCRCУ�麯��
u16 getModbusCRC16(uint8_t *puchMsg, uint8_t usDataLen);

//����ϵͳ�δ�ʱ�����ú���
#if SYSTEM_SUPPORT_OS == USE_OS_UCOSII  //�����Ҫ֧��uCOS-II
void mOSTickTimerCfg(void);
#endif

//��ʱ����
void delay(u32 time);
void delay_us(u16 us);
void delay_ms(u16 ms);
void delay_s(u16 s);

//NVIC���ú���
void mNVIC_config(void);

//���Ź�����
void mIWDG_config(u8 prer, u16 rlr);
void mIWDG_feed(void);

//�������������ʼ��������
void mRNG_Init(void);
u32 getRNG_value(void);

/*---------------------------------------------
�ڲ�FLASH��д����
��������FLASH��������Sector 11
��ַ0x080E0000-0x080FFFFF
������С128K
����ʹ�õĺ�����֤��ַ��1��2��4�ı���
STM32F407��FLASH�洢ʹ�õ���С��ģʽ
---------------------------------------------*/
#define FLASH_START_ADDRESS 0x080E0000
#define FLASH_END_ADDRESS   0x080FFFFF

u8 mFLASH_readByte(u32 add);
void mFLASH_readMoreByte(u32 add, u32 length, u8 buf[]);
void mFLASH_writeMoreByte(u32 add, u32 length, u8 dat[]);
u16 mFLASH_readHalf(u32 add);
void mFLASH_readMoreHalf(u32 add, u32 length, u16 buf[]);
void mFLASH_writeMoreHalf(u32 add, u32 length, u16 dat[]);
u32 mFLASH_readWord(u32 add);
void mFLASH_readMoreWord(u32 add, u32 length, u32 buf[]);
void mFLASH_writeMoreWord(u32 add, u32 length, u32 dat[]);

//�������ú���ģ��
int fputc(int ch, FILE *f);
int GetKey(void);
void mUSART1_config(u32 baudRate);
void mUSART2_config(u32 baudRate);
void mUSART3_config(u32 baudRate);
void mUART4_config(u32 baudRate);
void mUART5_config(u32 baudRate);
void mUSART6_config(u32 baudRate);

//��ʱ������ģ��
void mTIMER2_Config(u16 TIM_period, u16 TIM_prescaler, u8 TIM_clockDivision);
void mTIMER3_Config(u16 TIM_period, u16 TIM_prescaler, u8 TIM_clockDivision);
void mTIMER4_Config(u16 TIM_period, u16 TIM_prescaler, u8 TIM_clockDivision);
void mTIMER5_Config(u16 TIM_period, u16 TIM_prescaler, u8 TIM_clockDivision);

//DAC����ģ��
void mDAC_CH1_config(float refer);
void mDAC_CH1_output(float mV);

#endif



















