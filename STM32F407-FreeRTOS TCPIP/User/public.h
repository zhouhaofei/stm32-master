#ifndef _PUBLIC_H_
#define _PUBLIC_H_

#include "stm32f4xx.h"
#include "stdio.h"

#define mPrintf  printf  //用来打印
#define mTcpIpDebug  //printf  //用来调试 FreeRTOS TCPIP
#define mDebug  //printf  //用来调试
#define mTest   //printf  //用来测试

#define USE_OS_NONE        0
#define USE_OS_FREERTOS    1
#define USE_OS_UCOSII      2
#define USE_OS_UCOSIII     3
#define SYSTEM_SUPPORT_OS  USE_OS_FREERTOS  //使用FreeRTOS

#define STM32_INT_DIS       __set_FAULTMASK(1);  //关闭STM32所有中断
#define STM32_INT_EN        __set_FAULTMASK(0);  //开启中断
#define STM32_SYSTEM_RESET  __set_FAULTMASK(1); NVIC_SystemReset(); while(1);  //STM32软件复位

//ModbusCRC校验函数
u16 getModbusCRC16(uint8_t *puchMsg, uint8_t usDataLen);

//操作系统滴答定时器配置函数
#if SYSTEM_SUPPORT_OS == USE_OS_UCOSII  //如果需要支持uCOS-II
void mOSTickTimerCfg(void);
#endif

//延时函数
void delay(u32 time);
void delay_us(u16 us);
void delay_ms(u16 ms);
void delay_s(u16 s);

//NVIC配置函数
void mNVIC_config(void);

//看门狗函数
void mIWDG_config(u8 prer, u16 rlr);
void mIWDG_feed(void);

//随机数生成器初始化及生成
void mRNG_Init(void);
u32 getRNG_value(void);

/*---------------------------------------------
内部FLASH读写函数
以下所有FLASH操作均在Sector 11
地址0x080E0000-0x080FFFFF
扇区大小128K
根据使用的函数保证地址是1，2，4的倍数
STM32F407在FLASH存储使用的是小端模式
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

//串口配置函数模版
int fputc(int ch, FILE *f);
int GetKey(void);
void mUSART1_config(u32 baudRate);
void mUSART2_config(u32 baudRate);
void mUSART3_config(u32 baudRate);
void mUART4_config(u32 baudRate);
void mUART5_config(u32 baudRate);
void mUSART6_config(u32 baudRate);

//定时器函数模版
void mTIMER2_Config(u16 TIM_period, u16 TIM_prescaler, u8 TIM_clockDivision);
void mTIMER3_Config(u16 TIM_period, u16 TIM_prescaler, u8 TIM_clockDivision);
void mTIMER4_Config(u16 TIM_period, u16 TIM_prescaler, u8 TIM_clockDivision);
void mTIMER5_Config(u16 TIM_period, u16 TIM_prescaler, u8 TIM_clockDivision);

//DAC函数模版
void mDAC_CH1_config(float refer);
void mDAC_CH1_output(float mV);

#endif



















