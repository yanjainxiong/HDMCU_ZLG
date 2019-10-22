/******************************************************************************
* Copyright (C) 2016, Huada Semiconductor Co.,Ltd All rights reserved.
*
* This software is owned and published by:
* Huada Semiconductor Co.,Ltd ("HDSC").
*
* BY DOWNLOADING, INSTALLING OR USING THIS SOFTWARE, YOU AGREE TO BE BOUND
* BY ALL THE TERMS AND CONDITIONS OF THIS AGREEMENT.
*
* This software contains source code for use with HDSC
* components. This software is licensed by HDSC to be adapted only
* for use in systems utilizing HDSC components. HDSC shall not be
* responsible for misuse or illegal use of this software for devices not
* supported herein. HDSC is providing this software "AS IS" and will
* not be responsible for issues arising from incorrect user implementation
* of the software.
*
* Disclaimer:
* HDSC MAKES NO WARRANTY, EXPRESS OR IMPLIED, ARISING BY LAW OR OTHERWISE,
* REGARDING THE SOFTWARE (INCLUDING ANY ACOOMPANYING WRITTEN MATERIALS),
* ITS PERFORMANCE OR SUITABILITY FOR YOUR INTENDED USE, INCLUDING,
* WITHOUT LIMITATION, THE IMPLIED WARRANTY OF MERCHANTABILITY, THE IMPLIED
* WARRANTY OF FITNESS FOR A PARTICULAR PURPOSE OR USE, AND THE IMPLIED
* WARRANTY OF NONINFRINGEMENT.
* HDSC SHALL HAVE NO LIABILITY (WHETHER IN CONTRACT, WARRANTY, TORT,
* NEGLIGENCE OR OTHERWISE) FOR ANY DAMAGES WHATSOEVER (INCLUDING, WITHOUT
* LIMITATION, DAMAGES FOR LOSS OF BUSINESS PROFITS, BUSINESS INTERRUPTION,
* LOSS OF BUSINESS INFORMATION, OR OTHER PECUNIARY LOSS) ARISING FROM USE OR
* INABILITY TO USE THE SOFTWARE, INCLUDING, WITHOUT LIMITATION, ANY DIRECT,
* INDIRECT, INCIDENTAL, SPECIAL OR CONSEQUENTIAL DAMAGES OR LOSS OF DATA,
* SAVINGS OR PROFITS,
* EVEN IF Disclaimer HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
* YOU ASSUME ALL RESPONSIBILITIES FOR SELECTION OF THE SOFTWARE TO ACHIEVE YOUR
* INTENDED RESULTS, AND FOR THE INSTALLATION OF, USE OF, AND RESULTS OBTAINED
* FROM, THE SOFTWARE.
*
* This software may be replicated in part or whole for the licensed use,
* with the restriction that this Disclaimer and Copyright notice must be
* included with each copy of this software, whether used in part or whole,
* at all times.
*/
/******************************************************************************/
/** \file main.c
 **
 ** A detailed description is available at
 ** @link Sample Group Some description @endlink
 **
 **   - 2016-02-16  1.0  XYZ First version for Device Driver Library of Module.
 **
 ******************************************************************************/

/******************************************************************************
 * Include files
 ******************************************************************************/
#include "i2c.h"
#include "gpio.h"
/******************************************************************************
 * Local pre-processor symbols/macros ('#define')
 ******************************************************************************/

/******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/

/******************************************************************************
 * Local type definitions ('typedef')
 ******************************************************************************/

/******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/

/******************************************************************************
 * Local variable definitions ('static')                                      *
 ******************************************************************************/

/******************************************************************************
 * Local pre-processor symbols/macros ('#define')
 ******************************************************************************/
#define     T1_PORT                 (3)
#define     T1_PIN                  (3)

#define I2C_DEVADDR 0xA0
#define READLEN   10
/*****************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/
/**
 ******************************************************************************
 ** \brief  Main function of project
 **
 ** \return uint32_t return value, if needed
 **
 ** This sample
 **
 ******************************************************************************/
 //广播地址时，地址发送0x00，广播地址从机只接收，不回传
uint8_t u8Senddata[10] = {0x12,0x34,0x77,0x66,0x55,0x44,0x33,0x22,0x11,0x99};
uint8_t u8Recdata[10]={0x00};
uint8_t u8SendLen=0;
uint8_t u8RecvLen=0;
uint8_t SendFlg = 0,Comm_flg = 0;
uint8_t u8recvflg=0;
uint8_t u8State = 0;
uint8_t u8Addr = 0x00;//地址字节
//主发送函数
en_result_t I2C_MasterWriteData(en_i2c_channel_t enCh,uint8_t *pu8Data,uint32_t u32Len);
//主接收函数
en_result_t I2C_MasterReadData(en_i2c_channel_t enCh,uint8_t *pu8Data,uint32_t u32Len);
/**
 ******************************************************************************
 ** \brief  主机接收函数
 **
 ** \param u8Addr从机内存地址，pu8Data读数据存放缓存，u32Len读数据长度
 **
 ** \retval 读数据是否成功
 **
 ******************************************************************************/
 en_result_t I2C_MasterReadData(en_i2c_channel_t enCh,uint8_t *pu8Data,uint32_t u32Len)
{
    en_result_t enRet = Error;
    uint8_t u8i=0,u8State;
    
    I2C_SetFunc(enCh,I2cStart_En);
    
	while(1)
	{
		while(0 == I2C_GetIrq(enCh))
        {}
		u8State = I2C_GetState(enCh);
		switch(u8State)
		{
			case 0x08:
				I2C_ClearFunc(enCh,I2cStart_En);
				I2C_WriteByte(enCh,I2C_DEVADDR|0x01);//读命令发送
				break;
			case 0x10:
				I2C_ClearFunc(enCh,I2cStart_En);
				I2C_WriteByte(enCh,I2C_DEVADDR|0x01);//读命令发送
				break;
			case 0x40:
				if(u32Len>1)
				{
					I2C_SetFunc(enCh,I2cAck_En);
				}
				break;
			case 0x50:
				pu8Data[u8i++] = I2C_ReadByte(enCh);
				if(u8i==u32Len-1)
				{
					I2C_ClearFunc(enCh,I2cAck_En);//读数据时，倒数第二个字节ACK关闭
				}
				break;
			case 0x58:
				pu8Data[u8i++] = I2C_ReadByte(enCh);
				I2C_SetFunc(enCh,I2cStop_En);
				break;	
			case 0x38:
				I2C_SetFunc(enCh,I2cStart_En);
				break;
			case 0x48:
				I2C_SetFunc(enCh,I2cStop_En);
				I2C_SetFunc(enCh,I2cStart_En);
				break;
			default:
				I2C_SetFunc(enCh,I2cStart_En);//其他错误状态，重新发送起始条件
				break;
		}
		I2C_ClearIrq(enCh);
		if(u8i==u32Len)
		{
			break;
		}
	}
	enRet = Ok;
	return enRet;
}
/**
 ******************************************************************************
 ** \brief  主机发送函数
 **
 ** \param u8Addr从机内存地址，pu8Data写数据，u32Len写数据长度
 **
 ** \retval 写数据是否成功
 **
 ******************************************************************************/
en_result_t I2C_MasterWriteData(en_i2c_channel_t enCh,uint8_t *pu8Data,uint32_t u32Len)
{
    en_result_t enRet = Error;
    uint8_t u8i=0,u8State;
    I2C_SetFunc(enCh,I2cStart_En);
	while(1)
	{
		while(0 == I2C_GetIrq(enCh))
		{;}
		u8State = I2C_GetState(enCh);
		switch(u8State)
		{
			case 0x08:
				I2C_ClearFunc(enCh,I2cStart_En);
				I2C_WriteByte(enCh,I2C_DEVADDR);//从设备地址发送
				break;
			case 0x18://发送完写命令后即可发送数据
			case 0x28:	
				I2C_WriteByte(enCh,pu8Data[u8i++]);
				break;
			case 0x20:
			case 0x38:
				I2C_SetFunc(enCh,I2cStart_En);
				break;
			case 0x30:
				I2C_SetFunc(enCh,I2cStop_En);
				break;
			default:
				break;
		}			
		if(u8i>u32Len)
		{
			I2C_SetFunc(enCh,I2cStop_En);//此顺序不能调换，出停止条件
			I2C_ClearIrq(enCh);
			break;
		}
		I2C_ClearIrq(enCh);			
	}
    enRet = Ok;
    return enRet;
}
int32_t main(void)
{ 
    stc_gpio_config_t stcGpioCfg;
    stc_i2c_config_t stcI2cCfg;
    stc_sysctrl_clk_config_t stcCfg;
    DDL_ZERO_STRUCT(stcCfg);
    DDL_ZERO_STRUCT(stcI2cCfg);
    DDL_ZERO_STRUCT(stcGpioCfg);
    
    Sysctrl_ClkSourceEnable(SysctrlClkRCL,TRUE);
    Sysctrl_SysClkSwitch(SysctrlClkRCL);
    Sysctrl_SetRCHTrim(SysctrlRchFreq24MHz);
	Sysctrl_SysClkSwitch(SysctrlClkRCH);
	Sysctrl_ClkSourceEnable(SysctrlClkRCL,FALSE);
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio,TRUE);
    Sysctrl_SetPeripheralGate(SysctrlPeripheralI2c0,TRUE);
    
    stcGpioCfg.enDir = GpioDirOut;
    stcGpioCfg.enOD = GpioOdEnable;//开漏输出
    stcGpioCfg.enPuPd = GpioPu;
    
    Gpio_Init(GpioPortB,GpioPin8,&stcGpioCfg);//端口初始化
    Gpio_Init(GpioPortB,GpioPin9,&stcGpioCfg);
    
    Gpio_SetAfMode(GpioPortB,GpioPin8,GpioAf1);//SCL
    Gpio_SetAfMode(GpioPortB,GpioPin9,GpioAf1);//SDA
    
    stcI2cCfg.enFunc = I2cBaud_En;
    stcI2cCfg.u8Tm = 0x02;//1M=(24000000/(8*(2+1))波特率配置
    stcI2cCfg.pfnI2c0Cb = NULL;
    stcI2cCfg.bTouchNvic = FALSE;
    if(TRUE == stcI2cCfg.bTouchNvic)
	{
		EnableNvic(I2C0_IRQn,IrqLevel3,TRUE);
	} 
    I2C_DeInit(I2C0); 
    I2C_Init(I2C0,&stcI2cCfg);//模块初始化
    I2C_SetFunc(I2C0,I2cMode_En);//模块使能
    I2C_SetFunc(I2C0,I2cStart_En);//开始信号
	I2C_MasterWriteData(I2C0,u8Senddata,10);//主机发送10字节数据
	delay1ms(10);
	I2C_MasterReadData(I2C0,u8Recdata,10);//主机读取10字节数据
    while(1)
    {
    }
}

/******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/


