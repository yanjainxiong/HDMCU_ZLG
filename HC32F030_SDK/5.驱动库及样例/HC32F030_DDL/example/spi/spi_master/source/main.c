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
#include "ddl.h"
#include "spi.h"
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

/*****************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/
uint8_t data;
uint8_t u8Senddata[10] = {0x55,0x22,0x33,0x44,0x55,0x66,0x77,0x88,0x99,0xaa};
uint8_t u8Recvdata[10] = {0x00};
/**
 ******************************************************************************
 ** \brief  Main function of project
 **
 ** \return uint32_t return value, if needed
 **
 ** This sample
 **
 ******************************************************************************/
void Spi_PortInit(void)
{
    stc_gpio_config_t stcGpioCfg;
    DDL_ZERO_STRUCT(stcGpioCfg);
    
    stcGpioCfg.enDir = GpioDirOut;   
    Gpio_Init(GpioPortA, GpioPin12,&stcGpioCfg);//MOSI
    Gpio_SetAfMode(GpioPortA, GpioPin12,GpioAf6);   
    Gpio_Init(GpioPortA, GpioPin15,&stcGpioCfg);//CS
    Gpio_SetAfMode(GpioPortA, GpioPin15,GpioAf1);
    Gpio_Init(GpioPortB, GpioPin3,&stcGpioCfg);//SCK
    Gpio_SetAfMode(GpioPortB, GpioPin3,GpioAf1);
    stcGpioCfg.enDir = GpioDirIn;
    Gpio_Init(GpioPortA, GpioPin11,&stcGpioCfg);//MISO
    Gpio_SetAfMode(GpioPortA, GpioPin11,GpioAf6);
}
//字节间距2.5us
int32_t main(void)
{
    stc_spi_config_t  SPIConfig;
    uint8_t   i;
    DDL_ZERO_STRUCT(SPIConfig);
    
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio,TRUE);
    Sysctrl_SetPeripheralGate(SysctrlPeripheralSpi0,TRUE); //SPI0
    
    Spi_PortInit();//模块初始化
    
    Spi_SetCS(Spi0,TRUE);
    //SPI模块配置
    SPIConfig.bCPHA = Spicphafirst;//模式0
    SPIConfig.bCPOL = Spicpollow;
    SPIConfig.bIrqEn = FALSE;
    SPIConfig.bMasterMode = SpiMaster;
    SPIConfig.u8BaudRate = SpiClkDiv8;
    SPIConfig.pfnSpi0IrqCb = NULL;
	if(SPIConfig.bIrqEn)
	{
		EnableNvic(SPI0_IRQn,IrqLevel3,TRUE);
	}
    Spi_Init(Spi0,&SPIConfig);//模块初始化

    Spi_SetCS(Spi0,FALSE);//使能片选信号
    for(i=0;i<10;i++)
    {
      Spi_SendData(Spi0,u8Senddata[i]);//发送数据
    }
    i = Spi_ReceiveData(Spi0,1);// 接收推回数据AA
    for(i=0;i<10;i++)
    {
      u8Recvdata[i]=Spi_ReceiveData(Spi0,1);//主机接收数据
    }
    Spi_SetCS(Spi0,TRUE);
    while(1);
}

/******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/


