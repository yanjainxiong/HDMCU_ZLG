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
#define     T1_PORT                 (3)
#define     T1_PIN                  (2)
/*****************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/
uint8_t bIrq,bIrqData,Buff[20]; 

/**
 ******************************************************************************
 ** \brief  Main function of project
 **
 ** \return uint32_t return value, if needed
 **
 ** This sample
 **
 ******************************************************************************/

boolean_t bSendIrq,bRecvIrq;
uint8_t u8IrqData,Buff[20];
uint8_t u8SendData;
static void Spi0CallBack(void)
{
    
    if(Spi_GetState(Spi0)&0x38)
    {
        Spi_ClearStatus(Spi0);
        return;
    }
    if(Spi_GetStatus(Spi0,SpiRxne))
    {
      bRecvIrq = 1;
      Spi_FuncEn(Spi0,SpiRxNeIe,FALSE);
      u8IrqData = M0P_SPI0->DATA;
      Spi_ClearStatus(Spi0);
    }
    else if(Spi_GetStatus(Spi0,SpiTxe))
    {
      bSendIrq = 1;
      Spi_FuncEn(Spi0,SpiTxEIe,FALSE);
      M0P_SPI0->DATA = u8SendData;
      Spi_ClearStatus(Spi0);
    }
}

void WriteData(en_spi_channel_t enCh,uint8_t *sendstr,uint8_t sendlen)
{
    uint8_t i;
    bSendIrq = 0;
	bRecvIrq = 0;
    for(i=0;i<sendlen;i++)
    {
        u8SendData = *(sendstr + i);
        Spi_FuncEn(Spi0,SpiRxNeIe,TRUE);
        Spi_FuncEn(Spi0,SpiTxEIe,TRUE);
		while(0 == bSendIrq);
		bSendIrq = 0;
		while(0 == bRecvIrq);
		bRecvIrq = 0;
    }
}
void ReadData(en_spi_channel_t enCh,uint8_t *recvstr,uint8_t recvlen)
{
    uint8_t i;
    uint16_t u32TimeOut;

    bRecvIrq = 0;

    for(i=0;i<recvlen;i++)
    {
        Spi_FuncEn(Spi0,SpiRxNeIe,TRUE);
        u32TimeOut = 1000;   
        M0P_SPI0->DATA = 0; //接收数据时，用于主机产生时钟   
        while(u32TimeOut--)
        {
             if(1 == bRecvIrq)
            {
                *(recvstr+i) = u8IrqData;
                break;
            }
        }
        bRecvIrq = 0;
    }
}
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
//字节间距76.8us
int32_t main(void)
{
    uint8_t   i,j;
    stc_gpio_config_t stcGpioCfg;
    stc_spi_config_t  SPIConfig;
    
    DDL_ZERO_STRUCT(stcGpioCfg);
    DDL_ZERO_STRUCT(SPIConfig);
    
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio,TRUE);
    Sysctrl_SetPeripheralGate(SysctrlPeripheralSpi0,TRUE); //SPI0
    Spi_PortInit();
    stcGpioCfg.enDir = GpioDirOut;   
    Gpio_Init(GpioPortA, GpioPin0,&stcGpioCfg);//
    Gpio_WriteOutputIO(GpioPortA, GpioPin0,0);
       
    Spi_SetCS(Spi0,TRUE);
    //SPI模块配置
    SPIConfig.bCPHA = Spicphafirst;//模式0
    SPIConfig.bCPOL = Spicpollow;
    SPIConfig.bIrqEn = TRUE;
    SPIConfig.bMasterMode = SpiMaster;//主机
    SPIConfig.u8BaudRate = SpiClkDiv2;
    SPIConfig.pfnSpi0IrqCb = Spi0CallBack;

    Spi_Init(Spi0,&SPIConfig);

    for(i=3;i<8;i++)
    {
        Buff[i] = i+0x10;
    }
    
    Buff[0] = 0x06;
    Spi_SetCS(Spi0,FALSE);
    WriteData(Spi0,&Buff[0],1);
    Spi_SetCS(Spi0,TRUE);
    
    Buff[0] = 0x02;        //
    Buff[1] = 0x00;        //
    Buff[2] = 0x00;
    Spi_SetCS(Spi0,FALSE);
    WriteData(Spi0,&Buff[0],8); //
    Spi_SetCS(Spi0,TRUE);

    delay1ms(2000);     
    
    Buff[0] = 0x03;        //
    Buff[1] = 0x00;        //
    Buff[2] = 0x00;
    Spi_SetCS(Spi0,FALSE);
    WriteData(Spi0,&Buff[0],3); //
    ReadData(Spi0,&Buff[13],5); //
    Spi_SetCS(Spi0,TRUE);
    
    j=0;
    for(i=3;i<8;i++)   
    {
        if(Buff[10+i] == Buff[i])
        {
            j++;
        }
    }
    if(j == 5)
    {
       Gpio_WriteOutputIO(GpioPortA, GpioPin0,1);//读写一致时，PA00置高
    }

    while(1);
}

/******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/


