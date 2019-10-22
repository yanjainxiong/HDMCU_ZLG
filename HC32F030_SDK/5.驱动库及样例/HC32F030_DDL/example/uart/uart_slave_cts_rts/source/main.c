/******************************************************************************
* Copyright (C) 2017, Huada Semiconductor Co.,Ltd All rights reserved.
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
#include "uart.h"
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
/**
 ******************************************************************************
 ** \brief  Main function of project
 **
 ** \return uint32_t return value, if needed
 **
 ** This sample
 **
 ******************************************************************************/
#define SLAVEADDR   0xc0
uint8_t u8TxData[2] = {0xaa,0x55};
uint8_t u8RxData[2] = {0x00};
uint16_t u16AddrData = 0;
uint8_t u8TxCnt=0,u8RxCnt=0;
uint8_t u8AddrFlg = 0;

void TxIntCallback(void)
{

    if(u8TxCnt>1)
    {
      u8RxCnt = 0;
      Uart_DisableIrq(UARTCH1,UartTxIrq);
      Uart_EnableIrq(UARTCH1,UartRxIrq);
    }
    else
    {
        M0P_UART1->SBUF = u8TxData[u8TxCnt];
        u8TxCnt++;
    }
}
void RxIntCallback(void)
{
    if(0 == u8AddrFlg)
    {
        u8AddrFlg = 1;
        u16AddrData = M0P_UART1->SBUF;
        if(u16AddrData&0x100)
        {
            M0P_UART1->SCON_f.ADRDET = 0;
        }
    }
    else
    {
        u8RxData[u8RxCnt++] = Uart_ReceiveData(UARTCH1);
    }
}
void ErrIntCallback(void)
{
  
}
void PErrIntCallBack(void)
{
}
void CtsIntCallBack(void)
{
  
}
void Uart_PortInit(void)
{
    stc_gpio_config_t stcGpioCfg;
    DDL_ZERO_STRUCT(stcGpioCfg);
    
    stcGpioCfg.enDir = GpioDirOut;
    Gpio_Init(GpioPortA,GpioPin2,&stcGpioCfg);
    Gpio_SetAfMode(GpioPortA,GpioPin2,GpioAf1);//TX
    
    Gpio_Init(GpioPortC,GpioPin1,&stcGpioCfg);
    Gpio_SetAfMode(GpioPortC,GpioPin1,GpioAf3);//RTS
    
    stcGpioCfg.enDir = GpioDirIn;
    Gpio_Init(GpioPortA,GpioPin3,&stcGpioCfg);
    Gpio_SetAfMode(GpioPortA,GpioPin3,GpioAf1);//RX
    
    Gpio_Init(GpioPortC,GpioPin0,&stcGpioCfg);
    Gpio_SetAfMode(GpioPortC,GpioPin0,GpioAf3);//CTS

}
int32_t main(void)
{  
    uint16_t u16Scnt = 0;
    stc_uart_config_t  stcConfig;
    stc_uart_irq_cb_t stcUartIrqCb;
    stc_uart_multimode_t stcMulti;
    stc_uart_baud_t stcBaud;
    
    en_uart_mmdorck_t enTb8;

    DDL_ZERO_STRUCT(stcConfig);
    DDL_ZERO_STRUCT(stcUartIrqCb);
    DDL_ZERO_STRUCT(stcMulti);
    DDL_ZERO_STRUCT(stcBaud);
    
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio,TRUE);
    Sysctrl_SetPeripheralGate(SysctrlPeripheralUart1,TRUE);
    Uart_PortInit();


    stcUartIrqCb.pfnRxIrqCb   = RxIntCallback;
    stcUartIrqCb.pfnTxIrqCb   = TxIntCallback;
    stcUartIrqCb.pfnRxFEIrqCb = ErrIntCallback;
    stcUartIrqCb.pfnPEIrqCb   = PErrIntCallBack;
    stcUartIrqCb.pfnCtsIrqCb  = CtsIntCallBack;
    stcConfig.pstcIrqCb = &stcUartIrqCb;
    stcConfig.bTouchNvic = TRUE;
  

    stcConfig.enRunMode = UartMode3;//4种模式
    stcConfig.enStopBit = Uart1bit;  

    stcMulti.enMulti_mode = UartMulti;//测试项，更改此处来转换多机模式，mode2/3才有多机模式
    stcMulti.u8SaddEn = 0xff;//从机地址掩码
    stcMulti.u8SlaveAddr = SLAVEADDR;//从机地址
	Uart_SetMultiMode(UARTCH1,&stcMulti);//多主机单独配置
    Uart_Init(UARTCH1, &stcConfig);//串口初始化函数
    
    Uart_SetClkDiv(UARTCH1,Uart8Or16Div);//采样分频
    stcBaud.u32Pclk = Sysctrl_GetPClkFreq();
    stcBaud.enRunMode = UartMode3;
    stcBaud.u32Baud = 9600;
    u16Scnt = Uart_CalScnt(UARTCH1,&stcBaud);//波特率值计算
    Uart_SetBaud(UARTCH1,u16Scnt);//波特率设置
    
    Uart_ClrIsr(UARTCH1);//清中断请求
    Uart_EnableIrq(UARTCH1,UartRxIrq);//使能接收中断
    Uart_EnableFunc(UARTCH1,UartRx);//使能串口收发
    Uart_EnableFunc(UARTCH1,UartCtsRts);//使能RTS
    
    enTb8 = UartDataOrAddr;
    Uart_SetMMDOrCk(UARTCH1,enTb8);         
    while(1)
    {
        if(u8RxCnt>1)
        {
            u8RxCnt = 0;
            u8TxCnt = 0;
            Uart_EnableIrq(UARTCH1,UartTxIrq);
            M0P_UART1->SBUF = u8TxData[u8TxCnt];
            u8TxCnt++;
        }
    }
}

/******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/


