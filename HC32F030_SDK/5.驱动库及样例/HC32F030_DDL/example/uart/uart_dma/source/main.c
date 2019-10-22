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
 **   - 2017-05-17  1.0  cj First version for Device Driver Library of Module.
 **
 ******************************************************************************/

/******************************************************************************
 * Include files
 ******************************************************************************/
#include "ddl.h"
#include "uart.h"
#include "gpio.h"
#include "sysctrl.h"
#include "dmac.h"

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
uint8_t u8TxData[8] = {0xAA,0x55};
uint8_t u8RxData[8]={0X00};
uint8_t u8TxCnt=0,u8RxCnt=0;

void TxIntCallback(void)
{
    u8TxCnt++;
    if(u8TxCnt<=1)
        M0P_UART1->SBUF = u8TxData[1];
    
     
}
void RxIntCallback(void)
{
   // u8TxData[0]=Uart_ReceiveData(UARTCH1);
    u8RxCnt++;
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
    stcGpioCfg.enDir = GpioDirIn;
    Gpio_Init(GpioPortA,GpioPin3,&stcGpioCfg);
    Gpio_SetAfMode(GpioPortA,GpioPin3,GpioAf1);//RX
}
uint8_t cnt=0;
int32_t main(void)
{  
    uint16_t u16Scnt = 0;
    stc_uart_config_t  stcConfig;
    stc_uart_irq_cb_t stcUartIrqCb;
    stc_uart_multimode_t stcMulti;
    stc_uart_baud_t stcBaud;
	stc_dma_config_t stcDmaConfig;
    
    en_uart_mmdorck_t enTb8;

    DDL_ZERO_STRUCT(stcConfig);
    DDL_ZERO_STRUCT(stcUartIrqCb);
    DDL_ZERO_STRUCT(stcMulti);
    DDL_ZERO_STRUCT(stcBaud);
    DDL_ZERO_STRUCT(stcDmaConfig);
	
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio,TRUE);
    Sysctrl_SetPeripheralGate(SysctrlPeripheralDma,TRUE);
    Sysctrl_SetPeripheralGate(SysctrlPeripheralUart1,TRUE);
    Uart_PortInit();//端口初始化
    
    stcUartIrqCb.pfnRxIrqCb   = RxIntCallback;//中断服务函数入口地址设定
    stcUartIrqCb.pfnTxIrqCb   = TxIntCallback;
    stcUartIrqCb.pfnRxFEIrqCb = ErrIntCallback;
    stcUartIrqCb.pfnPEIrqCb   = PErrIntCallBack;
    stcUartIrqCb.pfnCtsIrqCb  = CtsIntCallBack;
    stcConfig.pstcIrqCb = &stcUartIrqCb;
    stcConfig.bTouchNvic = FALSE;
	if(TRUE == stcConfig.bTouchNvic)
	{
		EnableNvic(UART1_IRQn,IrqLevel3,TRUE);
	}
    stcConfig.enRunMode = UartMode3;//模式3
    stcConfig.enStopBit = Uart1bit;  //1位停止位

    stcMulti.enMulti_mode = UartNormal;//正常工作模式
	Uart_SetMultiMode(UARTCH1,&stcMulti);//多主机单独配置
    enTb8 = UartEven;//偶校验
    Uart_SetMMDOrCk(UARTCH1,enTb8);
      
    Uart_Init(UARTCH1, &stcConfig);//串口初始化函数
    
    Uart_SetClkDiv(UARTCH1,Uart8Or16Div);//采样分频
    stcBaud.u32Pclk = Sysctrl_GetPClkFreq();
    stcBaud.enRunMode = UartMode3;
    stcBaud.u32Baud = 9600;
    u16Scnt = Uart_CalScnt(UARTCH1,&stcBaud);//波特率值计算
    Uart_SetBaud(UARTCH1,u16Scnt);//波特率设置
    
    Uart_ClrStatus(UARTCH1,UartRC);//清接收请求
    Uart_EnableFunc(UARTCH1,UartRx);//使能收发
	Uart_EnableFunc(UARTCH1,UartDmaTx);//使能DMA发送
	Uart_EnableFunc(UARTCH1,UartDmaRx);//使能DMA接收
	//rx dma配置
	stcDmaConfig.u32SrcAddress = 0x40000100;//接收数据寄存器地址
	stcDmaConfig.u32DstAddress = (uint32_t)&u8RxData[0];//接收缓冲
	stcDmaConfig.bSrcAddrReloadCtl = TRUE;
	stcDmaConfig.bSrcBcTcReloadCtl = TRUE;
	stcDmaConfig.bDestAddrReloadCtl = FALSE;
	stcDmaConfig.bMsk = TRUE;
	stcDmaConfig.enDstAddrMode = AddressIncrease;
	stcDmaConfig.enSrcAddrMode = AddressFix;
	stcDmaConfig.u16BlockSize = 1;
	stcDmaConfig.u16TransferCnt = 1;
	stcDmaConfig.enMode = DmaBlock;
	stcDmaConfig.enTransferWidth = Dma8Bit;
	stcDmaConfig.enRequestNum = Uart1RxTrig;
	Dma_Enable();	
	Dma_InitChannel(DmaCh0, &stcDmaConfig);
	
	DDL_ZERO_STRUCT(stcDmaConfig);
	
	//tx
	stcDmaConfig.u32SrcAddress = (uint32_t)&u8RxData[0];
	stcDmaConfig.u32DstAddress = 0x40000100;
	stcDmaConfig.bSrcAddrReloadCtl = TRUE;
	stcDmaConfig.bSrcBcTcReloadCtl = TRUE;
	stcDmaConfig.bDestAddrReloadCtl = TRUE;
	stcDmaConfig.bMsk = FALSE;
	stcDmaConfig.enDstAddrMode = AddressFix;
	stcDmaConfig.enSrcAddrMode = AddressIncrease;
	stcDmaConfig.u16BlockSize = 1;
	stcDmaConfig.u16TransferCnt = 2;
	stcDmaConfig.enMode = DmaBlock;
	stcDmaConfig.enTransferWidth = Dma8Bit;
	stcDmaConfig.enRequestNum = Uart1TxTrig;

	Dma_InitChannel(DmaCh1, &stcDmaConfig);
	Dma_EnableChannel(DmaCh0);
	
    while(1)
    {
		if(5 == Dma_GetStat(DmaCh0))
		{
			cnt++;
			if(cnt>1)
			{
				cnt = 0;
				Dma_DisableChannel(DmaCh0);
				delay10us(100);
				Dma_EnableChannel(DmaCh1);
			}
			M0P_DMAC->CONFB0_f.STAT = 0;	
		} 
		if(5 == Dma_GetStat(DmaCh1))
		{
			Dma_DisableChannel(DmaCh1);
			Dma_EnableChannel(DmaCh0);
			M0P_DMAC->CONFB1_f.STAT = 0;	
		} 	

    }
}

/******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/


