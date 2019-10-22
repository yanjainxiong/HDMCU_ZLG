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
**   - 2018-03-09  1.0  Lux First version
**
******************************************************************************/

/******************************************************************************
* Include files
******************************************************************************/
#include "ddl.h"
#include "dmac.h"
#include "adc.h"
#include "gpio.h"
#include "bgr.h"
/******************************************************************************
* Local pre-processor symbols/macros ('#define')                            
******************************************************************************/

/******************************************************************************
* Global variable definitions (declared in header file with 'extern')
******************************************************************************/
uint16_t u16AdcRestult0;
uint16_t u16AdcRestult2;
uint16_t u16AdcRestult5;

static stc_adc_irq_t stcAdcIrqFlag;
/******************************************************************************
* Local type definitions ('typedef')                                         
******************************************************************************/

/******************************************************************************
* Local function prototypes ('static')
******************************************************************************/
void AdcContIrqCallback(void)
{    
    Adc_GetSqrResult(&u16AdcRestult0, 0);
    Adc_GetSqrResult(&u16AdcRestult2, 1);
    Adc_GetSqrResult(&u16AdcRestult5, 2);
  
    stcAdcIrqFlag.bAdcSQRIrq = TRUE;
}
/******************************************************************************
* Local variable definitions ('static')                                      *
******************************************************************************/
static uint32_t ADC_Result_Array[16] = {0};
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
******************************************************************************/
int32_t main(void)
{
   
  uint8_t                    u8AdcScanCnt;
  stc_adc_cfg_t              stcAdcCfg;
  stc_adc_irq_t              stcAdcIrq;
  stc_adc_irq_calbakfn_pt_t  stcAdcIrqCalbaks;
  stc_gpio_config_t          stcAdcAN0Port;
  stc_gpio_config_t          stcAdcAN2Port;
  stc_gpio_config_t          stcAdcAN5Port;
  stc_dma_config_t           stcDmaCfg;    
  
  DDL_ZERO_STRUCT(stcAdcCfg);
  DDL_ZERO_STRUCT(stcAdcIrq);
  DDL_ZERO_STRUCT(stcAdcIrqCalbaks);
  DDL_ZERO_STRUCT(stcAdcIrqFlag);
  DDL_ZERO_STRUCT(stcAdcAN0Port);
  DDL_ZERO_STRUCT(stcAdcAN2Port);
  DDL_ZERO_STRUCT(stcAdcAN5Port); 
  DDL_ZERO_STRUCT(stcDmaCfg);   
  
  
  
  
  // 使能GPIO，ADC, DMA时钟
  Sysctrl_SetPeripheralGate(SysctrlPeripheralDma,TRUE);
  Sysctrl_SetPeripheralGate(SysctrlPeripheralAdcBgr,TRUE);  
  Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio,TRUE); 
  
  
  if (Ok != Sysctrl_SetPeripheralGate(SysctrlPeripheralAdcBgr, TRUE))
  {
    return 1;
  }	
  
  Gpio_SetAnalogMode(GpioPortA, GpioPin0);        //PA00 (AIN0)
  stcAdcAN0Port.enDir = GpioDirIn;
  Gpio_Init(GpioPortA, GpioPin0, &stcAdcAN0Port);
  
  
  Gpio_SetAnalogMode(GpioPortA, GpioPin2);        //PA02 (AIN2)
  stcAdcAN2Port.enDir = GpioDirIn;
  Gpio_Init(GpioPortA, GpioPin2, &stcAdcAN2Port);
  
  
  
  Gpio_SetAnalogMode(GpioPortA, GpioPin5);        //PA05 (AIN5)
  stcAdcAN5Port.enDir = GpioDirIn;
  Gpio_Init(GpioPortA, GpioPin5, &stcAdcAN5Port);
  
  Adc_Enable();
  //使能BGR
  Bgr_BgrEnable();
  delay100us(1);			
  stcAdcCfg.enAdcOpMode = AdcSCanMode;         //扫描模式
  stcAdcCfg.enAdcClkDiv = AdcClkSysTDiv1;
  stcAdcCfg.enAdcSampTimeSel = AdcSampTime8Clk;
  stcAdcCfg.enAdcRefVolSel   = RefVolSelAVDD;
  stcAdcCfg.bAdcInBufEn      = FALSE;
  
  Adc_Init(&stcAdcCfg);
  
  Adc_ConfigSqrChannel(CH0MUX, AdcExInputCH0);
  Adc_ConfigSqrChannel(CH1MUX, AdcExInputCH2);
  Adc_ConfigSqrChannel(CH2MUX, AdcExInputCH5);
  Adc_ConfigSqrChannel(CH4MUX, AdcExInputCH0);
  Adc_ConfigSqrChannel(CH5MUX, AdcExInputCH2);
  Adc_ConfigSqrChannel(CH6MUX, AdcExInputCH5);
  Adc_ConfigSqrChannel(CH7MUX, AdcExInputCH0);
  Adc_ConfigSqrChannel(CH8MUX, AdcExInputCH2);
  Adc_ConfigSqrChannel(CH9MUX, AdcExInputCH5);
  
  EnableNvic(ADC_IRQn, IrqLevel3, TRUE);
  
  Adc_EnableIrq();
  
  stcAdcIrq.bAdcSQRIrq = TRUE;
  stcAdcIrqCalbaks.pfnAdcSQRIrq = AdcContIrqCallback;
  
  Adc_ConfigIrq(&stcAdcIrq, &stcAdcIrqCalbaks);
  
  u8AdcScanCnt = 9;
  
  Adc_ConfigSqrMode(&stcAdcCfg, u8AdcScanCnt, FALSE);
  
  //   Adc_SQR_Start();
  
  //使能DMA触发
  Adc_ConfigDmaTrig(DmaSqr);
  
  stcDmaCfg.enMode =  DmaBlock;                   
  stcDmaCfg.u16BlockSize = 0x03u;
  stcDmaCfg.u16TransferCnt = 0x03u;         //Block模式，一次传输数据大小为 1,传输三次
  stcDmaCfg.enTransferWidth = Dma32Bit;            
  stcDmaCfg.enSrcAddrMode = AddressIncrease;
  stcDmaCfg.enDstAddrMode = AddressIncrease;
  stcDmaCfg.bDestAddrReloadCtl = FALSE;
  stcDmaCfg.bSrcAddrReloadCtl = TRUE;
  stcDmaCfg.bSrcBcTcReloadCtl = TRUE;
  stcDmaCfg.u32SrcAddress = (uint32_t) &(M0P_ADC->SQRRESULT0);
  stcDmaCfg.u32DstAddress = (uint32_t)&ADC_Result_Array[0];
  stcDmaCfg.bMsk = TRUE;                                         //DMAC 在传输完成时不清除 CONFA:ENS 位。这个功能允许连续传输而不需要 CPU 干预。
  stcDmaCfg.enRequestNum = ADCSQRTrig;	                         //设置为ADC SQR触发
  
  Dma_InitChannel(DmaCh0,&stcDmaCfg);	
  
  //使能DMA，使能DMA0，启动DMA0
  Dma_Enable();
  Dma_EnableChannel(DmaCh0);
  Dma_Start(DmaCh0);	
  
  Adc_SQR_Start();
  delay1ms(500);
  Adc_SQR_Start();
  delay1ms(500);
  Adc_SQR_Start();//TC = 3,需触发三次

  //等待传输完成
  while(Dma_GetStat(DmaCh0) != DmaTransferComplete);
  
  while (1)
  {
    ;
  }
}

/******************************************************************************
* EOF (not truncated)
******************************************************************************/


