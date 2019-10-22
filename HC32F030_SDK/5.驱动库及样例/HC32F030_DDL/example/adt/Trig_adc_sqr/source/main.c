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
 **   - 2017-05-28 LiuHL    First Version
 **
 ******************************************************************************/

/******************************************************************************
 * Include files
 ******************************************************************************/
#include "adt.h"
#include "adc.h"
#include "gpio.h"

/******************************************************************************
 * Local pre-processor symbols/macros ('#define')                            
 ******************************************************************************/


/******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/
uint16_t u16AdcRestult0;
uint16_t u16AdcRestult2;
   
/******************************************************************************
 * Local type definitions ('typedef')                                         
 ******************************************************************************/

/******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/

/******************************************************************************
 * Local variable definitions ('static')                                      *
 ******************************************************************************/



/*****************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/

void AdcContIrqCallback(void)
{    
    Adc_GetSqrResult(&u16AdcRestult0, 0);
    Adc_GetSqrResult(&u16AdcRestult2, 1);

}

void ConfigAdc(void)
{
    uint8_t                    u8AdcScanCnt;
    stc_adc_cfg_t              stcAdcCfg;
    stc_adc_irq_t              stcAdcIrq;
    stc_adc_irq_calbakfn_pt_t  stcAdcIrqCalbaks;
    stc_adc_ext_trig_cfg_t     stcAdcExtTrigCfg;

    
    DDL_ZERO_STRUCT(stcAdcCfg);
    DDL_ZERO_STRUCT(stcAdcIrq);
    DDL_ZERO_STRUCT(stcAdcIrqCalbaks);
    DDL_ZERO_STRUCT(stcAdcExtTrigCfg);
    
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE);
    
    Gpio_SetAnalogMode(GpioPortA, GpioPin0);        //PA00
        
    Gpio_SetAnalogMode(GpioPortA, GpioPin2);        //PA02
    
    Sysctrl_SetPeripheralGate(SysctrlPeripheralAdcBgr, TRUE);
    
    //ADC配置
    Adc_Enable();
    M0P_BGR->CR_f.BGR_EN = 0x1u;  //BGR必须使能
    M0P_BGR->CR_f.TS_EN  = 0x0u;
    delay100us(10);
    
    stcAdcCfg.enAdcOpMode      = AdcSCanMode;           //连续采样模式
    stcAdcCfg.enAdcClkDiv      = AdcClkSysTDiv1;        //Adc工作时钟 PCLK
    stcAdcCfg.enAdcSampTimeSel = AdcSampTime8Clk;       //采样时钟 6个周期
    stcAdcCfg.enAdcRefVolSel   = RefVolSelAVDD;         //内部AVDD
    stcAdcCfg.bAdcInBufEn      = FALSE;
    
    Adc_Init(&stcAdcCfg);                               //Adc初始化
    
    Adc_ConfigSqrChannel(CH0MUX, AdcExInputCH0);        //配置顺序扫描转换通道
    Adc_ConfigSqrChannel(CH1MUX, AdcExInputCH2);        //扫描顺序CH1 --> CH0
    
    EnableNvic(ADC_IRQn, IrqLevel1, TRUE);              //Adc开中断
    
    Adc_EnableIrq();                                    //使能Adc中断
    
    stcAdcIrq.bAdcSQRIrq = TRUE;
    stcAdcIrqCalbaks.pfnAdcSQRIrq = AdcContIrqCallback;
    Adc_ConfigIrq(&stcAdcIrq, &stcAdcIrqCalbaks);       //中断函数入口配置
    
    u8AdcScanCnt = 2;                                   //转换次数2次(2-1已在库函数内计算)
    
    Adc_ConfigSqrMode(&stcAdcCfg, u8AdcScanCnt, FALSE); //配置顺序扫描转换模式
    
    stcAdcExtTrigCfg.enAdcExtTrigRegSel = AdcExtTrig0;
    stcAdcExtTrigCfg.enAdcTrig0Sel      = AdcTrigTimer4;
    Adc_ExtTrigCfg(&stcAdcExtTrigCfg);                  //Timer4触发顺序扫描转换
}

/**
 ******************************************************************************
 ** \brief  Main function of project
 **
 ** \return uint32_t return value, if needed
 **
 ** This sample
 **
 ******************************************************************************/
int32_t main(void)
{
    uint16_t                  u16Period;
    stc_adt_basecnt_cfg_t     stcAdtBaseCntCfg;
    stc_adt_irq_trig_cfg_t    stcAdtIrqTrigCfg;
    
    DDL_ZERO_STRUCT(stcAdtBaseCntCfg);
    DDL_ZERO_STRUCT(stcAdtIrqTrigCfg);
    
    Sysctrl_SetRCLTrim(SysctrlRclFreq32768);
    Sysctrl_ClkSourceEnable(SysctrlClkRCL, TRUE);
    Sysctrl_SysClkSwitch(SysctrlClkRCL);
    
    ///< 加载目标频率的RCH的TRIM值
    Sysctrl_SetRCHTrim(SysctrlRchFreq24MHz);
    ///< 使能RCH（默认打开，此处可不需要再次打开）
    //Sysctrl_ClkSourceEnable(SysctrlClkRCH, TRUE);
    ///< 时钟切换到RCH
    Sysctrl_SysClkSwitch(SysctrlClkRCH);
    ///< 关闭RCL时钟
    Sysctrl_ClkSourceEnable(SysctrlClkRCL, FALSE);
    
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE);    //端口外设时钟使能
    Sysctrl_SetPeripheralGate(SysctrlPeripheralAdvTim, TRUE);  //ADT外设时钟使能
    

    
    //ADT Timer4触发ADC采样，Frequency: 50K
    stcAdtBaseCntCfg.enCntMode = AdtSawtoothMode;              //锯齿波
    stcAdtBaseCntCfg.enCntDir = AdtCntUp;
    stcAdtBaseCntCfg.enCntClkDiv = AdtClkPClk0;                //PCLK
    
    Adt_Init(AdtTIM4, &stcAdtBaseCntCfg);                      //ADT载波、计数模式、时钟配置
    
    u16Period = 480;                                           //50K: 24M * 20us = 480
    Adt_SetPeriod(AdtTIM4, u16Period);                         //周期设置

    stcAdtIrqTrigCfg.bAdtOverFlowTrigEn = TRUE;
    Adt_IrqTrigConfig(AdtTIM4, &stcAdtIrqTrigCfg);            //上溢中断触发ADC转换
    
    ConfigAdc();
    
    Adt_StartCount(AdtTIM4);                                  //timer4启动
    
    while(1)
    {
        //用户可在此处更改PWM的占空比 GCMCR寄存器
    }
}

/******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/


