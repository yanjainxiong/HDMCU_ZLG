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
 **   - 2017-06-26 LiuHL    First Version
 **
 ******************************************************************************/

/******************************************************************************
 * Include files
 ******************************************************************************/
#include "adt.h"
#include "adc.h"
#include "gpio.h"
#include "flash.h"

/******************************************************************************
 * Local pre-processor symbols/macros ('#define')                            
 ******************************************************************************/


/******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/
uint16_t u16AdcResult00;
uint16_t u16AdcResult02;
uint16_t u16AdcResult05;

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

void Adt4UnderFullCalllback(void)
{
    //用户可在此处更改PWM的占空比 GCMCR寄存器
    //用户可在此处更改PWM的周期 PERBR寄存器
}

void AdcJQRIrqCallback(void)
{
    static uint8_t i;
    
    Adc_GetJqrResult(&u16AdcResult00, 0);
    Adc_GetJqrResult(&u16AdcResult02, 1);
    Adc_GetJqrResult(&u16AdcResult05, 2);
    
    //用户可在此处更改PWM的占空比 GCMCR寄存器，例如

    if(i%2 == 0)
    {
        Adt_SetCompareValue(AdtTIM4, AdtCompareC, 0x5555);
    }
    else
    {
        Adt_SetCompareValue(AdtTIM4, AdtCompareC, 0x9999);
    }
    
    i++;
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
    
    Gpio_SetAnalogMode(GpioPortA, GpioPin5);        //PA05
    
    Sysctrl_SetPeripheralGate(SysctrlPeripheralAdcBgr, TRUE);
    
    //ADC配置
    Adc_Enable();
    M0P_BGR->CR_f.BGR_EN = 0x1u;//BGR必须使能
    M0P_BGR->CR_f.TS_EN  = 0x0u;
    delay100us(10);
    
    stcAdcCfg.enAdcOpMode      = AdcSCanMode;           //连续采样模式
    stcAdcCfg.enAdcClkDiv      = AdcClkSysTDiv2;        //Adc工作时钟 PCLK/2
    stcAdcCfg.enAdcSampTimeSel = AdcSampTime6Clk;       //采样时钟 6个周期
    stcAdcCfg.enAdcRefVolSel   = RefVolSelAVDD;         //内部AVDD
    stcAdcCfg.bAdcInBufEn      = FALSE;
    
    Adc_Init(&stcAdcCfg);                               //Adc初始化
    
    Adc_ConfigJqrChannel(JQRCH0MUX, AdcExInputCH0);     //配置插队扫描转换通道
    Adc_ConfigJqrChannel(JQRCH1MUX, AdcExInputCH2);
    Adc_ConfigJqrChannel(JQRCH2MUX, AdcExInputCH5);     //ADC采样顺序CH2 --> CH1 --> CH0
    
    EnableNvic(ADC_IRQn, IrqLevel1, TRUE);              //Adc开中断
    Adc_EnableIrq();                                    //使能Adc中断
    
    stcAdcIrq.bAdcJQRIrq = TRUE;
    stcAdcIrqCalbaks.pfnAdcJQRIrq = AdcJQRIrqCallback;
    Adc_ConfigIrq(&stcAdcIrq, &stcAdcIrqCalbaks);       //中断函数入口配置
    
    u8AdcScanCnt = 3;                                   //转换次数3次(3-1已在库函数内计算)
    
    Adc_ConfigJqrMode(&stcAdcCfg, u8AdcScanCnt, FALSE); //配置插队扫描转换模式
    
    stcAdcExtTrigCfg.enAdcExtTrigRegSel = AdcExtTrig1;
    stcAdcExtTrigCfg.enAdcTrig1Sel      = AdcTrigTimer4;
    Adc_ExtTrigCfg(&stcAdcExtTrigCfg);                  //Timer4触发插队扫描转换
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
    uint16_t                  u16Compare;
    en_flash_waitcycle_t      enFlashWait;
    stc_sysctrl_clk_config_t  stcClkConfig;
    stc_adt_basecnt_cfg_t     stcAdtBaseCntCfg;
    stc_adt_CHxX_port_cfg_t   stcAdtTIM4ACfg;
    stc_adt_CHxX_port_cfg_t   stcAdtTIM4BCfg;
    stc_adt_validper_cfg_t    stcAdtValidPerCfg;
    stc_adt_irq_trig_cfg_t    stcAdtIrqTrigCfg;
    stc_adt_sw_sync_t         stcAdtSwSync;
    stc_gpio_config_t         stcTIM4Port;
    stc_gpio_config_t         stcTIM5Port;
    stc_gpio_config_t         stcTIM6Port;
    
    
    DDL_ZERO_STRUCT(stcClkConfig);
    DDL_ZERO_STRUCT(stcAdtBaseCntCfg);
    DDL_ZERO_STRUCT(stcAdtTIM4ACfg);
    DDL_ZERO_STRUCT(stcAdtTIM4BCfg);
    DDL_ZERO_STRUCT(stcAdtSwSync);
    DDL_ZERO_STRUCT(stcTIM4Port);
    DDL_ZERO_STRUCT(stcTIM5Port);
    DDL_ZERO_STRUCT(stcTIM6Port);
    DDL_ZERO_STRUCT(stcAdtValidPerCfg);
    DDL_ZERO_STRUCT(stcAdtIrqTrigCfg);
    
    enFlashWait = FlashWaitCycle1;                      //读等待周期设置为1（当HCLK大于24MHz时必须至少为1）
    Flash_WaitCycle(enFlashWait);                       // Flash 等待1个周期
    
    stcClkConfig.enClkSrc    = SysctrlClkXTH;           //使用外部高速晶振,32M
    stcClkConfig.enHClkDiv   = SysctrlHclkDiv1;         // HCLK = SystemClk/1
    stcClkConfig.enPClkDiv   = SysctrlPclkDiv1;         // PCLK = HCLK/1
    Sysctrl_SetXTHFreq(SysctrlXthFreq20_32MHz);         //设置外部高速频率为20~32M
    Sysctrl_ClkInit(&stcClkConfig);
    
    
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE); //端口外设时钟使能
       
    
    stcTIM4Port.enDir  = GpioDirOut;
    stcTIM5Port.enDir  = GpioDirOut;
    stcTIM6Port.enDir  = GpioDirOut;
    
    Gpio_Init(GpioPortA, GpioPin8, &stcTIM4Port);  
    Gpio_SetAfMode(GpioPortA,GpioPin8,GpioAf6);     //PA08设置为TIM4_CHA
    
    Gpio_Init(GpioPortA, GpioPin7, &stcTIM4Port);
    Gpio_SetAfMode(GpioPortA,GpioPin7,GpioAf7);     //PA07设置为TIM4_CHB
#if 0    
    Gpio_Init(GpioPortA, GpioPin4, &stcTIM5Port);
    Gpio_SetAfMode(GpioPortA,GpioPin4,GpioAf5);     //PA04设置为TIM5_CHA
    
    Gpio_Init(GpioPortA, GpioPin5, &stcTIM5Port);
    Gpio_SetAfMode(GpioPortA,GpioPin5,GpioAf5);     //PA05设置为TIM5_CHB

    Gpio_Init(GpioPortB, GpioPin11, &stcTIM6Port);
    Gpio_SetAfMode(GpioPortB,GpioPin11,GpioAf5);    //PB11设置为TIM6_CHA
    
    Gpio_Init(GpioPortB, GpioPin1, &stcTIM6Port);
    Gpio_SetAfMode(GpioPortB,GpioPin1,GpioAf4);     //PB01设置为TIM6_CHB

#endif
        
    if (Ok != Sysctrl_SetPeripheralGate(SysctrlPeripheralAdvTim, TRUE))//ADT外设时钟使能
    {
        return Error;
    }
    
    
    stcAdtBaseCntCfg.enCntMode = AdtTriangleModeA;           //三角波A
    stcAdtBaseCntCfg.enCntDir = AdtCntUp;
    stcAdtBaseCntCfg.enCntClkDiv = AdtClkPClk0;
    
    Adt_Init(AdtTIM4, &stcAdtBaseCntCfg);                      //ADT载波、计数模式、时钟配置
    
    u16Period = 0xEEEE;
    Adt_SetPeriod(AdtTIM4, u16Period);                         //周期设置
    Adt_SetPeriodBuf(AdtTIM4, u16Period);                      //周期缓存值设置并打开周期的缓存传送功能
    
    u16Compare = 0x9999;
    Adt_SetCompareValue(AdtTIM4, AdtCompareA, u16Compare);    //通用比较基准值寄存器A设置
    
    
    u16Compare = 0;
    Adt_SetCompareValue(AdtTIM4, AdtCompareB, u16Compare);    //通用比较基准值寄存器B设置

    
    u16Compare = 0x9999;
    Adt_SetCompareValue(AdtTIM4, AdtCompareC, u16Compare);    //通用比较基准值寄存器C设置

    Adt_EnableValueBuf(AdtTIM4, AdtCHxA, TRUE);
    //Adt_EnableValueBuf(AdtTIM4, AdtCHxB, TRUE);

    stcAdtTIM4ACfg.enCap = AdtCHxCompareOutput;            //比较输出
    stcAdtTIM4ACfg.bOutEn = TRUE;                          //CHA输出使能
    stcAdtTIM4ACfg.enPerc = AdtCHxPeriodKeep;              //计数值与周期匹配时CHA电平保持不变
    stcAdtTIM4ACfg.enCmpc = AdtCHxCompareInv;              //计数值与比较值A匹配时，CHA电平翻转
    stcAdtTIM4ACfg.enStaStp = AdtCHxStateSelSS;            //CHA起始结束电平由STACA与STPCA控制
    stcAdtTIM4ACfg.enStaOut = AdtCHxPortOutLow;            //CHA起始电平为低
    stcAdtTIM4ACfg.enStpOut = AdtCHxPortOutLow;            //CHA结束电平为低
    Adt_CHxXPortConfig(AdtTIM4, AdtCHxA, &stcAdtTIM4ACfg); //端口CHA配置
    
    stcAdtTIM4BCfg.enCap = AdtCHxCompareOutput;
    stcAdtTIM4BCfg.bOutEn = TRUE;
    stcAdtTIM4BCfg.enPerc = AdtCHxPeriodKeep;
    stcAdtTIM4BCfg.enCmpc = AdtCHxCompareInv;
    stcAdtTIM4BCfg.enStaStp = AdtCHxStateSelSS;
    stcAdtTIM4BCfg.enStaOut = AdtCHxPortOutHigh;            //CHA起始电平为高
    stcAdtTIM4BCfg.enStpOut = AdtCHxPortOutLow;
    Adt_CHxXPortConfig(AdtTIM4, AdtCHxB, &stcAdtTIM4BCfg);   //端口CHB配置

    Adt_SetDTUA(AdtTIM4, 0x2000);
    Adt_SetDTDA(AdtTIM4, 0x2000);
    Adt_ConfigDT(AdtTIM4, TRUE, TRUE);                       //死区配置

    Adt_ConfigIrq(AdtTIM4, AdtUDFIrq, TRUE, Adt4UnderFullCalllback);  //下溢中断配置
    
    stcAdtIrqTrigCfg.bAdtUnderFlowTrigEn = TRUE;
    Adt_IrqTrigConfig(AdtTIM4, &stcAdtIrqTrigCfg);           //下溢中断触发ADC转换
    
    stcAdtValidPerCfg.enValidCdt = AdtPeriodCnteMax;         //以三角波波峰为计数条件
    stcAdtValidPerCfg.enValidCnt = AdtPeriodCnts1;           //每隔1个周期有效
    Adt_SetValidPeriod(AdtTIM4, &stcAdtValidPerCfg);         //配置有效周期间隔寄存器
    
    ConfigAdc();
    
    stcAdtSwSync.bAdTim4 = TRUE;
    
    Adt_SwSyncStart(&stcAdtSwSync);
    
    while(1)
    {
        //用户可在此处更改PWM的占空比 GCMCR寄存器
    }
}

/******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/


