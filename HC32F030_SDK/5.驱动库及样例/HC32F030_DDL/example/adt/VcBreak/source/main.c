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
 **   - 2017-06-20 LiuHL    First Version
 **
 ******************************************************************************/

/******************************************************************************
 * Include files
 ******************************************************************************/
#include "adt.h"
#include "vc.h"
#include "gpio.h"
#include "flash.h"

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


/*****************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/

void VcIrqCallback(void)
{
    //使用时需要用户添加VC刹车处理代码，退出中断后VC刹车即无效;
  delay1ms(1000);    //延时仅作为上升沿或下降沿刹车时的效果演示，
}

void ConfigVc(void)
{
    stc_gpio_config_t        stcVC1PInPort;
    stc_gpio_config_t        stcVC1NInPort;
    stc_gpio_config_t        stcVC1OutPort;
    stc_vc_channel_config_t  stcChannelConfig;
    
    DDL_ZERO_STRUCT(stcChannelConfig);
    DDL_ZERO_STRUCT(stcVC1PInPort);
    DDL_ZERO_STRUCT(stcVC1NInPort);
    DDL_ZERO_STRUCT(stcVC1OutPort);
    
    Sysctrl_SetPeripheralGate(SysctrlPeripheralVcLvd, TRUE);
        
    //PB14设置为VC1_IN_P
    //Gpio_ClrAnalogMode(GpioPortB, GpioPin14);
    stcVC1PInPort.enDir  = GpioDirIn;
    Gpio_Init(GpioPortB, GpioPin14, &stcVC1PInPort);
    
    //PA00设置为VC1_IN_N
    //Gpio_ClrAnalogMode(GpioPortA, GpioPin0);
    stcVC1NInPort.enDir  = GpioDirIn;
    Gpio_Init(GpioPortA, GpioPin0, &stcVC1NInPort);
    
    //PA02设置为VC1_OUT
    //Gpio_ClrAnalogMode(GpioPortA, GpioPin2);
    stcVC1OutPort.enDir  = GpioDirOut;
    Gpio_Init(GpioPortA, GpioPin2, &stcVC1OutPort);
    Gpio_SetAfMode(GpioPortA,GpioPin2,GpioAf3);

    
    stcChannelConfig.enVcChannel      = VcChannel1;    //通道1
    stcChannelConfig.enVcCmpDly       = VcDelayoff;    //迟滞关闭
    stcChannelConfig.enVcBiasCurrent  = VcBias1200na;  //偏置电流1.2ua
    stcChannelConfig.enVcFilterTime   = VcFilter14us;  //输出滤波时间14us
    stcChannelConfig.enVcInPin_P      = VcInPCh11;     //VC1_IN_P 选择PB14
    stcChannelConfig.enVcInPin_N      = VcInNCh4;      //VC1_IN_N 选择PA00
    stcChannelConfig.enVcOutConfig    = VcOutBrake;    //结果使能输出到Advanced Timer刹车
    stcChannelConfig.enVcIrqSel       = VcIrqHigh;     //高电平触发中断

    stcChannelConfig.pfnAnalogCmpCb   = VcIrqCallback;
    
    Vc_ChannelInit(VcChannel1, &stcChannelConfig);     //VC1初始化
    
    Vc_EnableFilter(VcChannel1);                       //滤波使能
    
    Vc_EnableIrq(VcChannel1);                          //
    
    Vc_EnableChannel(VcChannel1);                      //VC1使能
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
    en_adt_unit_t            enAdt;
    uint16_t                 u16Period;
    en_adt_compare_t         enAdtCompareA;
    uint16_t                 u16CompareA;
    en_adt_compare_t         enAdtCompareB;
    uint16_t                 u16CompareB;
    en_flash_waitcycle_t     enFlashWait;
    stc_sysctrl_clk_config_t stcClkConfig;
    stc_adt_basecnt_cfg_t    stcAdtBaseCntCfg;
    stc_adt_CHxX_port_cfg_t  stcAdtTIM4ACfg;
    stc_adt_CHxX_port_cfg_t  stcAdtTIM4BCfg;
    stc_gpio_config_t        stcTIM4APort;
    stc_gpio_config_t        stcTIM4BPort;
    
    DDL_ZERO_STRUCT(stcClkConfig);
    DDL_ZERO_STRUCT(stcAdtBaseCntCfg);
    DDL_ZERO_STRUCT(stcAdtTIM4ACfg);
    DDL_ZERO_STRUCT(stcAdtTIM4BCfg);
    DDL_ZERO_STRUCT(stcTIM4APort);
    DDL_ZERO_STRUCT(stcTIM4BPort);
    
    enFlashWait = FlashWaitCycle1;                      //读等待周期设置为1（当HCLK大于24MHz时必须至少为1）
    Flash_WaitCycle(enFlashWait);                       // Flash 等待1个周期
    
    stcClkConfig.enClkSrc    = SysctrlClkXTH;
    stcClkConfig.enHClkDiv   = SysctrlHclkDiv1;
    stcClkConfig.enPClkDiv   = SysctrlPclkDiv1;
    Sysctrl_SetXTHFreq(SysctrlXthFreq20_32MHz);
    Sysctrl_ClkInit(&stcClkConfig);
    
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE); //端口外设时钟使能

    
    //PA08设置为TIM4_CHA
    //Gpio_ClrAnalogMode(GpioPortA, GpioPin8);
    stcTIM4APort.enDir  = GpioDirOut;
    Gpio_Init(GpioPortA, GpioPin8, &stcTIM4APort);
    Gpio_SetAfMode(GpioPortA,GpioPin8,GpioAf6);
    
    //PA07设置为TIM4_CHB
    //Gpio_ClrAnalogMode(GpioPortA, GpioPin7);
    stcTIM4BPort.enDir  = GpioDirOut;
    Gpio_Init(GpioPortA, GpioPin7, &stcTIM4BPort);
    Gpio_SetAfMode(GpioPortA,GpioPin7,GpioAf7);

    if (Ok != Sysctrl_SetPeripheralGate(SysctrlPeripheralAdvTim, TRUE))//ADT外设时钟使能
    {
        return Error;
    }
    
    enAdt = AdtTIM4;

    stcAdtBaseCntCfg.enCntMode = AdtTriangleModeA;
    stcAdtBaseCntCfg.enCntDir = AdtCntUp;
    stcAdtBaseCntCfg.enCntClkDiv = AdtClkPClk0Div4;
    Adt_Init(enAdt, &stcAdtBaseCntCfg);                      //ADT载波、计数模式、时钟配置
    
    u16Period = 0xC000;
    Adt_SetPeriod(enAdt, u16Period);                         //周期设置
    
    enAdtCompareA = AdtCompareA;
    u16CompareA = 0x6000;
    Adt_SetCompareValue(enAdt, enAdtCompareA, u16CompareA);  //通用比较基准值寄存器A设置
    
    enAdtCompareB = AdtCompareB;
    u16CompareB = 0x6000;
    Adt_SetCompareValue(enAdt, enAdtCompareB, u16CompareB);  //通用比较基准值寄存器B设置
    
    stcAdtTIM4ACfg.enCap = AdtCHxCompareOutput;             //比较输出
    stcAdtTIM4ACfg.bOutEn = TRUE;                           //CHA输出使能
    stcAdtTIM4ACfg.enPerc = AdtCHxPeriodKeep;               //计数值与周期匹配时CHA电平保持不变
    stcAdtTIM4ACfg.enCmpc = AdtCHxCompareInv;               //计数值与比较值A匹配时，CHA电平翻转
    stcAdtTIM4ACfg.enStaStp = AdtCHxStateSelSS;             //CHA起始结束电平由STACA与STPCA控制
    stcAdtTIM4ACfg.enStaOut = AdtCHxPortOutLow;             //CHA起始电平为低
    stcAdtTIM4ACfg.enStpOut = AdtCHxPortOutLow;             //CHA结束电平为低
    stcAdtTIM4ACfg.enDisSel = AdtCHxDisSel0;                //选择强制输出无效条件0(刹车条件0)
    stcAdtTIM4ACfg.enDisVal = AdtTIMxDisValLow;             //刹车时CHA端口输出低电平
    Adt_CHxXPortConfig(enAdt, AdtCHxA, &stcAdtTIM4ACfg);    //端口CHA配置
    
    stcAdtTIM4BCfg.enCap = AdtCHxCompareOutput;
    stcAdtTIM4BCfg.bOutEn = TRUE;
    stcAdtTIM4BCfg.enPerc = AdtCHxPeriodKeep;
    stcAdtTIM4BCfg.enCmpc = AdtCHxCompareInv;
    stcAdtTIM4BCfg.enStaStp = AdtCHxStateSelSS;
    stcAdtTIM4BCfg.enStaOut = AdtCHxPortOutHigh;
    stcAdtTIM4BCfg.enStpOut = AdtCHxPortOutLow;
    stcAdtTIM4BCfg.enDisSel = AdtCHxDisSel0;
    stcAdtTIM4BCfg.enDisVal = AdtTIMxDisValLow;
    Adt_CHxXPortConfig(enAdt, AdtCHxB, &stcAdtTIM4BCfg);    //端口CHB配置
    
    Adt_StartCount(enAdt);
    
    ConfigVc();
    
    while(1);
}

/******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/


