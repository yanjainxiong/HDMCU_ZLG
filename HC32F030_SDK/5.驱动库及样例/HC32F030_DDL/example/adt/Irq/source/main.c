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
#include "gpio.h"

/******************************************************************************
 * Local pre-processor symbols/macros ('#define')                            
 ******************************************************************************/
#define DEBUG_PRINT

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
static stc_adt_cntstate_cfg_t stcAdt5CntState;

/*****************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/

void Adt5CompACalllback(void)
{
    Adt_GetCntState(AdtTIM5, &stcAdt5CntState);
}

void Adt5CompBCalllback(void)
{
    Adt_GetCntState(AdtTIM5, &stcAdt5CntState);
}
void Adt5CompCCalllback(void)
{
    Adt_GetCntState(AdtTIM5, &stcAdt5CntState);
}
void Adt5CompDCalllback(void)
{
    Adt_GetCntState(AdtTIM5, &stcAdt5CntState);
}

void Adt5CaptureACalllback(void)
{
    uint16_t u16Capture;
    
    Adt_GetCaptureValue(AdtTIM5, AdtCHxA, &u16Capture);
    Adt_GetCntState(AdtTIM5, &stcAdt5CntState);
}

void Adt5CaptureBCalllback(void)
{
    uint16_t u16Capture;
    
    Adt_GetCaptureValue(AdtTIM5, AdtCHxB, &u16Capture);
    Adt_GetCntState(AdtTIM5, &stcAdt5CntState);
}

void Adt5OVFCalllback(void)
{
    Adt_GetCntState(AdtTIM5, &stcAdt5CntState);
}
void Adt5UDFCalllback(void)
{
    Adt_GetCntState(AdtTIM5, &stcAdt5CntState);
}

void Adt5DTECalllback(void)
{
    Adt_StopCount(AdtTIM5);
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
    en_adt_unit_t enAdt;
    uint16_t u16Period;
    en_adt_compare_t enAdtCompare;
    uint16_t u16Compare;
    
    stc_adt_basecnt_cfg_t     stcAdtBaseCntCfg;
    stc_adt_CHxX_port_cfg_t   stcAdtTIMACfg;
    stc_adt_CHxX_port_cfg_t   stcAdtTIMBCfg;
    stc_adt_port_trig_cfg_t   stcAdtPortTrig;
    stc_gpio_config_t         stcTIM5APort;
    stc_gpio_config_t         stcTIM5BPort;
    stc_gpio_config_t         stcTRIBPort;
    stc_gpio_config_t         stcOutPort;
    
    DDL_ZERO_STRUCT(stcAdtBaseCntCfg);
    DDL_ZERO_STRUCT(stcAdtTIMACfg);
    DDL_ZERO_STRUCT(stcAdtTIMBCfg);
    DDL_ZERO_STRUCT(stcAdtPortTrig);
    DDL_ZERO_STRUCT(stcTIM5APort);
    DDL_ZERO_STRUCT(stcTIM5BPort);
    DDL_ZERO_STRUCT(stcTRIBPort);
    DDL_ZERO_STRUCT(stcOutPort);
    
    
    
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE);         //端口外设时钟使能
        
    //PA04设置为TIM5_CHA
    //Gpio_ClrAnalogMode(GpioPortA, GpioPin4);
    stcTIM5APort.enDir  = GpioDirOut;
    Gpio_Init(GpioPortA, GpioPin4, &stcTIM5APort);
    Gpio_SetAfMode(GpioPortA,GpioPin4,GpioAf5);
    
    //PA05设置为TIM5_CHB
    //Gpio_ClrAnalogMode(GpioPortA, GpioPin5);
    stcTIM5BPort.enDir  = GpioDirOut;
    Gpio_Init(GpioPortA, GpioPin5, &stcTIM5BPort);
    Gpio_SetAfMode(GpioPortA,GpioPin5,GpioAf5);
    
    if (Ok != Sysctrl_SetPeripheralGate(SysctrlPeripheralAdvTim, TRUE))//ADT外设时钟使能
    {
        return Error;
    }
    
    enAdt = AdtTIM5;
    
    stcAdtBaseCntCfg.enCntMode = AdtTriangleModeA;
    stcAdtBaseCntCfg.enCntDir = AdtCntUp;
    stcAdtBaseCntCfg.enCntClkDiv = AdtClkPClk0;
    Adt_Init(enAdt, &stcAdtBaseCntCfg);                      //ADT载波、计数模式、时钟配置

    /******************************比较中断测试********************************/
    stcAdtTIMACfg.enCap = AdtCHxCompareOutput;
    stcAdtTIMACfg.bOutEn = FALSE;
    stcAdtTIMBCfg.enCap = AdtCHxCompareOutput;
    stcAdtTIMBCfg.bOutEn = FALSE;
    Adt_CHxXPortConfig(enAdt, AdtCHxA, &stcAdtTIMACfg);        //TIM5_CHA端口配置
    Adt_CHxXPortConfig(enAdt, AdtCHxB, &stcAdtTIMBCfg);        //TIM5_CHA端口配置

    
    u16Period = 0xA000;
    Adt_SetPeriod(enAdt, u16Period);                           //周期设置

    enAdtCompare = AdtCompareA;
    u16Compare = 0x1000;
    Adt_SetCompareValue(enAdt, enAdtCompare, u16Compare);      //通用比较基准值寄存器A设置

    enAdtCompare = AdtCompareB;
    u16Compare = 0x3000;
    Adt_SetCompareValue(enAdt, enAdtCompare, u16Compare);       //通用比较基准值寄存器B设置
    
    enAdtCompare = AdtCompareC;
    u16Compare = 0x5000;
    Adt_SetCompareValue(enAdt, enAdtCompare, u16Compare);       //通用比较基准值寄存器C设置
    
    enAdtCompare = AdtCompareD;
    u16Compare = 0x7000;
    Adt_SetCompareValue(enAdt, enAdtCompare, u16Compare);       //通用比较基准值寄存器D设置
    
    Adt_ConfigIrq(enAdt, AdtCMAIrq, TRUE, Adt5CompACalllback);
    Adt_ConfigIrq(enAdt, AdtCMBIrq, TRUE, Adt5CompBCalllback);
    Adt_ConfigIrq(enAdt, AdtCMCIrq, TRUE, Adt5CompCCalllback);
    Adt_ConfigIrq(enAdt, AdtCMDIrq, TRUE, Adt5CompDCalllback);  //比较中断配置
    
    Adt_StartCount(enAdt);
    
    delay1ms(1000);
    
    Adt_ConfigIrq(enAdt, AdtCMAIrq, FALSE, Adt5CompACalllback);
    Adt_ConfigIrq(enAdt, AdtCMBIrq, FALSE, Adt5CompBCalllback);
    Adt_ConfigIrq(enAdt, AdtCMCIrq, FALSE, Adt5CompCCalllback);
    Adt_ConfigIrq(enAdt, AdtCMDIrq, FALSE, Adt5CompDCalllback); //比较中断解除
    Adt_StopCount(enAdt);
    Adt_ClearCount(enAdt);
    
    
    /*******************************捕获中断测试*******************************/
    
    // PB14 -> PB11 Rise -> Time TrigB Rise -> HW Capture A
    // PB14 -> PB11 Fall -> Time TrigB Fall -> HW Capture B
        
    //PB11设置为ADT TRIB 输入
    //Gpio_ClrAnalogMode(GpioPortB, GpioPin11);
    stcTRIBPort.enDir  = GpioDirIn;
    Gpio_Init(GpioPortB, GpioPin11, &stcTRIBPort);

    
    //PB14设置为输出信号口
    //Gpio_ClrAnalogMode(GpioPortB, GpioPin14);
    stcOutPort.enDir  = GpioDirOut;
    Gpio_Init(GpioPortB, GpioPin14, &stcOutPort);

    Gpio_WriteOutputIO(GpioPortB, GpioPin14, FALSE);                //PB14输出低
    
    stcAdtPortTrig.enTrigSrc = AdtTrigxSelPB11;                     //PB11作为TRIB端口
    stcAdtPortTrig.bFltEn = TRUE;                                   //端口滤波使能
    stcAdtPortTrig.enFltClk = AdtFltClkPclk0Div16;                  //滤波时钟
    Adt_PortTrigConfig(AdtTrigB, &stcAdtPortTrig);                  //端口触发配置
    Adt_ConfigHwCaptureA(AdtTIM5, AdtHwTrigTimTriBRise);             //硬件捕获A条件配置TRIB端口采到上升沿
    Adt_ConfigHwCaptureB(AdtTIM5, AdtHwTrigTimTriBFall);             //硬件捕获B条件配置TRIB端口采到下降沿
    Adt_ConfigIrq(AdtTIM5, AdtCMAIrq, TRUE, Adt5CaptureACalllback);
    Adt_ConfigIrq(AdtTIM5, AdtCMBIrq, TRUE, Adt5CaptureBCalllback);  //捕获中断配置
    
    Adt_StartCount(enAdt);
    delay1ms(1000);
    Gpio_WriteOutputIO(GpioPortB, GpioPin14, TRUE);     //PB14输出高
    delay1ms(1000);
    Gpio_WriteOutputIO(GpioPortB, GpioPin14, FALSE);     //PB14输出低
    delay1ms(1000);
    
    Adt_ConfigIrq(AdtTIM5, AdtCMAIrq, FALSE, Adt5CaptureACalllback);
    Adt_ConfigIrq(AdtTIM5, AdtCMBIrq, FALSE, Adt5CaptureBCalllback);//捕获中断解除
    Adt_ClearHwCaptureA(enAdt);
    Adt_ClearHwCaptureB(enAdt);
    Adt_StopCount(enAdt);
    Adt_ClearCount(enAdt);


    /***************************上下溢中断测试**********************************/
    Adt_ConfigIrq(enAdt, AdtOVFIrq, TRUE, Adt5OVFCalllback);//上溢中断配置
    Adt_ConfigIrq(enAdt, AdtUDFIrq, TRUE, Adt5UDFCalllback);//下溢中断配置
    
    Adt_StartCount(enAdt);
    
    delay1ms(2000);

    Adt_ConfigIrq(enAdt, AdtOVFIrq, FALSE, Adt5OVFCalllback);//上溢中断解除
    Adt_ConfigIrq(enAdt, AdtUDFIrq, FALSE, Adt5UDFCalllback);//下溢中断解除
    
    Adt_StopCount(enAdt);
    Adt_ClearCount(enAdt);
    
    
    /****************************死区时间错误中断测试****************************/
        
    enAdtCompare = AdtCompareA;
    u16Compare = 0x5000;
    Adt_SetCompareValue(enAdt, enAdtCompare, u16Compare);//通用比较基准值寄存器A设置
    DDL_ZERO_STRUCT(stcAdtTIMBCfg);
    stcAdtTIMBCfg.enCap = AdtCHxCompareOutput;
    stcAdtTIMBCfg.bOutEn = TRUE;
    stcAdtTIMBCfg.enPerc = AdtCHxPeriodKeep;
    stcAdtTIMBCfg.enCmpc = AdtCHxCompareInv;
    stcAdtTIMBCfg.enStaStp = AdtCHxStateSelSS;
    stcAdtTIMBCfg.enStaOut = AdtCHxPortOutHigh;
    Adt_CHxXPortConfig(enAdt, AdtCHxB, &stcAdtTIMBCfg);//CHxB端口配置
    
    Adt_SetDTUA(enAdt, 0x6666);
    Adt_SetDTDA(enAdt, 0x6666);
    Adt_ConfigDT(enAdt, TRUE, TRUE);                     //死区功能配置
    
    Adt_ConfigIrq(enAdt, AdtDTEIrq, TRUE, Adt5DTECalllback);//死区中断配置
    
    Adt_StartCount(enAdt);
    
    delay1ms(1000);
    Adt_ConfigIrq(enAdt, AdtDTEIrq, FALSE, Adt5DTECalllback);//死区中断解除
    Adt_ConfigDT(enAdt, FALSE, FALSE);
    
    Adt_StopCount(enAdt);
    Adt_ClearCount(enAdt);
    
    while(1);
}

/******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/


