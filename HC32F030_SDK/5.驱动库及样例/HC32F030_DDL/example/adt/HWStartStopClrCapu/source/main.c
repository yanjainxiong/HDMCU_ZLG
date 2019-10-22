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
#include "pca.h"

/******************************************************************************
 * Local pre-processor symbols/macros ('#define')                            
 ******************************************************************************/
#define DEBUG_PRINT

/******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/
uint16_t u16TIM6_HW_Sync_Cnt;

/******************************************************************************
 * Local type definitions ('typedef')                                         
 ******************************************************************************/

/******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/


/******************************************************************************
 * Local variable definitions ('static')                                      *
 ******************************************************************************/
static stc_adt_cntstate_cfg_t stcAdt4CntState;
static stc_adt_cntstate_cfg_t stcAdt5CntState;
static stc_adt_cntstate_cfg_t stcAdt6CntState;


/*****************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/

 void Adt4CmpACalllback(void)
{
    u16TIM6_HW_Sync_Cnt = Adt_GetCount(AdtTIM6);
    Adt_StopCount(AdtTIM4);   
}

void Adt5CmpACalllback(void)
{
    u16TIM6_HW_Sync_Cnt = Adt_GetCount(AdtTIM6);
    Adt_StopCount(AdtTIM5);
}


void PcaInt(void)
{
    if (TRUE == Pca_GetCntIntFlag())  //PCA 计数器中断
    {
        u16TIM6_HW_Sync_Cnt = Adt_GetCount(AdtTIM6);
        Pca_ClearCntIntFlag();
        Pca_Stop();                //PCA运行
    }
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
    uint16_t                u16Period;
    uint16_t                u16GCMAValue;
    uint16_t                u16ArrData;
    stc_adt_basecnt_cfg_t   stcAdtBaseCntCfg;
    stc_adt_port_trig_cfg_t stcAdtPortTrig;
    stc_pca_config_t        stcPcaConfig;
    stc_pca_capmodconfig_t  stcPcaModConfig;
    stc_adt_aos_trig_cfg_t  stcAdtAosTrigCfg;
    stc_adt_irq_trig_cfg_t  stcAdtIrqTrigCfg;
    stc_adt_sw_sync_t       stcAdtSwSync;
    
    DDL_ZERO_STRUCT(stcAdtBaseCntCfg);
    DDL_ZERO_STRUCT(stcAdtPortTrig);
    DDL_ZERO_STRUCT(stcPcaConfig);
    DDL_ZERO_STRUCT(stcPcaModConfig);
    DDL_ZERO_STRUCT(stcAdtAosTrigCfg);
    DDL_ZERO_STRUCT(stcAdtIrqTrigCfg);
    DDL_ZERO_STRUCT(stcAdtSwSync);
    DDL_ZERO_STRUCT(stcAdt4CntState);
    DDL_ZERO_STRUCT(stcAdt5CntState);
    DDL_ZERO_STRUCT(stcAdt6CntState);
    
    
    
    
    //使能PCA,GPIO外设时钟
    Sysctrl_SetPeripheralGate(SysctrlPeripheralPca, TRUE);
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE);

    
    if (Ok != Sysctrl_SetPeripheralGate(SysctrlPeripheralAdvTim, TRUE)) //ADT外设时钟使能
    {
        return Error;
    }
    
    stcAdtBaseCntCfg.enCntMode = AdtSawtoothMode;
    stcAdtBaseCntCfg.enCntDir = AdtCntUp;
    stcAdtBaseCntCfg.enCntClkDiv = AdtClkPClk0Div2;//AdtClkPClk0Div256;
    Adt_Init(AdtTIM4, &stcAdtBaseCntCfg);
    Adt_Init(AdtTIM5, &stcAdtBaseCntCfg);
    Adt_Init(AdtTIM6, &stcAdtBaseCntCfg);                        //ADT载波、计数模式、时钟配置

    u16Period = 0xF000;
    Adt_SetPeriod(AdtTIM4, u16Period);
    Adt_SetPeriod(AdtTIM5, u16Period);
    Adt_SetPeriod(AdtTIM6, u16Period);                           //周期设置
    
    stcAdtAosTrigCfg.enAos0TrigSrc = AdtAosxTrigSelPcaInt;       //AOS0选择PCA中断
    stcAdtAosTrigCfg.enAos1TrigSrc = AdtAosxTrigSelTim4Int;      //AOS1选择TIM4
    stcAdtAosTrigCfg.enAos2TrigSrc = AdtAosxTrigSelTim5Int;      //AOS2选择TIM5
    Adt_AosTrigConfig(&stcAdtAosTrigCfg);
        
    Adt_ConfigHwStart(AdtTIM6, AdtHwTrigAos0);                   //AOS0事件触发TIM6启动
    Adt_ConfigHwStop(AdtTIM6, AdtHwTrigAos1);                    //AOS1事件 停止TIM6
    Adt_ConfigHwClear(AdtTIM6, AdtHwTrigAos2);                   //AOS2事件 清零TIM6

    Adt_EnableHwStart(AdtTIM6);                                  //硬件启动使能
    Adt_EnableHwStop(AdtTIM6);                                   //硬件停止使能
    Adt_EnableHwClear(AdtTIM6);                                  //硬件清零使能
    
    u16GCMAValue = 0x6000;
    Adt_SetCompareValue(AdtTIM4, AdtCompareA, u16GCMAValue);     //通用比较基准值寄存器A设置
    Adt_ConfigIrq(AdtTIM4, AdtCMAIrq, TRUE, Adt4CmpACalllback);  //配置TIM4比较中断
    
    u16GCMAValue = 0xE000;
    Adt_SetCompareValue(AdtTIM5, AdtCompareA, u16GCMAValue);     //通用比较基准值寄存器A设置
    Adt_ConfigIrq(AdtTIM5, AdtCMAIrq, TRUE, Adt5CmpACalllback);  //配置TIM5比较中断
    
    stcAdtIrqTrigCfg.bAdtCntMatchATrigEn = 1;
    Adt_IrqTrigConfig(AdtTIM4, &stcAdtIrqTrigCfg);               //TIM4 比较中断A触发AOS事件使能
    Adt_IrqTrigConfig(AdtTIM5, &stcAdtIrqTrigCfg);               //TIM5 比较中断A触发AOS事件使能
    
    stcPcaConfig.enCIDL = IdleGoOn;                              //休眠模式PCA工作 
    stcPcaConfig.enWDTE = PCAWDTDisable;                         //wdt功能关闭
    stcPcaConfig.enCPS  = PCAPCLKDiv8;                           //PCLK/8,
    
    stcPcaConfig.pfnPcaCb = PcaInt;
    
    stcPcaModConfig.enECOM     = ECOMDisable;                    //允许比较功能
    stcPcaModConfig.enCAPP     = CAPPDisable;                    //上升沿捕获禁止
    stcPcaModConfig.enCAPN     = CAPNDisable;                    //下降沿捕获禁止
    stcPcaModConfig.enMAT      = MATDisable;                     //允许匹配
    stcPcaModConfig.enTOG      = TOGDisable;                     //禁止翻转
    stcPcaModConfig.en8bitPWM  = PCA8bitPWMDisable;              //禁止8位PWM功能
    
    if (Ok != Pca_Init(&stcPcaConfig))
    {
        return Error;
    }
    if (Ok != Pca_CapModConfig(Module0, &stcPcaModConfig))
    {
        return Error;
    }
    
    Pca_Enable16bitPWM();                        //(增强PWM功能使能)使能周期重载功能.
    u16ArrData = 0x10;
    Pca_CARRSet(u16ArrData);                     //周期重载存器设置
    Pca_Cnt16Set(0);                             //计数初值设置
    EnableNvic(PCA_IRQn, IrqLevel3, TRUE);
    Pca_EnableCntIrq();                          //PCA计数器中断使能
    
    stcAdtSwSync.bAdTim4 = TRUE;
    stcAdtSwSync.bAdTim5 = TRUE;
    
    Pca_Run();
     
    Adt_SwSyncStart(&stcAdtSwSync);
    
    while(1);
}

/******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/


