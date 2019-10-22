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
#include "bt.h"
#include "lpm.h"
#include "gpio.h"
#include "flash.h"
#include "adc.h"

/******************************************************************************
 * Local pre-processor symbols/macros ('#define')
 ******************************************************************************/

/******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/
uint16_t u16AdcResult02;
uint16_t u16AdcResult03;
uint16_t u16AdcResult05;

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/


/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/


/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/

/*******************************************************************************
 * BT2中断服务函数
 ******************************************************************************/
void Tim0Int(void)
{
    static uint8_t i;
    
    if(TRUE == Bt_GetIntFlag(TIM0, BtUevIrq))
    {
        if(0 == i)
        {
            Gpio_WriteOutputIO(GpioPortD, GpioPin5, TRUE);
            i++;
        }
        else if(1 == i)
        {
            Gpio_WriteOutputIO(GpioPortD, GpioPin5, FALSE);
            i = 0;
        }
        
        Bt_ClearIntFlag(TIM0,BtUevIrq);
    }
}

void AdcJQRIrqCallback(void)
{
    static uint8_t i;
    
    Adc_GetJqrResult(&u16AdcResult02, 0);
    Adc_GetJqrResult(&u16AdcResult03, 1);
    Adc_GetJqrResult(&u16AdcResult05, 2);
    
    if(0 == i)
    {
        Bt_M23_CCR_Set(TIM0, BtCCR0A, 0x3000);
        i++;
    }
    else if(1 == i)
    {
        Bt_M23_CCR_Set(TIM0, BtCCR0A, 0x6000);
        i = 0;
    }
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
        
    Gpio_SetAnalogMode(GpioPortA, GpioPin2);        //PA02
    Gpio_SetAnalogMode(GpioPortA, GpioPin3);        //PA03
    Gpio_SetAnalogMode(GpioPortA, GpioPin5);        //PA05
    
    Sysctrl_SetPeripheralGate(SysctrlPeripheralAdcBgr, TRUE);
    
    //ADC配置
    Adc_Enable();
    M0P_BGR->CR_f.BGR_EN = 0x1u;//BGR必须使能
    M0P_BGR->CR_f.TS_EN  = 0x0u;
    delay100us(10);
    
    stcAdcCfg.enAdcOpMode      = AdcSCanMode;          //连续采样模式
    stcAdcCfg.enAdcClkDiv      = AdcClkSysTDiv2;       //Adc工作时钟 PCLK/2
    stcAdcCfg.enAdcSampTimeSel = AdcSampTime8Clk;      //采样时钟 8个周期
    stcAdcCfg.enAdcRefVolSel   = RefVolSelAVDD;        //内部AVDD
    stcAdcCfg.bAdcInBufEn      = FALSE;

    
    Adc_Init(&stcAdcCfg);                              //Adc初始化
    
    Adc_ConfigJqrChannel(JQRCH0MUX, AdcExInputCH2);    //配置插队扫描转换通道
    Adc_ConfigJqrChannel(JQRCH1MUX, AdcExInputCH3);
    Adc_ConfigJqrChannel(JQRCH2MUX, AdcExInputCH5);
    
    stcAdcIrq.bAdcJQRIrq = TRUE;
    stcAdcIrqCalbaks.pfnAdcJQRIrq = AdcJQRIrqCallback;
    Adc_ConfigIrq(&stcAdcIrq, &stcAdcIrqCalbaks);       //中断函数入口配置
    
    u8AdcScanCnt = 3;                                   //转换次数3次(3-1已在库函数内计算)
    
    Adc_ConfigJqrMode(&stcAdcCfg, u8AdcScanCnt, FALSE); //配置插队扫描转换模式
    
    stcAdcExtTrigCfg.enAdcExtTrigRegSel = AdcExtTrig1;
    stcAdcExtTrigCfg.enAdcTrig1Sel      = AdcTrigTimer0;
    Adc_ExtTrigCfg(&stcAdcExtTrigCfg);                  //Timer0触发插队扫描转换
    
    Adc_EnableIrq();                                    //使能Adc中断
    EnableNvic(ADC_IRQn, IrqLevel1, TRUE);              //Adc开中断
    
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
    uint16_t                      u16ArrValue;
    uint16_t                      u16CompareAValue;
    uint16_t                      u16CntValue;
    uint8_t                       u8ValidPeriod;
    en_flash_waitcycle_t          enFlashWait;
    stc_sysctrl_clk_config_t      stcClkConfig;
    stc_bt_mode23_config_t        stcBtBaseCfg;
    stc_bt_m23_compare_config_t   stcBtPortCmpCfg;
    stc_bt_m23_adc_trig_config_t  stcBtTrigAdc;
    stc_bt_m23_dt_config_t        stcBtDeadTimeCfg;
    stc_gpio_config_t             stcTIM0Port;
    stc_gpio_config_t             stcLEDPort;
    
    DDL_ZERO_STRUCT(stcClkConfig);
    DDL_ZERO_STRUCT(stcBtBaseCfg);
    DDL_ZERO_STRUCT(stcTIM0Port);
    DDL_ZERO_STRUCT(stcLEDPort);
    DDL_ZERO_STRUCT(stcBtPortCmpCfg);
    DDL_ZERO_STRUCT(stcBtTrigAdc);
    DDL_ZERO_STRUCT(stcBtDeadTimeCfg);
    
    enFlashWait = FlashWaitCycle1;                      //读等待周期设置为1（当HCLK大于24MHz时必须至少为1）
    Flash_WaitCycle(enFlashWait);                       // Flash 等待1个周期
    
    stcClkConfig.enClkSrc    = SysctrlClkXTH;           //使用外部高速晶振,32M
    stcClkConfig.enHClkDiv   = SysctrlHclkDiv1;         // HCLK = SystemClk/1
    stcClkConfig.enPClkDiv   = SysctrlPclkDiv1;         // PCLK = HCLK/1
    Sysctrl_SetXTHFreq(SysctrlXthFreq20_32MHz);         //设置外部高速频率为20~32M
    Sysctrl_ClkInit(&stcClkConfig);
    
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE);   //GPIO 外设时钟使能
    Sysctrl_SetPeripheralGate(SysctrlPeripheralBTim, TRUE);   //Base Timer外设时钟使能
        
    stcLEDPort.enDir  = GpioDirOut;
    Gpio_Init(GpioPortD, GpioPin5, &stcLEDPort);              //PD05设置为LED输出
    
    stcTIM0Port.enDir  = GpioDirOut;
    
    Gpio_Init(GpioPortA, GpioPin0, &stcTIM0Port);
    Gpio_SetAfMode(GpioPortA,GpioPin0,GpioAf7);            //PA00设置为TIM0_CHA
    
    Gpio_Init(GpioPortA, GpioPin1, &stcTIM0Port);
    Gpio_SetAfMode(GpioPortA,GpioPin1,GpioAf3);            //PA01设置为TIM0_CHB
        
    stcBtBaseCfg.enWorkMode    = BtWorkMode3;              //三角波模式
    stcBtBaseCfg.enCT          = BtTimer;                  //定时器功能，计数时钟为内部PCLK
    stcBtBaseCfg.enPRS         = BtPCLKDiv1;               //PCLK
    //stcBtBaseCfg.enCntDir    = BtCntUp;                  //向上计数，在三角波模式时只读
    stcBtBaseCfg.enPWMTypeSel  = BtComplementaryPWM;       //互补输出PWM
    stcBtBaseCfg.enPWM2sSel    = BtSinglePointCmp;         //单点比较功能
    stcBtBaseCfg.bOneShot      = FALSE;                    //循环计数
    stcBtBaseCfg.bURSSel       = FALSE;                    //上下溢更新
    
    stcBtBaseCfg.pfnTim0Cb  = Tim0Int;                     //中断函数入口
    
    Bt_Mode23_Init(TIM0, &stcBtBaseCfg);                   //TIM0 的模式0功能初始化
        
    u16ArrValue = 0x9000;
    Bt_M23_ARRSet(TIM0, u16ArrValue, TRUE);                //设置重载值,并使能缓存
    
    u16CompareAValue = 0x6000;
    Bt_M23_CCR_Set(TIM0, BtCCR0A, u16CompareAValue);       //设置比较值A,(PWM互补模式下只需要设置比较值A)
    
    
    stcBtPortCmpCfg.enCH0ACmpCtrl   = BtPWMMode2;          //OCREFA输出控制OCMA:PWM模式2
    stcBtPortCmpCfg.enCH0APolarity  = BtPortPositive;      //正常输出
    stcBtPortCmpCfg.bCh0ACmpBufEn   = TRUE;                //A通道缓存控制
    stcBtPortCmpCfg.enCh0ACmpIntSel = BtCmpIntNone;        //A通道比较控制:无
    
    stcBtPortCmpCfg.enCH0BCmpCtrl   = BtPWMMode2;          //OCREFB输出控制OCMB:PWM模式2(PWM互补模式下也要设置，避免强制输出)
    stcBtPortCmpCfg.enCH0BPolarity  = BtPortPositive;      //正常输出
    //stcBtPortCmpCfg.bCH0BCmpBufEn   = TRUE;              //B通道缓存控制使能
    stcBtPortCmpCfg.enCH0BCmpIntSel = BtCmpIntNone;        //B通道比较控制:无
    
    Bt_M23_PortOutput_Config(TIM0, &stcBtPortCmpCfg);      //比较输出端口配置
    
    stcBtTrigAdc.bEnTrigADC    = TRUE;                     //使能ADC触发全局控制
    stcBtTrigAdc.bEnUevTrigADC = TRUE;                     //Uev更新触发ADC
    Bt_M23_TrigADC_Config(TIM0, &stcBtTrigAdc);            //触发ADC配置
    
    u8ValidPeriod = 1;                                     //事件更新周期设置，0表示三角波每半个周期更新一次，每+1代表延迟半个周期
    Bt_M23_SetValidPeriod(TIM0,u8ValidPeriod);             //间隔周期设置
    
    stcBtDeadTimeCfg.bEnDeadTime      = TRUE;
    stcBtDeadTimeCfg.u8DeadTimeValue  = 0xFF;
    Bt_M23_DT_Config(TIM0, &stcBtDeadTimeCfg);             //死区配置
    
    ConfigAdc();
    
    u16CntValue = 0;
    
    Bt_M23_Cnt16Set(TIM0, u16CntValue);                    //设置计数初值
    
    Bt_ClearAllIntFlag(TIM0);                              //清中断标志
    Bt_Mode23_EnableIrq(TIM0,BtUevIrq);                    //使能TIM0 UEV更新中断
    EnableNvic(TIM0_IRQn, IrqLevel0, TRUE);                //TIM0中断使能   
    
    Bt_M23_EnPWM_Output(TIM0, TRUE, FALSE);                //TIM0 端口输出使能
    
    Bt_M23_Run(TIM0);                                      //TIM0 运行。

    while (1);
    
    
    
}

/******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/


