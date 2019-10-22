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
#include "timer3.h"
#include "lpm.h"
#include "gpio.h"

/******************************************************************************
 * Local pre-processor symbols/macros ('#define')
 ******************************************************************************/

/******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/

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
void Tim3Int(void)
{
     //if(TRUE == Tim3_GetIntFlag(Tim3UevIrq))
     //{
     //    
     //   Tim3_ClearIntFlag(Tim3UevIrq);
     //}
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
    uint16_t                  u16ArrValue;
    uint16_t                  u16CntValue;
    stc_tim3_mode0_config_t   stcTim3BaseCfg;
    stc_gpio_config_t         stcTIM3A0Port;
    stc_gpio_config_t         stcTIM3B0Port;
    
    DDL_ZERO_STRUCT(stcTim3BaseCfg);
    DDL_ZERO_STRUCT(stcTIM3A0Port);
    DDL_ZERO_STRUCT(stcTIM3B0Port);
    
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE); //GPIO 外设时钟使能
    Sysctrl_SetPeripheralGate(SysctrlPeripheralTim3, TRUE); //Timer3外设时钟使能
    
    //PA08设置为TIM3_CH0A
    //Gpio_ClrAnalogMode(GpioPortA, GpioPin8);
    stcTIM3A0Port.enDir  = GpioDirOut;
    Gpio_Init(GpioPortA, GpioPin8, &stcTIM3A0Port);
    Gpio_SetAfMode(GpioPortA,GpioPin8,GpioAf2);
    
    //PA07设置为TIM3_CH0B
    //Gpio_ClrAnalogMode(GpioPortA, GpioPin7);
    stcTIM3B0Port.enDir  = GpioDirOut;
    Gpio_Init(GpioPortA, GpioPin7, &stcTIM3B0Port);
    Gpio_SetAfMode(GpioPortA,GpioPin7,GpioAf4);
    
    stcTim3BaseCfg.enWorkMode = Tim3WorkMode0;               //定时器模式
    stcTim3BaseCfg.enCT       = Tim3Timer;                   //定时器功能，计数时钟为内部PCLK
    stcTim3BaseCfg.enPRS      = Tim3PCLKDiv16;               //PCLK/16
    stcTim3BaseCfg.enCntMode  = Tim316bitArrMode;            //自动重载16位计数器/定时器
    stcTim3BaseCfg.bEnTog     = TRUE;
    stcTim3BaseCfg.bEnGate    = FALSE;
    stcTim3BaseCfg.enGateP    = Tim3GatePositive;
    
    //stcTim3BaseCfg.pfnTim3Cb  = Tim3Int;                   //中断函数入口
    
    Tim3_Mode0_Init(&stcTim3BaseCfg);                        //TIM3 的模式0功能初始化
        
    u16ArrValue = 0x9000;
    
    Tim3_M0_ARRSet(u16ArrValue);                             //设置重载值
    
    u16CntValue = 0x9000;
    
    Tim3_M0_Cnt16Set(u16CntValue);                           //设置计数初值
    
    //Tim3_ClearIntFlag(Tim3UevIrq);                         //清中断标志
    
    //EnableNvic(TIM3_IRQn, 3, TRUE);                        //TIM3 开中断
    
    //Tim3_Mode0_EnableIrq();                                //使能TIM3中断(模式0时只有一个中断)    
    
    Tim3_M0_EnTOG_Output(TRUE);                              //TIM3 端口输出使能
    
    Tim3_M0_Run();                                           //TIM3 运行。
    
    while (1);
}

/******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/


