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

/******************************************************************************
 * Local pre-processor symbols/macros ('#define')
 ******************************************************************************/

/******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/
uint32_t u32PwcCapValue;
uint16_t u16TIM0_CntValue;

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
 * BT1中断服务函数
 ******************************************************************************/
void Tim0Int(void)
{
    static uint16_t u16TIM0_OverFlowCnt;
    
    static uint16_t u16TIM0_CapValue;
     
    if(TRUE == Bt_GetIntFlag(TIM0, BtUevIrq))
    {
        u16TIM0_OverFlowCnt++;
        
        //u16TIM0_CntValue = 0;
        //u16TIM0_CapValue = 0;
        //u32PwcCapValue = 0;
        
        Bt_ClearIntFlag(TIM0,BtUevIrq);
    }

    if(TRUE == Bt_GetIntFlag(TIM0, BtCA0Irq))
    {
        u16TIM0_CntValue = Bt_M1_Cnt16Get(TIM0);
        u16TIM0_CapValue = Bt_M1_PWC_CapValueGet(TIM0);
        
        u32PwcCapValue = u16TIM0_OverFlowCnt*0x10000 + u16TIM0_CapValue;
        
        u16TIM0_OverFlowCnt = 0;
        
        Bt_ClearIntFlag(TIM0, BtCA0Irq);
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
    uint16_t                   u16CntValue;
    stc_bt_mode1_config_t      stcBtBaseCfg;
    stc_bt_pwc_input_config_t  stcBtPwcInCfg;
    stc_gpio_config_t          stcTIM0APort;
    stc_gpio_config_t          stcTIM0BPort;

    DDL_ZERO_STRUCT(stcBtBaseCfg);
    DDL_ZERO_STRUCT(stcTIM0APort);
    DDL_ZERO_STRUCT(stcTIM0BPort);

    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE); //端口外设时钟使能
    Sysctrl_SetPeripheralGate(SysctrlPeripheralBTim, TRUE); //Base Timer外设时钟使能
    
    //PA00设置为TIM0_CHA
    //Gpio_ClrAnalogMode(GpioPortA, GpioPin0);
    stcTIM0APort.enDir  = GpioDirIn;
    Gpio_Init(GpioPortA, GpioPin0, &stcTIM0APort);
    Gpio_SetAfMode(GpioPortA,GpioPin0,GpioAf7);
    
    //PA01设置为TIM0_CHB
    //Gpio_ClrAnalogMode(GpioPortA, GpioPin1);
    //stcTIM0BPort.enDir  = GpioDirIn;
    //Gpio_Init(GpioPortA, GpioPin1, &stcTIM0BPort);
    //Gpio_SetAfMode(GpioPortA,GpioPin1,GpioAf3);
    
    stcBtBaseCfg.enWorkMode = BtWorkMode1;                  //定时器模式
    stcBtBaseCfg.enCT       = BtTimer;                      //定时器功能，计数时钟为内部PCLK
    stcBtBaseCfg.enPRS      = BtPCLKDiv1;                   //PCLK
    stcBtBaseCfg.enOneShot  = BtPwcOneShotDetect;           //PWC循环检测

    stcBtBaseCfg.pfnTim0Cb  = Tim0Int;                      //中断函数入口
    
    Bt_Mode1_Init(TIM0, &stcBtBaseCfg);                     //TIM0 的模式0功能初始化
    
    stcBtPwcInCfg.enTsSel  = BtTs6IAFP;                     //PWC输入选择 CHA
    stcBtPwcInCfg.enIA0Sel = BtIA0Input;                    //CHA选择IA0
    stcBtPwcInCfg.enFltIA0 = BtFltPCLKDiv16Cnt3;            //PCLK/16 3个连续有效
    //stcBtPwcInCfg.enIB0Sel = BtIB0Input;                  //CHB选择IB0
    //stcBtPwcInCfg.enFltIB0 = BtFltPCLKDiv16Cnt3;          //PCLK/16 3个连续有效
    
    Bt_M1_Input_Config(TIM0, &stcBtPwcInCfg);               //PWC输入设置
    
    Bt_M1_PWC_Edge_Sel(TIM0, BtPwcRiseToFall);              //上升沿到下降沿捕获
    
    u16CntValue = 0;
    
    Bt_M1_Cnt16Set(TIM0, u16CntValue);                      //设置计数初值
		
    Bt_ClearIntFlag(TIM0,BtUevIrq);                         //清Uev中断标志
    Bt_ClearIntFlag(TIM0,BtCA0Irq);                         //清捕捉中断标志
    
    Bt_Mode1_EnableIrq(TIM0, BtUevIrq);                     //使能TIM0溢出中断
    Bt_Mode1_EnableIrq(TIM0, BtCA0Irq);                     //使能TIM0捕获中断
		
    EnableNvic(TIM0_IRQn, IrqLevel3, TRUE);                 //TIM0中断使能
    
    Bt_M1_Run(TIM0);                                        //TIM0 运行。
    
    while (1)
    {
        delay1ms(2000);
        Bt_M1_Run(TIM0);                                   //重复使能PWC功能
    }
}

/******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/


