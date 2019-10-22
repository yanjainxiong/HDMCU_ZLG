/******************************************************************************
* Copyright (C) 2018, Huada Semiconductor Co.,Ltd All rights reserved.
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
 **   - 2018-05-08  1.0  Lux First version for Device Driver Library of Module.
 **
 ******************************************************************************/

/******************************************************************************
 * Include files
 ******************************************************************************/
#include "gpio.h"
#include "lpm.h"
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
void Gpio_IRQHandler(uint8_t u8Param)
{
    ///< PORT D
    if(3 == u8Param)
    {
        if(TRUE == Gpio_GetIrqStatus(GpioPortD, GpioPin4))
        {            
            ///< 端口PD05设置为高电平（LED点亮）
            Gpio_SetIO(GpioPortD, GpioPin5);
            
            delay1ms(2000);
            
            ///< 端口PD05设置为低电平（LED关闭）
            Gpio_ClrIO(GpioPortD, GpioPin5);  

			Gpio_ClearIrq(GpioPortD, GpioPin4);	
        }

    }
}    


static void _UserKeyWait(void)
{
    stc_gpio_config_t stcGpioCfg;
    
    ///< 端口方向配置->输出
    stcGpioCfg.enDir = GpioDirIn;
    ///< 端口驱动能力配置->低驱动能力
    stcGpioCfg.enDrv = GpioDrvL;
    ///< 端口上下拉配置->上拉
    stcGpioCfg.enPuPd = GpioPu;
    ///< 端口开漏输出配置->开漏输出关闭
    stcGpioCfg.enOD = GpioOdDisable;
    ///< 端口输入/输出值寄存器总线控制模式配置->AHB
    stcGpioCfg.enCtrlMode = GpioAHB;
    
    ///< GPIO IO PD04初始化(PD04在STK上外接KEY(USER))
    Gpio_Init(GpioPortD, GpioPin4, &stcGpioCfg);
    
    while(1)
    {
        ///< 检测PD04电平(USER按键是否按下(低电平))
        if(FALSE == Gpio_GetInputIO(GpioPortD, GpioPin4))
        {
            break;
        }
        else
        {
            continue;
        }
    }
}


static void _LowPowerModeGpioSet(void)
{
	//Demo板所有低功耗模式不使用的IO配置为输出
    M0P_GPIO->PAADS = 0x0000;
    M0P_GPIO->PBADS = 0x0000;
    M0P_GPIO->PCADS = 0x0000;
    M0P_GPIO->PDADS = 0x00;
    
    M0P_GPIO->PADIR = 0x0000;
    M0P_GPIO->PBDIR = 0x0000;
    M0P_GPIO->PCDIR = 0x0000;
    M0P_GPIO->PDDIR = 0x00;
    
    //Demo所有低功耗模式下不使用的IO输出低电平
    M0P_GPIO->PAOUT = 0x0000;
    M0P_GPIO->PBOUT = 0x0000;
    M0P_GPIO->PCOUT = 0x0000;
    M0P_GPIO->PDOUT = 0x00;
	
}
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
int32_t main(void)
{
    stc_gpio_config_t stcGpioCfg;
    stc_lpm_config_t stcConfig;
    
    ///< 打开GPIO外设时钟门控
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE);
    
    ///< ===============================================
    ///< ============ 警告，警告，警告！！！=============
    ///< ===============================================
    ///< 本样例程序会进入深度休眠模式，因此以下两行代码起防护作用（防止进入深度休眠后芯片调试功能不能再次使用），
    ///< 在使用本样例时，禁止在没有唤醒机制的情况下删除以下两行代码。
    delay1ms(2000);
    _UserKeyWait();    
    
	///< 配置Demo板上所有不使用的IO为输出低
	_LowPowerModeGpioSet();
    
    ///< 端口方向配置->输入
    stcGpioCfg.enDir = GpioDirIn;
    ///< 端口驱动能力配置->高驱动能力
    stcGpioCfg.enDrv = GpioDrvL;
    ///< 端口上下拉配置->上拉
    stcGpioCfg.enPuPd = GpioPu;
    ///< 端口开漏输出配置->开漏输出关闭
    stcGpioCfg.enOD = GpioOdDisable;
    ///< 端口输入/输出值寄存器总线控制模式配置->AHB
    stcGpioCfg.enCtrlMode = GpioAHB;
    
    ///< GPIO IO PD04初始化(PD04在STK上外接KEY(USER))
    Gpio_Init(GpioPortD, GpioPin4, &stcGpioCfg);    
    
    ///< 端口方向配置->输出(其它参数与以上（输入）配置参数一致)
    stcGpioCfg.enDir = GpioDirOut;
    ///< 端口上下拉配置->下拉
    stcGpioCfg.enPuPd = GpioPd;
    ///< GPIO IO PD05初始化(PD05在STK上外接LED(LED3))
    Gpio_Init(GpioPortD, GpioPin5, &stcGpioCfg);
    ///< 端口PD05设置为低电平（LED关闭）
    Gpio_ClrIO(GpioPortD, GpioPin5);
    
	///< 深度休眠模式下响应端口中断
    Gpio_SfIrqModeConfig(GpioSfIrqDpslpMode);
	///< 打开并配置PD04为下降沿中断
    Gpio_EnableIrq(GpioPortD, GpioPin4, GpioIrqFalling);
    ///< 使能端口PORTD系统中断
    EnableNvic(PORTD_IRQn, IrqLevel3, TRUE);
	
	
    ///< 低功耗模式配置
    stcConfig.enSEVONPEND   = SevPndDisable;
    stcConfig.enSLEEPDEEP   = SlpDpEnable;
    stcConfig.enSLEEPONEXIT = SlpExtEnable;
    Lpm_Config(&stcConfig);
    
    Lpm_GotoLpmMode();
    
    while(1)
    {
        ;
    }
}

/******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/


