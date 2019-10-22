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
#include "sysctrl.h"
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
static void _UserKeyWait(void)
{
    stc_gpio_config_t pstcGpioCfg;
    
    ///< 端口方向配置->输出
    pstcGpioCfg.enDir = GpioDirIn;
    ///< 端口驱动能力配置->低驱动能力
    pstcGpioCfg.enDrv = GpioDrvL;
    ///< 端口上下拉配置->上拉
    pstcGpioCfg.enPuPd = GpioPu;
    ///< 端口开漏输出配置->开漏输出关闭
    pstcGpioCfg.enOD = GpioOdDisable;
    ///< GPIO IO PD04初始化(PD04在STK上外接KEY(USER))
    Gpio_Init(GpioPortD, GpioPin4, &pstcGpioCfg);
    
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
 ** check Pxx to verify the clock frequency.
 **
 ******************************************************************************/
int32_t main(void)
{
    stc_sysctrl_clk_config_t stcCfg;
    stc_gpio_config_t pstcGpioCfg;
    
    ///< 开启GPIO外设时钟
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE);
    _UserKeyWait(); //USER KEY按下后继续执行程序
    
    ///< 因要使用的时钟源HCLK小于24M：此处设置FLASH 读等待周期为0 cycle(默认值也为0 cycle)
    Flash_WaitCycle(FlashWaitCycle0);
    
    ///< 时钟初始化前，优先设置要使用的时钟源：此处设置RCH为4MHz（默认值为4MHz）
    Sysctrl_SetRCHTrim(SysctrlRchFreq4MHz);
    
    ///< 选择内部RCH作为HCLK时钟源;
    stcCfg.enClkSrc    = SysctrlClkRCH;
    ///< HCLK SYSCLK/2
    stcCfg.enHClkDiv   = SysctrlHclkDiv2;
    ///< PCLK 为HCLK/8
    stcCfg.enPClkDiv   = SysctrlPclkDiv8;
    ///< 系统时钟初始化
    Sysctrl_ClkInit(&stcCfg);
    

    ///< 端口方向配置->输出
    pstcGpioCfg.enDir = GpioDirOut;
    ///< 端口驱动能力配置->高驱动能力
    pstcGpioCfg.enDrv = GpioDrvH;
    ///< 端口上下拉配置->无上下拉
    pstcGpioCfg.enPuPd = GpioNoPuPd;
    ///< 端口开漏输出配置->开漏输出关闭
    pstcGpioCfg.enOD = GpioOdDisable;
    ///< GPIO IO PB00初始化
    Gpio_Init(GpioPortB, GpioPin0, &pstcGpioCfg);
    ///< GPIO IO PA01初始化
    Gpio_Init(GpioPortA, GpioPin1, &pstcGpioCfg);
    ///< GPIO IO PB01初始化
    Gpio_Init(GpioPortB, GpioPin1, &pstcGpioCfg);
    
    ///< 设置RCH从PB00输出
    Gpio_SetAfMode(GpioPortB, GpioPin0, GpioAf5);
    
    ///< 设置HCLK从PA01输出
    Gpio_SfHClkOutputConfig(GpioSfHclkOutEnable, GpioSfHclkOutDiv1);
    Gpio_SetAfMode(GpioPortA, GpioPin1, GpioAf6);
    
    ///< 设置PCLK从PB01输出
    Gpio_SfPClkOutputConfig(GpioSfPclkOutEnable, GpioSfPclkOutDiv1);
    Gpio_SetAfMode(GpioPortB, GpioPin1, GpioAf2);
    while (1);

}

/******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/



