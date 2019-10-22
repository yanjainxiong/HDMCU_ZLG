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
#include "vc.h"
#include "gpio.h"
#include "adc.h"
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

/******************************************************************************
 * Local variable definitions ('static')                                      *
 ******************************************************************************/
/******************************************************************************
 * Local pre-processor symbols/macros ('#define')
 ******************************************************************************/

/*****************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/
boolean_t bPortVal = 0;
void VcIrqCallback(void)
{

    Vc_ClearIrq(VcChannel0);
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
void Vc_PortInit(void)
{
    stc_gpio_config_t stcGpioCfg;
    DDL_ZERO_STRUCT(stcGpioCfg);
    stcGpioCfg.enDir = GpioDirOut;
    Gpio_Init(GpioPortA,GpioPin6,&stcGpioCfg);
    Gpio_SetAfMode(GpioPortA,GpioPin6,GpioAf5);//VC输出
    Gpio_WriteOutputIO(GpioPortD,GpioPin5,0);
    Gpio_Init(GpioPortD,GpioPin5,&stcGpioCfg);
    stcGpioCfg.enDir = GpioDirIn;
    Gpio_Init(GpioPortD,GpioPin4,&stcGpioCfg);
    Gpio_SetAnalogMode(GpioPortC,GpioPin0);//模拟输入
    Gpio_SetAnalogMode(GpioPortA,GpioPin0);//模拟输入
}
int main(void)
{
    stc_vc_channel_config_t stcChannelConfig;
    stc_lpm_config_t stcLpmConfig;

    DDL_ZERO_STRUCT(stcChannelConfig);//变量清0
    DDL_ZERO_STRUCT(stcLpmConfig);//变量清0

    if (Ok != Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE))//开GPIO时钟
    {
        return 1;
    }
    if (Ok != Sysctrl_SetPeripheralGate(SysctrlPeripheralVcLvd, TRUE))//开LVD时钟
    {
        return 1;
    } 
    if (Ok != Sysctrl_SetPeripheralGate(SysctrlPeripheralAdcBgr, TRUE))//开adc时钟
    {
        return 1;
    } 	
    M0P_BGR->CR_f.BGR_EN = 0x1u;                 //BGR必须使能
    M0P_BGR->CR_f.TS_EN = 0x0u;
    delay100us(1);
    
    //配置测试IO口
    Vc_PortInit();
    
    stcLpmConfig.enSLEEPDEEP = SlpDpEnable;
    stcLpmConfig.enSLEEPONEXIT = SlpExtDisable;
    
    stcChannelConfig.enVcChannel = VcChannel0;
    stcChannelConfig.enVcCmpDly  = VcDelay10mv;
    stcChannelConfig.enVcFilterTime = VcFilter28us;
    stcChannelConfig.enVcInPin_P = VcInPCh0;
    stcChannelConfig.enVcInPin_N = VcInNCh0;
    stcChannelConfig.enVcIrqSel = VcIrqHigh;
    stcChannelConfig.enVcOutConfig = VcOutDisable;
    stcChannelConfig.pfnAnalogCmpCb = VcIrqCallback;
    Vc_EnableFilter(VcChannel0);
    Vc_ChannelInit(VcChannel0,&stcChannelConfig);
    Vc_EnableIrq(VcChannel0);
    Vc_EnableChannel(VcChannel0);
    Lpm_Config(&stcLpmConfig);
    while(1 == Gpio_GetInputIO(GpioPortD,GpioPin4))
    {;}
    Lpm_GotoLpmMode();
    Gpio_WriteOutputIO(GpioPortD,GpioPin5,1);//唤醒后点亮LED3
    while (1);
}

/******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/


