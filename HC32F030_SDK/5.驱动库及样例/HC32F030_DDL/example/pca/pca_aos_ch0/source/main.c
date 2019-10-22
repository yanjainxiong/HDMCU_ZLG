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
#include "pca.h"
#include "lpm.h"
#include "gpio.h"
#include "vc.h"

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

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/
static volatile uint32_t u32PcaTestFlag = 0;
static volatile uint16_t u16CapData;
static volatile uint16_t u16CntData;

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/


/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/
/*******************************************************************************
 * Pca中断服务程序
 ******************************************************************************/
void PcaInt(void)
{
    if (TRUE == Pca_GetCntIntFlag())
    {
        Pca_ClearCntIntFlag();
        u32PcaTestFlag |= 0x20;
    }
    if (TRUE == Pca_GetIntFlag(Module0))
    {
        Pca_ClearIntFlag(Module0);
        u32PcaTestFlag |= 0x01;
        
        u16CntData = Pca_Cnt16Get();
        u16CapData = Pca_CapData16Get(Module0);
        Pca_Stop();
        Pca_Cnt16Set(0);
        Pca_Run();
    }
    if (TRUE == Pca_GetIntFlag(Module1))
    {
        Pca_ClearIntFlag(Module1);
        u32PcaTestFlag |= 0x02;
    }
    if (TRUE == Pca_GetIntFlag(Module2))
    {
        Pca_ClearIntFlag(Module2);
        u32PcaTestFlag |= 0x04;
    }
    if (TRUE == Pca_GetIntFlag(Module3))
    {
        Pca_ClearIntFlag(Module3);
        u32PcaTestFlag |= 0x08;
    }
    if (TRUE == Pca_GetIntFlag(Module4))
    {
        Pca_ClearIntFlag(Module4);
        u32PcaTestFlag |= 0x10;
    }
        
}

/*******************************************************************************
 * VC1 初始化配置程序
 ******************************************************************************/
void VC1_Config(void)
{
    stc_gpio_config_t        stcVC1InPort;
    stc_gpio_config_t        stcVC1OutPort;
    stc_vc_channel_config_t  stcChannelConfig;
    
    DDL_ZERO_STRUCT(stcChannelConfig);
    DDL_ZERO_STRUCT(stcVC1InPort);
    DDL_ZERO_STRUCT(stcVC1OutPort);
    
    Sysctrl_SetPeripheralGate(SysctrlPeripheralVcLvd, TRUE);
        
    //PB14设置为VC1_IN
    //Gpio_ClrAnalogMode(GpioPortB, GpioPin14);
    stcVC1InPort.enDir  = GpioDirIn;
    Gpio_Init(GpioPortB, GpioPin14, &stcVC1InPort);
    
    //PA07设置为VC1_OUT
    //Gpio_ClrAnalogMode(GpioPortA, GpioPin7);
    stcVC1OutPort.enDir  = GpioDirOut;
    Gpio_Init(GpioPortA, GpioPin7, &stcVC1OutPort);
    Gpio_SetAfMode(GpioPortA,GpioPin7,GpioAf6);
    
    stcChannelConfig.enVcChannel      = VcChannel1;    //通道1
    stcChannelConfig.enVcCmpDly       = VcDelay10mv;   //迟滞电压10mv
    stcChannelConfig.enVcBiasCurrent  = VcBias1200na;  //偏置电流1.2ua
    stcChannelConfig.enVcFilterTime   = VcFilter14us;  //输出滤波时间14us
    stcChannelConfig.enVcInPin_P      = VcInPCh11;     //VC1_IN_P 选择PB14
    stcChannelConfig.enVcInPin_N      = AiLdo;         //VC1_IN_N 选择内部LDO
    
    Vc_ChannelInit(VcChannel1, &stcChannelConfig);     //VC1初始化
    
    Vc_EnableFilter(VcChannel1);                       //滤波使能
    
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
    stc_pca_config_t        stcConfig;
    stc_pca_capmodconfig_t  stcModConfig;
    uint16_t                u16InitCntData = 0;
    
    DDL_ZERO_STRUCT(stcConfig);
    DDL_ZERO_STRUCT(stcModConfig);
    
    //使能PCA,GPIO外设时钟
    Sysctrl_SetPeripheralGate(SysctrlPeripheralPca, TRUE);
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE);

    Gpio_SfPcaConfig(GpioSfPcaCH0, GpioSf4);    //PCA_CH0 捕获选择VC1_OUT
    
    stcConfig.enCIDL = IdleGoOn;                  //休眠模式PCA工作 
    stcConfig.enWDTE = PCAWDTDisable;             //wdt功能关闭
    stcConfig.enCPS  = PCAPCLKDiv16;              //PCLK/16,
    
    stcConfig.pfnPcaCb = PcaInt;                  //中断函数入口
    
    stcModConfig.enECOM     = ECOMDisable;        //比较器功能禁止
    stcModConfig.enCAPP     = CAPPEnable;         //上升沿捕获允许
    stcModConfig.enCAPN     = CAPNEnable;         //下降沿捕获允许
    stcModConfig.enMAT      = MATDisable;         //禁止匹配
    stcModConfig.enTOG      = TOGDisable;         //禁止翻转
    stcModConfig.en8bitPWM  = PCA8bitPWMDisable;  //禁止8位PWM功能
    
    
    if (Ok != Pca_Init(&stcConfig))
    {
        return Error;
    }
    if (Ok != Pca_CapModConfig(Module0, &stcModConfig))
    {
        return Error;
    }
    
    //INT ENABLE
    Pca_EnableIrq(Module0);    
    EnableNvic(PCA_IRQn, IrqLevel3, TRUE);

    
    Pca_Cnt16Set(u16InitCntData);
    Pca_Run();
    
    VC1_Config();
    
    while(1);
    
    
}

/******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/


