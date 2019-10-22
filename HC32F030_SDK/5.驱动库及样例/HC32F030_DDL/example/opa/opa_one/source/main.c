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
#include "gpio.h"
#include "opa.h"
#include "adc.h"
/******************************************************************************
 * Local pre-processor symbols/macros ('#define')
 ******************************************************************************/
 
/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/
#define Button_PORT  GpioPortD
#define Button_PIN   GpioPin4
/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/

#define OP0_INP()         { 	M0P_GPIO->PCADS_f.PC06 = 1;	M0P_GPIO->PCDIR_f.PC06 = 1;}
#define OP0_INN()         { 	M0P_GPIO->PBADS_f.PB15 = 1;	M0P_GPIO->PBDIR_f.PB15 = 1;}
#define OP0_OUT()         { 	M0P_GPIO->PCADS_f.PC07 = 1;	M0P_GPIO->PCDIR_f.PC07 = 1;}

#define OP1_INP()         { 	M0P_GPIO->PBADS_f.PB13 = 1;	M0P_GPIO->PBDIR_f.PB13 = 1;}
#define OP1_INN()         { 	M0P_GPIO->PBADS_f.PB12 = 1;	M0P_GPIO->PBDIR_f.PB12 = 1;}
#define OP1_OUT()         { 	M0P_GPIO->PBADS_f.PB14 = 1;	M0P_GPIO->PBDIR_f.PB14 = 1;}

#define OP2_INP()         { 	M0P_GPIO->PBADS_f.PB10 = 1;	M0P_GPIO->PBDIR_f.PB10 = 1;}
#define OP2_INN()         { 	M0P_GPIO->PBADS_f.PB02 = 1;	M0P_GPIO->PBDIR_f.PB02 = 1;}
#define OP2_OUT()         { 	M0P_GPIO->PBADS_f.PB11 = 1;	M0P_GPIO->PBDIR_f.PB11 = 1;}


/******************************************************************************
 * Local variable definitions ('static')                                      *
 ******************************************************************************/
static stc_adc_irq_t stcAdcIrqFlag;

uint16_t u16AdcRestult0;
/*****************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/

void AdcContIrqCallback(void)
{    
    Adc_GetSglResult(&u16AdcRestult0);
  
    stcAdcIrqFlag.bAdcIrq = TRUE;
}
/**
 *******************************************************************************
 ** \brief OPA 单位增益 
 **
 ******************************************************************************/
void OPA_UintMode_Test(void)
{
	  stc_opa_gain_config_t strGain;
	
	  DDL_ZERO_STRUCT(strGain);
	  OP0_INP();
	  OP0_OUT();
	  OPA_Operate(OPA0 ,OpaUintMode,&strGain);   //单位
}

/**
 *******************************************************************************
 ** \brief OPA 正向增益 
 **
 ******************************************************************************/
void OPA_ForWardMode_Test(void)
{
	  stc_opa_gain_config_t strGain;
	
	  DDL_ZERO_STRUCT(strGain);
	  strGain.enNoInGain =  Gain02;  //正向增益 PGA = 2
	  OP0_INP();
	  OP0_OUT();
	  OPA_Operate(OPA0 ,OpaForWardMode,&strGain);   //正向增益
}
/**
 *******************************************************************************
 ** \brief OPA 反向增益 
 **
 ******************************************************************************/
void OPA_OppositeMode_Test(void)
{
	  stc_opa_gain_config_t strGain;
	
	  DDL_ZERO_STRUCT(strGain);
	  strGain.enInGain =  Gain03;  //反向增益 PGA = 3
	  OP0_INP();
	  OP0_INN();
	  OP0_OUT();
	  OPA_Operate(OPA0 ,OpaOppositeMode,&strGain);   //反向增益
}
/**
 *******************************************************************************
 ** \brief OPA 通用OPA
 **
 ******************************************************************************/
void OPA_GpMode_Test(void)
{
	  stc_opa_gain_config_t strGain;
	
	  DDL_ZERO_STRUCT(strGain);
	  OP0_INP();
	  OP0_INN();
	  OP0_OUT();
	  OPA_Operate(OPA0 ,OpaGpMode,&strGain);   //
}
/**
 *******************************************************************************
 ** \brief OPA 外部链接ADC
 **
 ******************************************************************************/
void OPA_ADCMode_Test(void)
{
    stc_adc_cfg_t              stcAdcCfg;
    stc_adc_irq_t              stcAdcIrq;
    stc_adc_irq_calbakfn_pt_t  stcAdcIrqCalbaks;
	
	  DDL_ZERO_STRUCT(stcAdcCfg);
    DDL_ZERO_STRUCT(stcAdcIrq);
    DDL_ZERO_STRUCT(stcAdcIrqCalbaks);
    DDL_ZERO_STRUCT(stcAdcIrqFlag);
	  
	  Sysctrl_SetPeripheralGate(SysctrlPeripheralAdcBgr, TRUE);
	
	
	  //ADC配置
    Adc_Enable();
    M0P_BGR->CR_f.BGR_EN = 0x1u;                 //BGR必须使能
    M0P_BGR->CR_f.TS_EN = 0x0u;
    delay100us(1);
    
	  OPA_UintMode_Test();
	
    stcAdcCfg.enAdcOpMode = AdcSCanMode;         //扫描模式
    stcAdcCfg.enAdcClkDiv = AdcClkSysTDiv1;
    stcAdcCfg.enAdcSampTimeSel = AdcSampTime8Clk;
    stcAdcCfg.enAdcRefVolSel   = RefVolSelAVDD;
    stcAdcCfg.bAdcInBufEn      = FALSE;
    
    Adc_Init(&stcAdcCfg);
    
    Adc_ConfigSglChannel(AdcOPA0Input);  //OPA0_OUT
    
    EnableNvic(ADC_IRQn, (en_irq_level_t)3, TRUE);
    
    Adc_EnableIrq();
    
    stcAdcIrq.bAdcIrq = TRUE;
    stcAdcIrqCalbaks.pfnAdcIrq = AdcContIrqCallback;
    
    Adc_ConfigIrq(&stcAdcIrq, &stcAdcIrqCalbaks);
      
    Adc_SGL_Start();

    while(1)
    {        
        while(FALSE == stcAdcIrqFlag.bAdcIrq);

        delay1ms(1000);
        
        Adc_SGL_Start();
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

    stc_gpio_config_t         stcButtonPort;

     
	  Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE);
    Sysctrl_SetPeripheralGate(SysctrlPeripheralOpa, TRUE);
 	  
	  Sysctrl_SetPeripheralGate(SysctrlPeripheralAdcBgr, TRUE);
	  
	
	  DDL_ZERO_STRUCT(stcButtonPort);
    //PD04设置为GPIO<--SW2控制脚
    stcButtonPort.enDrv  = GpioDrvH;
    stcButtonPort.enDir  = GpioDirIn;
    Gpio_Init(Button_PORT, Button_PIN, &stcButtonPort);  
	  
	  OPA_Init(); 

	  OPA_UintMode_Test();
//  	  OPA_ForWardMode_Test();
//    OPA_OppositeMode_Test();
//       OPA_GpMode_Test();
//		while (TRUE == Gpio_GetInputIO(Button_PORT,Button_PIN));
//	  OPA_DeInit();		
		
		
		
//		OPA_ADCMode_Test();
    while (1)
		{   
		}
}

/******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/


