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

/**
 *******************************************************************************
 ** \brief OPA 三个反向级联
 **
 ******************************************************************************/
void OPA_ThreeOppMode_Test(void)
{
	  stc_opa_gain_config_t strGain0,strGain1,strGain2;
	
	  DDL_ZERO_STRUCT(strGain0);
	  DDL_ZERO_STRUCT(strGain1);
	  DDL_ZERO_STRUCT(strGain2);
	  strGain0.enInGain =  Gain01;  
	  strGain1.enInGain =  Gain01;
	  strGain2.enInGain =  Gain01;      
	  OP0_INP();
	  OP0_INN();
	  OP1_INP();
	  OP2_INP();
	  OP2_OUT();
	  OPA_ThreeOperate(OpaThreeOppMode,&strGain0,&strGain1,&strGain2);
}
/**
 *******************************************************************************
 ** \brief OPA 三个正向级联
 **
 ******************************************************************************/
void OPA_ThreeForMode_Test(void)
{
	  stc_opa_gain_config_t strGain0,strGain1,strGain2;
	
	  DDL_ZERO_STRUCT(strGain0);
	  DDL_ZERO_STRUCT(strGain1);
	  DDL_ZERO_STRUCT(strGain2);
	  strGain0.enNoInGain =  Gain04_3;  
	  strGain1.enNoInGain =  Gain04_3;
	  strGain2.enNoInGain =  Gain02;      //PGA = 4/3*4/3*2 = 3.55
	  OP0_INP();
	  OP1_INP();
	  OP2_INP();
	  OP2_OUT();
	  OPA_ThreeOperate(OpaThreeForMode,&strGain0,&strGain1,&strGain2);
}
/**
 *******************************************************************************
 ** \brief OPA 仪表模式
 **
 ******************************************************************************/
void OPA_MeterMode_Test(void)
{
	  en_opa_metergain_t strGain;
	
    strGain = OpaMeterGain1;  //PGA = 1

	  OP0_INP();
	  OP1_INP();
	  OP2_OUT();
	  OPA_MeterOperate(strGain);
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
 	  
	  DDL_ZERO_STRUCT(stcButtonPort);
    //PD04设置为GPIO<--SW2控制脚
    stcButtonPort.enDrv  = GpioDrvH;
    stcButtonPort.enDir  = GpioDirIn;
    Gpio_Init(Button_PORT, Button_PIN, &stcButtonPort);  
	  
	  OPA_Init(); 

      OPA_ThreeOppMode_Test();
//		OPA_ThreeForMode_Test();
//		OPA_MeterMode_Test();
		
    while (TRUE == Gpio_GetInputIO(Button_PORT,Button_PIN));
	  OPA_DeInit(); 
    while (1);
}

/******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/


