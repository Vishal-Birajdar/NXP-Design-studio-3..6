/*==================================================================================================
* Project : RTD AUTOSAR 4.7
* Platform : CORTEXM
* Peripheral : S32K14X
* Dependencies : none
*
* Autosar Version : 4.7.0
* Autosar Revision : ASR_REL_4_7_REV_0000
* Autosar Conf.Variant :
* SW Version : 3.0.0
* Build Version : S32K1_RTD_3_0_0_D2503_ASR_REL_4_7_REV_0000_20250307
* Code  Vishal Birajdar
* project interfacing
* 1) PWM for suspension and Lumbard
* 2) Uart for Distance
* 3) 1-GPIO Input for magnetic sensor
* 4) 2-GPIO Output for releasing air
* Copyright 2020-2025 NXP
*
* NXP Confidential and Proprietary. This software is owned or controlled by NXP and may only be 
*   used strictly in accordance with the applicable license terms.  By expressly 
*   accepting such terms or by downloading, installing, activating and/or otherwise 
*   using the software, you are agreeing that you have read, and that you agree to 
*   comply with and are bound by, such license terms.  If you do not agree to be 
*   bound by the applicable license terms, then you may not retain, install,
*   activate or otherwise use the software.
==================================================================================================*/

/**
*   @file main.c
*
*   @addtogroup main_module main module documentation
*   @{
*/

/* Including necessary configuration files. */
//#include "Mcal.h"
//#include "Clock_Ip.h"
//#include "Port_Ci_Port_Ip_Cfg.h"
//#include "Ftm_Pwm_Ip.h"
//#include "Port_Ci_Port_Ip.h"
//#include "S32K146.h"
//#include "Lpuart_Uart_Ip.h"
//#include "Gpio_Dio_Ip.h"
//#include "IntCtrl_Ip.h"
//#include "OsIf.h"
#include "ssctrl.h"
volatile int exit_code = 0;
/* User includes */

//uint8_t TF_Distance[5] = {0x55, 0xAA, 0x81,0,0xFA};
// uint8 rxResp[8] = {0};
// uint16_t distance = 0;
// uint8_t st = 0;


volatile boolean bRxFlag = FALSE;
//Flexcan_Ip_MsgBuffType aRxDataBuffer[FLEXCAN_NUMBER_OF_MSG];
/*
 * Init all uart lidar and gpio ,PWm
 */

/*void uart0_Init(void)
{
	Lpuart_Uart_Ip_Init(LPUART_UART_IP_INSTANCE_USING_0, &Lpuart_Uart_Ip_xHwConfigPB_0);
	OsIfDelay(200);
	Lpuart_Uart_Ip_AsyncReceive(LPUART_UART_IP_INSTANCE_USING_0, rxResp, 8);
}*/

/*
 * read disance
 */

int main(void)
{
	uint8_t ops_status;
	Lpuart_Uart_Ip_StatusType uart_st;
	uint32_t start_time = 0;
	uint32_t duration_time = 0;
    /* Write your code here */
	init();
	uart0_Init();
	 Can0_Init();
	 CAN_RX_ArmReceive(0U);
	 CAN_RX_WaitComplete();
    while(1)
    {

    	uart_st = Read_Distance();
    	if(uart_st == LPUART_UART_IP_STATUS_SUCCESS)
    	{
    		OsIfDelay(10);
    		CAN_send_distance();
    	}
    	ProcessRxData2();
    	/*Fill_suspension_air();
    	if(bRxFlag == TRUE)
    	{
    		bRxFlag = FALSE;
    		Gpio_Dio_Ip_WritePin(IP_PTD,15U,0);
    		ProcessRxData();
    		//Fill_suspension_air();
    	}*/
    	/*Read_Distance();
    	OsIfDelay(100);
    	Motor_SetDutyPercent_susp_lumb(MOTOR_PWM_CH1, 20);
    	OsIfDelay(100);
    	ops_status = Read_ops();
    	if(ops_status == 1)
    	{
    		//release();
    		Gpio_Dio_Ip_WritePin(IP_PTD, Green_Led,1);
    	}
    	else
    	{
    		Gpio_Dio_Ip_WritePin(IP_PTD, Green_Led,0);
    	}
    	release_Lumbard();*/

    	OsIfDelay(200);

    }
    return exit_code;
}



