/*
 * ssctrl.h
 *
 *  Created on: 29-May-2026
 *      Author: 13197
 */

#ifndef INCLUDE_SSCTRL_H_
#define INCLUDE_SSCTRL_H_
#include "Mcal.h"
#include "Clock_Ip.h"
#include "Port_Ci_Port_Ip_Cfg.h"
#include "Ftm_Pwm_Ip.h"
#include "Port_Ci_Port_Ip.h"
#include "S32K146.h"
#include "Lpuart_Uart_Ip.h"
#include "Gpio_Dio_Ip.h"
#include "IntCtrl_Ip.h"
#include "OsIf.h"
#include "FlexCAN_Ip.h"
#include "LPit_Gpt_Ip.h"
#include <stdio.h>
#include <string.h>

#define FTM_INSTANCE_0		(uint8_t)0U
#define PWM_FTM_INSTANCE    (FTM_INSTANCE_0)
#define TIME_TYPE 			OSIF_COUNTER_SYSTEM
#define MSG_LEN  			8
#define UART0_INST   		LPUART_UART_IP_INSTANCE_USING_0

#define PORTD_BASE			IP_PTD
#define PORTB_BASE			IP_PTB
#define PORTA_BASE			IP_PTA
#define Blue_Led			0U
#define Green_Led			16U
#define Red_Led				15U
#define OPS_PIN				7U
#define RELEASE_LUMB		0U
#define RELEASE_LUMB_V2		1U
#define RELEASE_SUSP		3U
#define SUSP_AIR_F			13U
#define LUMB_AIR_F			12U

#define RX_MB_IDX                  (0U)
#define TX_MB_IDX                  (1U)
#define FLEXCAN_NUMBER_OF_MSG      (1U)
#define TIMEOUT_VALUE              (1000U)
#define RX_BUFFER_SIZE              (8U)

#define LPIT_INST_0 0U
/* LPIT Channel used - 0 */
#define CH_0 0U
/* LPIT time-out period - equivalent to 1s */
#define LPIT_PERIOD  8000000


extern volatile boolean bRxFlag;
extern Flexcan_Ip_MsgBuffType aRxDataBuffer[FLEXCAN_NUMBER_OF_MSG];
typedef enum
{
    MOTOR_PWM_CH0 = 0u,
    MOTOR_PWM_CH1 = 1u,
   // MOTOR_PWM_CH2 = 2u,
   // MOTOR_PWM_CH3 = 3u,
   // MOTOR_PWM_CH4 = 5u,
    MOTOR_PWM_COUNT
} MotorPwmChannel_t;


Lpuart_Uart_Ip_StatusType Read_Distance(void);
uint8_t Read_ops(void);
Lpuart_Uart_Ip_StatusType lidar_SendCommand(void);
Ftm_Pwm_Ip_StatusType Motor_SetDutyPercent_susp_lumb(MotorPwmChannel_t motorIndex, uint8_t dutyPercent);
boolean init(void);
void Can0_Init(void);
void uart0_Init(void);
void release_Lumbard(void);
void release_Lumbard_V2(void);
void release_suspension(void);
void Fill_suspension_air(void);
void Fill_lumbard_air(void);
void ProcessRxData(void);

void ProcessRxData1(void);
void ProcessRxData2(void);

void CAN_send_distance(void);
void CAN_RX_WaitComplete(void);
void CAN_RX_ArmReceive(uint8 bufferIndex);
void CAN_send(char *txdata,uint8_t len);
void OsIfDelay(uint32 timeoutMs);


#endif /* INCLUDE_SSCTRL_H_ */
