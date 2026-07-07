/*
 * ssctrl.c
 *
 *  Created on: 29-May-2026
 *      Author: 13197
 */
#include "ssctrl.h"

uint8_t g_targetHeight = 0;         // Persistent target for suspension
static uint8_t g_prevLumbarVal = 0; // Remember last lumbar position
// static boolean g_lumbarFilling = FALSE;
// static boolean g_lumbarReleasing = FALSE;
volatile uint8_t currentVal = 0;
uint32_t g_lumbarStartTime = 0U; // Snapshot of time when action started
uint32_t g_lumbarDurationTicks = 0U;
int8_t delta;
uint8_t abs_delta;

static uint32_t g_lastAliveTick = 0;
// bool     g_emergencyTimerActive = FALSE;
// uint8_t  g_emergencyActivePort = 0;
// uint32_t g_emergencyTimerEndTick = 0;
const uint32_t ALIVE_INTERVAL_TICKS = 9000000U;

Clock_Ip_StatusType clockStatus;
Port_Ci_Port_Ip_PortStatusType portStatus;
IntCtrl_Ip_StatusType intctrlStatus;
volatile Lpuart_Uart_Ip_StatusType lpuartStatus;

volatile uint8 u8TxConfirmCnt = 0U;
volatile uint8 u8RxIndicationCnt = 0U;
boolean bTxFlag = FALSE;

// uint32 u32TimeOut = 0U;
Flexcan_Ip_StatusType status;
Flexcan_Ip_StatusType initstatus;
Flexcan_Ip_StatusType Rxstatus;
Flexcan_Ip_StatusType Rcstatus;
volatile uint8 u8RxWriteIdx = 0U;
volatile uint8 u8RxReadIdx = 0U;
volatile boolean bNewDataAvailable = FALSE;
uint8 rcData1 = 0U;
uint8 rcData2 = 0U;
uint8 rcData3 = 0U;
uint8 rcData4 = 0U;

Flexcan_Ip_MsgBuffType aRxDataBuffer[FLEXCAN_NUMBER_OF_MSG];

uint8_t cmd = 0;
uint8_t sub = 0;
uint8_t val = 0;
char TxData[5];
uint8_t TF_Distance[5] = {0x55, 0xAA, 0x81, 0, 0xFA};
uint8 rxResp[8] = {0};
uint8 aliveData[8] = {0};
volatile uint16_t distance = 0;
uint16_t prev_distance = 0;
uint8_t st = 0;

Flexcan_Ip_DataInfoType TxInfo = {
    .msg_id_type = FLEXCAN_MSG_ID_STD,
    .data_length = 8U,
    .is_polling = TRUE,
    .is_remote = FALSE};
Flexcan_Ip_DataInfoType RxInfo = {
    .msg_id_type = FLEXCAN_MSG_ID_STD,
    .data_length = 8U,
    .is_polling = FALSE,
    .is_remote = FALSE

};

void FlexCAN_UserCallback(uint8 instance, Flexcan_Ip_EventType eventType, uint32 buffIdx, const struct FlexCANState *driverState)
{
    // uint8 u8NextIdx = 0U;

    /*------------------------------------------------------------------
     * RX COMPLETE EVENT
     * Data already stored in aRxBuffer[u8RxWriteIdx] by driver
     *-----------------------------------------------------------------*/
    if (FLEXCAN_EVENT_RX_COMPLETE == eventType)
    {

        bRxFlag = TRUE;
        if (bRxFlag == TRUE)
        {
            Gpio_Dio_Ip_WritePin(IP_PTD, 15U, 1);
        }

        (void)FlexCAN_Ip_Receive(INST_FLEXCAN_0,
                                 RX_MB_IDX,
                                 &aRxDataBuffer[0],
                                 FALSE);

        /* Step 4: Notify main() */
        // bNewDataAvailable = TRUE;
        cmd = aRxDataBuffer->data[0];
        sub = aRxDataBuffer->data[1];
        val = aRxDataBuffer->data[3];
    }

    (void)instance;
    (void)buffIdx;
    (void)driverState;
}

/* Global variables to track the active timed pin */
static uint32_t g_TimedPort = 0;
static uint16_t g_TimedPin = 0;

void LpitNotification(void)
{
    /* TIME IS UP! Turn off the valve we were timing */
    if (g_TimedPort != 0)
    {
        Gpio_Dio_Ip_WritePin(g_TimedPort, g_TimedPin, 0);
    }

    /* Stop the timer and clear state */
    Lpit_Gpt_Ip_StopTimer(0U, 0U);
    g_TimedPort = 0;
    g_TimedPin = 0;
}

void OsIfDelay(uint32 timeoutMs)
{
    uint32 curTime = 0u;
    uint32 endTime = 0u;
    uint32 timeoutCnt = 0u;

    curTime = OsIf_GetCounter(OSIF_COUNTER_SYSTEM);
    timeoutCnt = OsIf_MicrosToTicks(timeoutMs * 1000u, TIME_TYPE);

    while (1)
    {
        endTime += OsIf_GetElapsed(&curTime, TIME_TYPE);

        if (timeoutCnt <= endTime)
        {
            break;
        }
    }
}

boolean init(void)
{
    /* 1) Init clocks */
    clockStatus = Clock_Ip_Init(&Clock_Ip_aClockConfig[0]); /* generated */

    /* 2) Init pins (Port driver) */
    portStatus = Port_Ci_Port_Ip_Init(NUM_OF_CONFIGURED_PINS_PortContainer_0_BOARD_InitPeripherals,
                                      g_pin_mux_InitConfigArr_PortContainer_0_BOARD_InitPeripherals);

    /* 4) Init PWM driver (FTM0) */
    Ftm_Pwm_Ip_Init(PWM_FTM_INSTANCE, &Ftm_Pwm_Ip_UserCfg0); /* generated user cfg */
    intctrlStatus = IntCtrl_Ip_Init(&IntCtrlConfig_0);
    OsIf_Init(NULL_PTR);
    if (clockStatus != CLOCK_IP_SUCCESS && portStatus != PORT_CI_PORT_SUCCESS && intctrlStatus != INTCTRL_IP_STATUS_SUCCESS)
    {
        return FALSE;
    }
    Lpit_Gpt_Ip_Init(LPIT_INST_0, &LPIT_0_InitConfig_PB);
    /* Initial channel 0 */
    Lpit_Gpt_Ip_InitChannel(LPIT_INST_0, LPIT_0_CH_0);
    /* Enable channel interrupt LPIT_0 - CH_0 */
    Lpit_Gpt_Ip_EnableChInterrupt(LPIT_INST_0, CH_0);
    /* Start channel CH_0 */
    // Lpit_Gpt_Ip_StartTimer(LPIT_INST_0, CH_0, LPIT_PERIOD);
    return TRUE;
}

void uart0_Init(void)
{
    // Lpuart_Uart_Ip_StatusType lpuart_status;
    Lpuart_Uart_Ip_Init(LPUART_UART_IP_INSTANCE_USING_0, &Lpuart_Uart_Ip_xHwConfigPB_0);
    OsIfDelay(200);
    // lpuart_status = Lpuart_Uart_Ip_AsyncReceive(LPUART_UART_IP_INSTANCE_USING_0, rxResp, 8);
    // return lpuart_status;
}

void Can0_Init()
{
    /* Initialize FlexCAN driver */
    initstatus = FlexCAN_Ip_Init(INST_FLEXCAN_0, &FlexCAN_State0, &FlexCAN_Config0);

    /* Set Rx filter mask type */
    FlexCAN_Ip_SetRxMaskType(INST_FLEXCAN_0, FLEXCAN_RX_MASK_INDIVIDUAL);

    /* Set Rx individual mask value */
    /* Expect to receive all IDs, mask = 0x0 */
    FlexCAN_Ip_SetRxIndividualMask(INST_FLEXCAN_0, RX_MB_IDX, 0x7FFU);

    /* Start FlexCAN controller */
    status = FlexCAN_Ip_SetStartMode(INST_FLEXCAN_0);

    /* Configure Rx message buffer */
    Rxstatus = FlexCAN_Ip_ConfigRxMb(INST_FLEXCAN_0, RX_MB_IDX, &RxInfo, 0x456U);
}

boolean TFLC02_ParseDistance(uint8_t *r, uint16_t *dist_mm, uint8_t *status)
{
    // char msg[32];
    //     int n;
    if ((r[0] != 0x55u) || (r[1] != 0xAAu))
        return FALSE;
    if (r[2] != 0x81u)
        return FALSE;
    if (r[3] != 0x03u)
        return FALSE;
    if (r[7] != 0xFAu)
        return FALSE;

    *dist_mm = ((uint16_t)r[4] << 8) | (uint16_t)r[5]; /* big-endian */
    *status = (uint8_t)r[6];                           /* 0x00 = valid */

    //    n = snprintf(msg, sizeof(msg), "Dist=%u mm\r\n", (unsigned)dist_mm);

    return TRUE;
}

Lpuart_Uart_Ip_StatusType lidar_SendCommand()
{
    Lpuart_Uart_Ip_SyncSend(LPUART_UART_IP_INSTANCE_USING_0, TF_Distance, 5, 500);
    OsIfDelay(35);
    return lpuartStatus;
}

Lpuart_Uart_Ip_StatusType Read_Distance()
{

    /* Example: if LiDAR responds with 9 bytes, set respLen = 9 */
    // boolean ok = Lpuart_SendCmd_RecvResp(UART_LPUART_INTERNAL_CHANNEL,TF_Distance, sizeof(TF_Distance),rxResp, 8);
    lidar_SendCommand();
    lpuartStatus = Lpuart_Uart_Ip_AsyncReceive(LPUART_UART_IP_INSTANCE_USING_0, rxResp, 8);
    OsIfDelay(100);
    distance = ((uint16_t)rxResp[4] << 8) | (uint16_t)rxResp[5];
    // dist_mm = (rxResp[4]<<8)| rxResp[5];
    // TFLC02_ParseDistance(rxResp,&distance,&st);
    // TFLC02_ParseDistance(rxResp);
    return lpuartStatus;
}

uint8_t Read_ops()
{
    Gpio_Dio_Ip_PinsLevelType status;

    status = Gpio_Dio_Ip_ReadPin(PORTD_BASE, OPS_PIN);
    return status;
}
Ftm_Pwm_Ip_StatusType Motor_SetDutyPercent_susp_lumb(MotorPwmChannel_t motorIndex, uint8_t dutyPercent)
{
    Ftm_Pwm_Ip_StatusType pwmStatus;
    // if (motorIndex >= 5U) return;
    // if (dutyPercent > 100U) dutyPercent = 100U;

    // uint32_t dutyTicks = ((uint32_t)g_pwmPeriodTicks * (uint32_t)dutyPercent) / 100U;

    pwmStatus = Ftm_Pwm_Ip_UpdatePwmChannel(PWM_FTM_INSTANCE, motorIndex, (uint16_t)dutyPercent, 0U, TRUE);
    return pwmStatus;
}

void CAN_RX_ArmReceive(uint8 bufferIndex)
{
    /* Arm MB0 to receive first message
     * When message arrives: ISR -> Callback -> Store data -> Re-arm */
    FlexCAN_Ip_Receive(INST_FLEXCAN_0,
                       RX_MB_IDX,
                       &aRxDataBuffer[bufferIndex],
                       FALSE); /* FALSE = interrupt mode */
}
void CAN_RX_WaitComplete(void)
{
    uint32 u32TimeOut = (uint32)(TIMEOUT_VALUE * 100U);

    /* Wait for all messages or timeout
     * bRxComplete is set in interrupt callback */
    while ((bRxFlag == FALSE) && (u32TimeOut > 0U))
    {
        /* Can add FlexCAN_Ip_MainFunctionRead() here
         * if you want polling fallback */
        u32TimeOut--;
    }
}
void CAN_send_distance(void)
{
    char buffer[8] = {0};
    // sprintf(buffer,"D:%hu",distance);
    snprintf(buffer, sizeof(buffer), "D:%hu", distance);
    memcpy(aliveData, buffer, sizeof(buffer));

    // aliveData[2] = 1;

    if (FlexCAN_Ip_GetTransferStatus(INST_FLEXCAN_0, TX_MB_IDX) != FLEXCAN_STATUS_BUSY)
    {
        status = FlexCAN_Ip_Send(INST_FLEXCAN_0,
                                 TX_MB_IDX,
                                 &TxInfo,
                                 0x456U,
                                 aliveData);

        if (status == FLEXCAN_STATUS_SUCCESS)
        {
            while (FlexCAN_Ip_GetTransferStatus(INST_FLEXCAN_0, TX_MB_IDX) == FLEXCAN_STATUS_BUSY)
            {
                FlexCAN_Ip_MainFunctionWrite(INST_FLEXCAN_0, TX_MB_IDX);
            }
        }
    }
    OsIfDelay(500);
}

void CAN_send(char *txdata, uint8_t len)
{

    // snprintf(TxData, sizeof(TxData), "V1:%u", 0);
    TxInfo.data_length = len;
    if (FlexCAN_Ip_GetTransferStatus(INST_FLEXCAN_0, TX_MB_IDX) != FLEXCAN_STATUS_BUSY)
    {
        status = FlexCAN_Ip_Send(INST_FLEXCAN_0,
                                 TX_MB_IDX,
                                 &TxInfo,
                                 0x456U,
                                 (uint8_t *)txdata);

        if (status == FLEXCAN_STATUS_SUCCESS)
        {
            while (FlexCAN_Ip_GetTransferStatus(INST_FLEXCAN_0, TX_MB_IDX) == FLEXCAN_STATUS_BUSY)
            {
                FlexCAN_Ip_MainFunctionWrite(INST_FLEXCAN_0, TX_MB_IDX);
            }
        }
    }
    // OsIfDelay(50);
}
/* These must be STATIC so they remember their values between function calls */

/* --- 1. Global/Static Variables (Maintain state between loops) --- */

void ProcessRxData(void)
{
    /* --- 2. CAN RECEIVE SECTION (Update Targets Only) ---
     * PTB 12 PORT B LUMBAR AIR FILL
     * PTB 13 PORT B SUSPENSION AIR FILL
     * PTA 0  PORT A RELEASE LUMBAR AIR V1
     * PTA 3  PORT A RELEASE SUSPENSION AIR
     * PTA 1  PORT A RELEASE LUMBAR AIR V2
     * */
    char aliveMsg[8];
    uint32_t currentTime = OsIf_GetCounter(OSIF_COUNTER_SYSTEM);
    uint32_t intervalTicks = OsIf_MicrosToTicks(ALIVE_INTERVAL_TICKS, OSIF_COUNTER_SYSTEM);
    int len = snprintf(aliveMsg, sizeof(aliveMsg), "HW%u:2", 2);
    /* --- 1. HEARTBEAT SECTION (Every 9 Seconds) --- */
    // If it's the first run, or 9 seconds have passed
    // uint32_t elaps = OsIf_GetElapsed(currentTime,OSIF_COUNTER_SYSTEM);

    if (g_lastAliveTick == 0 || (currentTime - g_lastAliveTick >= intervalTicks))
    {

        CAN_send(aliveMsg, len);

        g_lastAliveTick = currentTime; // Reset the timer
    }

    // uint32_t currentTime = OsIf_GetCounter(OSIF_COUNTER_SYSTEM);
    if (bRxFlag == TRUE)
    {
        bRxFlag = FALSE;
        Gpio_Dio_Ip_WritePin(IP_PTD, 15U, 0);
        if (NULL_PTR == aRxDataBuffer)
            return;

        uint8_t cmd = aRxDataBuffer->data[0];
        uint8_t sub = aRxDataBuffer->data[1];
        uint8_t val = aRxDataBuffer->data[3];

        // Command C1:1 Set Suspension Target
        if (cmd == 67 && sub == 49)
        {

            g_targetHeight = val;
            if (g_targetHeight > 180)
                g_targetHeight = 180;
            if (g_targetHeight < 60)
                g_targetHeight = 60;
        }
        // Command C2:2 Set Lumbar Target and Calculate Delta
        else if (cmd == 67 && sub == 50)
        {
            currentVal = val;
            if (currentVal != g_prevLumbarVal)
            {
                delta = (int8_t)currentVal - (int8_t)g_prevLumbarVal;
                abs_delta = (delta < 0) ? -delta : delta;

                // g_prevLumbarVal = currentVal; // Update history
                if (delta > 0)
                {
                    Gpio_Dio_Ip_WritePin(PORTB_BASE, LUMB_AIR_F, 1);
                    OsIfDelay(abs_delta * 1000);
                    Gpio_Dio_Ip_WritePin(PORTB_BASE, LUMB_AIR_F, 0);
                    Gpio_Dio_Ip_WritePin(PORTA_BASE, RELEASE_LUMB, 0);
                }
                else if (delta < 0)
                {
                    Gpio_Dio_Ip_WritePin(PORTB_BASE, LUMB_AIR_F, 0);
                    Gpio_Dio_Ip_WritePin(PORTA_BASE, RELEASE_LUMB, 1);
                    OsIfDelay(abs_delta * 1000);
                    Gpio_Dio_Ip_WritePin(PORTA_BASE, RELEASE_LUMB, 0);
                }
                else
                {
                    Gpio_Dio_Ip_WritePin(PORTB_BASE, LUMB_AIR_F, 0);
                    Gpio_Dio_Ip_WritePin(PORTA_BASE, RELEASE_LUMB, 0);
                }
                g_prevLumbarVal = currentVal;
            }
        }
        // Command V:1 Manual Emergency Stop 86 -> V
        else if (cmd == 86 && val == 1)
        {
            if (sub == 49) // 1 -> 49
            {
                int len = snprintf(TxData, sizeof(TxData), "V%u:0", 1);
                // g_targetHeight = distance;
                Gpio_Dio_Ip_WritePin(PORTB_BASE, SUSP_AIR_F, 0);
                Gpio_Dio_Ip_WritePin(PORTA_BASE, RELEASE_SUSP, 1);
                OsIfDelay(4000);
                Gpio_Dio_Ip_WritePin(PORTA_BASE, RELEASE_SUSP, 0);
                CAN_send(TxData, len);
            } // Stop suspension at current pos  2->50, 3->51 V:2
            else if (sub == 50)
            {
                int len = snprintf(TxData, sizeof(TxData), "V%u:0", 2);
                // g_lumbarFilling = FALSE;
                // g_lumbarReleasing = FALSE;
                Gpio_Dio_Ip_WritePin(PORTB_BASE, LUMB_AIR_F, 0);
                Gpio_Dio_Ip_WritePin(PORTA_BASE, RELEASE_LUMB, 1);
                OsIfDelay(4000);
                Gpio_Dio_Ip_WritePin(PORTA_BASE, RELEASE_LUMB, 0);
                CAN_send(TxData, len);
            }
            else if (sub == 51)
            {
                int len = snprintf(TxData, sizeof(TxData), "V%u:0", 3);
                Gpio_Dio_Ip_WritePin(PORTB_BASE, LUMB_AIR_F, 0);
                Gpio_Dio_Ip_WritePin(PORTA_BASE, RELEASE_LUMB_V2, 1);
                OsIfDelay(4000);
                Gpio_Dio_Ip_WritePin(PORTA_BASE, RELEASE_LUMB_V2, 0);
                CAN_send(TxData, len);
            }
        }
    }
    /* --- 3. ACTUATOR SECTION (Runs Every Loop Cycle) --- */
    // A. Suspension Control (Continuous adjustment)
    // This runs every loop, comparing current Lidar 'distance' to g_targetHeight
    if (g_targetHeight > 0)
    {
        if (g_targetHeight > (distance + 5))
        {
            Gpio_Dio_Ip_WritePin(PORTB_BASE, SUSP_AIR_F, 1);
            Gpio_Dio_Ip_WritePin(PORTA_BASE, RELEASE_SUSP, 0);
        }
        else if (g_targetHeight < (distance - 5))
        {
            Gpio_Dio_Ip_WritePin(PORTB_BASE, SUSP_AIR_F, 0);
            Gpio_Dio_Ip_WritePin(PORTA_BASE, RELEASE_SUSP, 1);
        }
        else
        {
            Gpio_Dio_Ip_WritePin(PORTB_BASE, SUSP_AIR_F, 0);
            Gpio_Dio_Ip_WritePin(PORTA_BASE, RELEASE_SUSP, 0);
        }
    }
}

/* Static variables to maintain state between loop cycles */
// static uint32_t g_lastAliveTick = 0;
// static uint32_t g_targetHeight = 0;
// static uint8_t  g_prevLumbarVal = 0;

/* Timer variables */
static uint32_t g_timerStart = 0;
static uint32_t g_timerDurationTicks = 0;
static boolean g_timerActive = FALSE;
static uint8_t g_activeValvePin = 0;
static uint32_t g_activePortBase = 0;

// Using osIf delay method non blocking
void ProcessRxData1(void)
{
    uint32_t currentTime = OsIf_GetCounter(OSIF_COUNTER_SYSTEM);
    char TxData[12];

    /* ================= 1. HEARTBEAT SECTION (Every 9 Seconds) ================= */
    uint32_t intervalTicks = OsIf_MicrosToTicks(9000000, OSIF_COUNTER_SYSTEM);
    if (g_lastAliveTick == 0 || (currentTime - g_lastAliveTick >= intervalTicks))
    {
        int len = snprintf(TxData, sizeof(TxData), "HW%u:2", 2);
        CAN_send(TxData, len);
        g_lastAliveTick = currentTime;
    }

    /* ================= 2. CAN RECEIVE SECTION (Async) ================= */
    if (bRxFlag == TRUE)
    {
        bRxFlag = FALSE;
        if (NULL_PTR == aRxDataBuffer)
            return;

        uint8_t cmd = aRxDataBuffer->data[0];
        uint8_t sub = aRxDataBuffer->data[1];
        uint8_t val = aRxDataBuffer->data[3];

        // C1: Suspension Target
        if (cmd == 'C' && sub == '1')
        {
            g_targetHeight = val;
            if (g_targetHeight > 180)
                g_targetHeight = 180;
            if (g_targetHeight < 60)
                g_targetHeight = 60;
        }

        // C2: Lumbar Delta Timer
        else if (cmd == 'C' && sub == '2')
        {
            uint8_t currentVal = (val > 5) ? 5 : val;
            if (currentVal != g_prevLumbarVal)
            {
                int8_t delta = (int8_t)currentVal - (int8_t)g_prevLumbarVal;
                uint8_t abs_delta = (delta < 0) ? -delta : delta;

                g_timerDurationTicks = OsIf_MicrosToTicks(abs_delta * 1000000U, OSIF_COUNTER_SYSTEM);
                g_timerStart = currentTime;
                g_timerActive = TRUE;

                if (delta > 0)
                {
                    g_activePortBase = PORTB_BASE;
                    g_activeValvePin = LUMB_AIR_F;
                }
                else
                {
                    g_activePortBase = PORTA_BASE;
                    g_activeValvePin = RELEASE_LUMB;
                }
                Gpio_Dio_Ip_WritePin(g_activePortBase, g_activeValvePin, 1);
                g_prevLumbarVal = currentVal;
            }
        }

        // V: Manual Release (4 seconds)
        else if (cmd == 'V' && val == 1)
        {
            g_timerDurationTicks = OsIf_MicrosToTicks(4000000U, OSIF_COUNTER_SYSTEM);
            g_timerStart = currentTime;
            g_timerActive = TRUE;

            int v_num = 0;
            if (sub == '1')
            {
                g_activePortBase = PORTA_BASE;
                g_activeValvePin = RELEASE_SUSP;
                v_num = 1;
            }
            else if (sub == '2')
            {
                g_activePortBase = PORTA_BASE;
                g_activeValvePin = RELEASE_LUMB;
                v_num = 2;
            }
            else if (sub == '3')
            {
                g_activePortBase = PORTA_BASE;
                g_activeValvePin = RELEASE_LUMB_V2;
                v_num = 3;
            }

            Gpio_Dio_Ip_WritePin(g_activePortBase, g_activeValvePin, 1);
            int len = snprintf(TxData, sizeof(TxData), "V%d:0", v_num);
            CAN_send(TxData, len);
        }
    }

    /* ================= 3. ACTUATOR MONITORING (Runs Every Loop) ================= */

    // A. Suspension (Distance Based)
    // Added 5mm hysteresis to prevent valve flickering (hunting)
    if (g_targetHeight >= 60)
    {
        if (distance < (g_targetHeight - 5))
        {
            Gpio_Dio_Ip_WritePin(PORTB_BASE, SUSP_AIR_F, 1);
            Gpio_Dio_Ip_WritePin(PORTA_BASE, RELEASE_SUSP, 0);
        }
        else if (distance > (g_targetHeight + 5))
        {
            Gpio_Dio_Ip_WritePin(PORTB_BASE, SUSP_AIR_F, 0);
            Gpio_Dio_Ip_WritePin(PORTA_BASE, RELEASE_SUSP, 1);
        }
        else
        {
            // Only turn off if the manual 4s release is NOT running
            if (!(g_timerActive && g_activeValvePin == RELEASE_SUSP))
            {
                Gpio_Dio_Ip_WritePin(PORTB_BASE, SUSP_AIR_F, 0);
                Gpio_Dio_Ip_WritePin(PORTA_BASE, RELEASE_SUSP, 0);
            }
        }
    }

    // B. Timed Actions (Lumbar or V-Valves)
    if (g_timerActive)
    {
        uint32_t dummyRef = g_timerStart;
        if (OsIf_GetElapsed(&dummyRef, OSIF_COUNTER_SYSTEM) >= g_timerDurationTicks)
        {
            // Time is up! Turn off the specific valve that was active
            Gpio_Dio_Ip_WritePin(g_activePortBase, g_activeValvePin, 0);
            g_timerActive = FALSE;
        }
    }
}

// Using lpit TImer for non blocking delay
void ProcessRxData2(void)
{
    char aliveMsg[8];
    uint32_t currentTime = OsIf_GetCounter(OSIF_COUNTER_SYSTEM);
    uint32_t intervalTicks = OsIf_MicrosToTicks(ALIVE_INTERVAL_TICKS, OSIF_COUNTER_SYSTEM);

    /* --- 1. HEARTBEAT SECTION --- */
    if (g_lastAliveTick == 0 || (currentTime - g_lastAliveTick >= intervalTicks))
    {
        int len = snprintf(aliveMsg, sizeof(aliveMsg), "HW%u:1", 2);
        CAN_send(aliveMsg, len);
        g_lastAliveTick = currentTime;
    }

    if (bRxFlag == TRUE)
    {
        bRxFlag = FALSE;
        Gpio_Dio_Ip_WritePin(IP_PTD, 15U, 0);
        if (NULL_PTR == aRxDataBuffer)
            return;

        uint8_t cmd = aRxDataBuffer->data[0];
        uint8_t sub = aRxDataBuffer->data[1];
        uint8_t val = aRxDataBuffer->data[3];
/* New added  */
        if (cmd == 67 && sub == 49)
	   {
		   if (val == 0 || val == 1)
		   {
			   g_targetHeight = 0; // Set to 0 to trigger "Stop All Operations"
		   }
		   else if (val >= 60 && val <= 185)
		   {
			   g_targetHeight = val; // Valid range: start operation
		   }
		   /* Values like 2-59 are ignored - no change to g_targetHeight */
	   }

        // Command C1:1 (Suspension Target)new added
 /*       if (cmd == 67 && sub == 49)
        {

            g_targetHeight = val;
            //0

           if (g_targetHeight > 180) g_targetHeight = 180;
           if (g_targetHeight < 60) g_targetHeight = 60;

        }//new added end*/


        // Command C2:2 (Lumbar Target)
        else if (cmd == 67 && sub == 50)
        {
            uint8_t currentVal = val;
            if (currentVal != g_prevLumbarVal)
            {
                int8_t delta = (int8_t)currentVal - (int8_t)g_prevLumbarVal;
                uint8_t abs_delta = (delta < 0) ? -delta : delta;

                // Stop any previous timer before starting a new one
                Lpit_Gpt_Ip_StopTimer(LPIT_INST_0, CH_0);

                if (delta > 0)
                {
                    g_TimedPort = PORTB_BASE;
                    g_TimedPin = LUMB_AIR_F;
                    Gpio_Dio_Ip_WritePin(PORTB_BASE, LUMB_AIR_F, 1);   // PTB 12
                    Gpio_Dio_Ip_WritePin(PORTA_BASE, RELEASE_LUMB, 0); // PTA0

                    // Start LPIT: delta * 1 second (48,000,000 ticks)
                    Lpit_Gpt_Ip_StartTimer(LPIT_INST_0, CH_0, (uint32_t)abs_delta * 8000000U);
                }
                else if (delta < 0)
                {
                    g_TimedPort = PORTA_BASE;
                    g_TimedPin = RELEASE_LUMB;
                    Gpio_Dio_Ip_WritePin(PORTB_BASE, LUMB_AIR_F, 0);
                    Gpio_Dio_Ip_WritePin(PORTA_BASE, RELEASE_LUMB, 1);

                    Lpit_Gpt_Ip_StartTimer(LPIT_INST_0, CH_0, (uint32_t)abs_delta * 8000000U);
                }
                g_prevLumbarVal = currentVal;
            }
        }

        // Command V:1 Manual Emergency Stop (ASCII 86 = V)
        else if (cmd == 86 && val == 1)
        {
            Lpit_Gpt_Ip_StopTimer(0U, 0U);
            char TxData[10];

            if (sub == 49) // V1: Release Suspension
            {
                g_TimedPort = PORTA_BASE;
                g_TimedPin = RELEASE_SUSP;
                Gpio_Dio_Ip_WritePin(PORTB_BASE, SUSP_AIR_F, 0);
                Gpio_Dio_Ip_WritePin(PORTA_BASE, RELEASE_SUSP, 1);
                Lpit_Gpt_Ip_StartTimer(LPIT_INST_0, CH_0, 4U * 8000000U); // 4 Seconds

                int lenV = snprintf(TxData, sizeof(TxData), "V%u:0", 1);
                CAN_send(TxData, lenV);
            }
            else if (sub == 50) // V2: Release Lumbar
            {
                g_TimedPort = PORTA_BASE;
                g_TimedPin = RELEASE_LUMB;
                Gpio_Dio_Ip_WritePin(PORTB_BASE, LUMB_AIR_F, 0);
                Gpio_Dio_Ip_WritePin(PORTA_BASE, RELEASE_LUMB, 1);
                Lpit_Gpt_Ip_StartTimer(LPIT_INST_0, CH_0, 4U * 8000000U);

                int lenV = snprintf(TxData, sizeof(TxData), "V%u:0", 2);
                CAN_send(TxData, lenV);
            }
            else if (sub == 51) // V3: Release Lumbar V2
            {
                g_TimedPort = PORTA_BASE;
                g_TimedPin = RELEASE_LUMB_V2;
                Gpio_Dio_Ip_WritePin(PORTB_BASE, LUMB_AIR_F, 0);
                Gpio_Dio_Ip_WritePin(PORTA_BASE, RELEASE_LUMB_V2, 1);
                Lpit_Gpt_Ip_StartTimer(LPIT_INST_0, CH_0, 4U * 8000000U);

                int lenV = snprintf(TxData, sizeof(TxData), "V%u:0", 3);
                CAN_send(TxData, lenV);
            }
        }
    }

    /* --- 3. ACTUATOR SECTION (Continuous adjustment) --- */
    // This now runs even while Lumbar is filling/releasing! new added
    if ((g_targetHeight == 0) || (g_targetHeight == 1))
    {
        Gpio_Dio_Ip_WritePin(PORTB_BASE, SUSP_AIR_F, 0);
        // Only turn off Release if the 4-second manual release (V1) is not active
        if (g_TimedPin != RELEASE_SUSP)
        {
            Gpio_Dio_Ip_WritePin(PORTA_BASE, RELEASE_SUSP, 0);
        }
    } // new added
    // if (g_targetHeight > 0)
    else
    {
        if (g_targetHeight > (distance + 5))
        {
            Gpio_Dio_Ip_WritePin(PORTB_BASE, SUSP_AIR_F, 1);   // PTB13
            Gpio_Dio_Ip_WritePin(PORTA_BASE, RELEASE_SUSP, 0); // PTA3
        }
        else if (g_targetHeight < (distance - 5))
        {
            Gpio_Dio_Ip_WritePin(PORTB_BASE, SUSP_AIR_F, 0);
            Gpio_Dio_Ip_WritePin(PORTA_BASE, RELEASE_SUSP, 1);
        }
        else
        {
            // Safety: Only turn OFF if the 4-second V1 release isn't using this pin
            if (!(g_TimedPin == RELEASE_SUSP))
            {
                Gpio_Dio_Ip_WritePin(PORTB_BASE, SUSP_AIR_F, 0);
                Gpio_Dio_Ip_WritePin(PORTA_BASE, RELEASE_SUSP, 0);
            }
        }
    }
}
