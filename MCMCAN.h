#ifndef MCMCAN_H_
#define MCMCAN_H_ 1

#include <stdio.h>
#include <string.h>
#include "Ifx_Types.h"
#include "IfxCan_Can.h"
#include "IfxCan.h"
#include "IfxCpu_Irq.h"
#include "IfxPort.h"

#define CAN_MESSAGE_ID              (uint32)0x777
#define PIN0                        0
#define PIN1                        1
#define INVALID_RX_DATA_VALUE       0xA5
#define INVALID_ID_VALUE            (uint32)0xFFFFFFFF
#define ISR_PRIORITY_CAN_TX         2
#define ISR_PRIORITY_CAN_RX         1
#define MAXIMUM_CAN_DATA_PAYLOAD    2

#define CAN0_TXD                    IfxCan_TXD00_P20_8_OUT
#define CAN0_RXD                    IfxCan_RXD00B_P20_7_IN

typedef struct
{
    IfxCan_Can_Config canConfig;
    IfxCan_Can canModule;
    IfxCan_Can_Node canSrcNode;
    IfxCan_Can_Node canDstNode;
    IfxCan_Can_NodeConfig canNodeConfig;
    IfxCan_Filter canFilter;
    IfxCan_Message txMsg;
    IfxCan_Message rxMsg;
    uint32 txData[MAXIMUM_CAN_DATA_PAYLOAD];
    uint32 rxData[MAXIMUM_CAN_DATA_PAYLOAD];
} McmcanType;

void initMcmcan(void);
void transmitCanPosition(uint32 pos20);
void initLeds(void);
void enableCanTransceiver(void);

#endif
