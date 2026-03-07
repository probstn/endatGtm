#include "MCMCAN.h"

McmcanType         g_mcmcan;
IfxPort_Pin_Config g_led1;
IfxPort_Pin_Config g_led2;

IFX_INTERRUPT(canIsrTxHandler, 2, ISR_PRIORITY_CAN_TX);

void canIsrTxHandler(void)
{
    IfxCan_Node_clearInterruptFlag(g_mcmcan.canSrcNode.node,
                                   IfxCan_Interrupt_transmissionCompleted);

    IfxPort_setPinLow(g_led1.port, g_led1.pinIndex);
}

/* Optional: enable onboard CAN transceiver if your board has STB pin */
void enableCanTransceiver(void)
{
    /* Example for boards where standby is on P20.6.
       Change this pin if your board uses another STB pin. */
    IfxPort_setPinModeOutput(&MODULE_P20, 6, IfxPort_OutputMode_pushPull,
                             IfxPort_OutputIdx_general);
    IfxPort_setPinLow(&MODULE_P20, 6);   /* LOW = normal mode on many boards */
}

void initMcmcan(void)
{
    const IfxCan_Can_Pins canPins =
    {
        &CAN0_TXD,                       IfxPort_OutputMode_pushPull,
        &CAN0_RXD,                       IfxPort_InputMode_pullUp,
        IfxPort_PadDriver_cmosAutomotiveSpeed1
    };

    IfxCan_Can_initModuleConfig(&g_mcmcan.canConfig, &MODULE_CAN0);
    IfxCan_Can_initModule(&g_mcmcan.canModule, &g_mcmcan.canConfig);

    IfxCan_Can_initNodeConfig(&g_mcmcan.canNodeConfig, &g_mcmcan.canModule);

    /* Real bus, not internal loopback */
    g_mcmcan.canNodeConfig.busLoopbackEnabled = FALSE;
    g_mcmcan.canNodeConfig.nodeId             = IfxCan_NodeId_0;
    g_mcmcan.canNodeConfig.frame.type         = IfxCan_FrameType_transmit;
    g_mcmcan.canNodeConfig.pins               = &canPins;

    /* Bitrate: match this to your CAN analyzer / other node */
    g_mcmcan.canNodeConfig.baudRate.baudrate = 1000000;

    g_mcmcan.canNodeConfig.interruptConfig.transmissionCompletedEnabled = TRUE;
    g_mcmcan.canNodeConfig.interruptConfig.traco.priority              = ISR_PRIORITY_CAN_TX;
    g_mcmcan.canNodeConfig.interruptConfig.traco.interruptLine         = IfxCan_InterruptLine_0;
    g_mcmcan.canNodeConfig.interruptConfig.traco.typeOfService         = IfxSrc_Tos_cpu2;

    IfxCan_Can_initNode(&g_mcmcan.canSrcNode, &g_mcmcan.canNodeConfig);
}

void transmitCanMessage(void)
{
    IfxCan_Can_initMessage(&g_mcmcan.txMsg);

    g_mcmcan.txData[0] = TX_DATA_LOW_WORD;
    g_mcmcan.txData[1] = TX_DATA_HIGH_WORD;

    g_mcmcan.txMsg.messageId = CAN_MESSAGE_ID;
    g_mcmcan.txMsg.dataLengthCode = IfxCan_DataLengthCode_8;

    while (IfxCan_Status_notSentBusy ==
           IfxCan_Can_sendMessage(&g_mcmcan.canSrcNode,
                                  &g_mcmcan.txMsg,
                                  &g_mcmcan.txData[0]))
    {
    }
}

void initLeds(void)
{
    g_led1.port      = &MODULE_P13;
    g_led1.pinIndex  = PIN0;
    g_led1.mode      = IfxPort_OutputIdx_general;
    g_led1.padDriver = IfxPort_PadDriver_cmosAutomotiveSpeed1;

    g_led2.port      = &MODULE_P13;
    g_led2.pinIndex  = PIN1;
    g_led2.mode      = IfxPort_OutputIdx_general;
    g_led2.padDriver = IfxPort_PadDriver_cmosAutomotiveSpeed1;

    IfxPort_setPinHigh(g_led1.port, g_led1.pinIndex);
    IfxPort_setPinHigh(g_led2.port, g_led2.pinIndex);

    IfxPort_setPinModeOutput(g_led1.port, g_led1.pinIndex,
                             IfxPort_OutputMode_pushPull, g_led1.mode);
    IfxPort_setPinModeOutput(g_led2.port, g_led2.pinIndex,
                             IfxPort_OutputMode_pushPull, g_led2.mode);

    IfxPort_setPinPadDriver(g_led1.port, g_led1.pinIndex, g_led1.padDriver);
    IfxPort_setPinPadDriver(g_led2.port, g_led2.pinIndex, g_led2.padDriver);
}
