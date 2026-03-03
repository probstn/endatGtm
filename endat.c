#include "IfxGtm.h"
#include "IfxGtm_Atom.h"
#include "IfxGtm_Tim.h"
#include "IfxPort.h"
#include "IfxGtm_PinMap.h"
#include "IfxSrc.h"
#include "IfxCpu_Irq.h"
#include <stdio.h>
#include "IfxStm.h"
#include "IfxDma_Dma.h"
#include "IfxDma.h"
#include <stdbool.h>
#include <inttypes.h>


// ---------------------- CONFIGURATION & DEFINES (Moved to Top) ----------------------
#define FREQUENCY   1e5
#define SYS_FREQ    100e6
#define PERIOD      ((uint32)(SYS_FREQ / FREQUENCY))

#define RX_WORD_LENGTH_BITS     13u
#define RX_SEGS         2u

// Interrupt Priorities
#define ISR_PRIORITY_RX_START  9
#define ISR_PRIORITY_DMA_RX    10

// DMA Settings
#define DMA_CH_RX   0

// ---------------------- HW handles ----------------------
static Ifx_GTM *gtm = &MODULE_GTM;
static Ifx_GTM_ATOM     *atom = NULL_PTR;
static Ifx_GTM_ATOM_AGC *agc  = NULL_PTR;
static Ifx_GTM_TIM      *tim  = NULL_PTR;

// ---------------------- Pin maps ------------------------
static const IfxGtm_Atom_ToutMap *ss_pin = &IfxGtm_ATOM0_5_TOUT5_P02_5_OUT;
static const IfxGtm_Atom_ToutMap *tx_clk_pin    = &IfxGtm_ATOM0_1_TOUT1_P02_1_OUT;
static const IfxGtm_Atom_ToutMap *tx_pin   = &IfxGtm_ATOM0_2_TOUT2_P02_2_OUT;
static const IfxGtm_Atom_ToutMap *dir_pin   = &IfxGtm_ATOM0_3_TOUT3_P02_3_OUT;
static const IfxGtm_Atom_ToutMap *rx_clk_pin   = &IfxGtm_ATOM0_4_TOUT4_P02_4_OUT;

static const IfxGtm_Tim_TinMap   *rx_pin    = &IfxGtm_TIM0_5_P23_0_IN;

#define DBG_DMA_PIN     &MODULE_P14,6
#define DBG_START_PIN     &MODULE_P02,7

// ---------------------- Global Var -----------------------
static IfxDma_Dma dma;
static IfxDma_Dma_Channel dmaCh;

// ---------------------- Helpers --------------------------
static inline Ifx_GTM_ATOM_CH *atomCh(uint8 ch) {
    return IfxGtm_Atom_Ch_getChannelPointer(atom, ch);
}

static inline Ifx_GTM_TIM_CH *timCh(uint8 ch) {
    return IfxGtm_Tim_getChannel(tim, ch);
}

// ---------------------- Forward decl ---------------------
static void initGtmBase(void);
static void initGtmCmu(void);
static void initAtomTxClock(void);
static void initAtomTx(void);
static void initAtomDir(void);
static void initTimStartBitDetect(void);
static void initPins(void);
static void initDmaRx(void);
void fireTransmission(void);
static void initAtomRxClock(void);
static void initAtomSlaveSelect(void);

#define ISR_PRIORITY_ATOM0_CH1_CCU0TC  10
IFX_INTERRUPT(atomDirShiftCompleteIsr, 0, ISR_PRIORITY_ATOM0_CH1_CCU0TC);

void atomDirShiftCompleteIsr(void)
{
//    initTimStartBitDetect();
    IfxPort_togglePin(DBG_START_PIN);
    initTimStartBitDetect();

}

/**
 * Start a single transmission frame.
 * Enables clocks + enables ATOM channels + sets initial counters, then HOST_TRIG.
 */
void fireTransmission(void)
{
    atom->CH1.CN0.U = 0;
    atom->CH2.CN0.U = 0;
    atom->CH3.CN0.U = 0;
    agc->GLB_CTRL.B.HOST_TRIG = 1;     // now start is synchronous and deterministic
}

void init(void)
{
    // Use literal 0 instead of enum to avoid symbol errors
    atom = &gtm->ATOM[0];
    agc  = &atom->AGC;
    tim  = &gtm->TIM[0];

    initGtmBase();
    initGtmCmu();
    initAtomTx();
    initAtomDir();
    initAtomTxClock();
    initAtomRxClock();

    initAtomSlaveSelect();
    initDmaRx();

    initPins();
    IfxPort_setPinModeOutput(DBG_DMA_PIN, IfxPort_OutputMode_pushPull, IfxPort_OutputIdx_general);
    IfxPort_setPinModeOutput(DBG_START_PIN, IfxPort_OutputMode_pushPull, IfxPort_OutputIdx_general);

    fireTransmission();

    while(true)
    {
    }
}

static void initDmaRx(void)
{
    IfxDma_Dma_Config dmaCfg;
    IfxDma_Dma_initModuleConfig(&dmaCfg, &MODULE_DMA);
    IfxDma_Dma_initModule(&dma, &dmaCfg);

    IfxDma_Dma_ChannelConfig chCfg;
    IfxDma_Dma_initChannelConfig(&chCfg, &dma);

    chCfg.channelId = DMA_CH_RX;

    chCfg.sourceAddress = (uint32)&MODULE_GTM.TIM[0].CH0.GPR1.U;
    //chCfg.sourceAddressIncrementStep      = IfxDma_ChannelIncrementStep_1;
    //chCfg.sourceAddressIncrementDirection = IfxDma_ChannelIncrementDirection_positive;
    chCfg.sourceCircularBufferEnabled     = TRUE;
    chCfg.sourceAddressCircularRange      = IfxDma_ChannelIncrementCircular_none;  // keep source fixed (32-bit)


    // DESTINATION: Buffer in RAM
    //chCfg.destinationAddress = IFXCPU_GLB_ADDR_DSPR(IfxCpu_getCoreId(), &rx_buffer[0]);
    chCfg.destinationAddressIncrementStep      = IfxDma_ChannelIncrementStep_1;
    chCfg.destinationAddressIncrementDirection = IfxDma_ChannelIncrementDirection_positive;

    // TRANSFER CONFIG
    chCfg.blockMode     = IfxDma_ChannelMove_1;
    chCfg.transferCount = RX_SEGS;
    chCfg.moveSize      = IfxDma_ChannelMoveSize_32bit;

    // TRIGGER CONFIG
    chCfg.hardwareRequestEnabled = TRUE;
    chCfg.requestSource = IfxDma_ChannelRequestSource_peripheral;
    chCfg.requestMode = IfxDma_ChannelRequestMode_oneTransferPerRequest;

    // INTERRUPT CONFIG
    chCfg.channelInterruptEnabled = TRUE;
    chCfg.channelInterruptPriority = ISR_PRIORITY_DMA_RX;
    chCfg.channelInterruptTypeOfService = IfxSrc_Tos_cpu0;
    chCfg.channelInterruptControl = IfxDma_ChannelInterruptControl_thresholdLimitMatch;
    chCfg.interruptRaiseThreshold = 0;

    IfxDma_Dma_initChannel(&dmaCh, &chCfg);
}

static void initGtmBase(void)
{
    IfxGtm_enable(gtm);

    float32 mod = IfxGtm_Cmu_getModuleFrequency(gtm);
    IfxGtm_Cmu_setGclkFrequency(gtm, mod);
}

static void initGtmCmu(void)
{
    // CMU_CLK0 = fast (for debug, etc.)
    IfxGtm_Cmu_setClkFrequency(gtm, IfxGtm_Cmu_Clk_0, SYS_FREQ);
    MODULE_GTM.CMU.CLK_EN.U = 0x2; //0b10 enables CMU0
}

static void initAtomTxClock(void)
{
    Ifx_GTM_ATOM_CH *ch = atomCh(tx_clk_pin->channel);

    ch->CTRL.B.MODE       = 2;   // SOMP
    ch->CTRL.B.SL         = 0;
    ch->CTRL.B.CLK_SRC_SR = 0; //CMU0

    ch->SR1.U = PERIOD / 2;
    ch->SR0.U = PERIOD;

    ch->CTRL.B.TRIGOUT = 1;

    agc->OUTEN_CTRL.B.OUTEN_CTRL1 = 2;  // enable output
    agc->GLB_CTRL.B.UPEN_CTRL1    = 2;  // enable update
    agc->ENDIS_CTRL.B.ENDIS_CTRL1 = 2;  // enable operation
    //agc->FUPD_CTRL.B.RSTCN0_CH1 = 2;
}

static void initAtomRxClock(void)
{
    Ifx_GTM_ATOM_CH *ch = atomCh(rx_clk_pin->channel);

    ch->CTRL.B.MODE       = 2;   // SOMP
    ch->CTRL.B.SL         = 0;
    ch->CTRL.B.CLK_SRC_SR = 0; //CMU0

    ch->SR1.U = PERIOD/2;
    ch->SR0.U = PERIOD;
    ch->CN0.U = PERIOD/2-1;

    ch->CTRL.B.TRIGOUT = 1;

    agc->OUTEN_CTRL.B.OUTEN_CTRL4 = 2;  // enable output
    agc->GLB_CTRL.B.UPEN_CTRL4    = 2;  // enable update
    agc->ENDIS_CTRL.B.ENDIS_CTRL4 = 2;  // enable operation
    //agc->FUPD_CTRL.B.RSTCN0_CH4 = 2;
}

static void initAtomTx(void)
{
    Ifx_GTM_ATOM_CH *ch = atomCh(tx_pin->channel);

    ch->CTRL.B.MODE   = 3;
    ch->CTRL.B.ARU_EN = 0;
    ch->CTRL.B.ACB    = 1;
    ch->CTRL.B.OSM    = 1;
    ch->CTRL.B.SL     = 1;

    // Triggered clock from CH0 trigger
    ch->CTRL.B.ECLK_SRC   = 1;
    ch->CTRL.B.CLK_SRC_SR = 5;

    //forward trigger
    ch->CTRL.B.TRIGOUT    = 0;  // TRIG_[1] forwards TRIG_[0]
    ch->CTRL.B.EXTTRIGOUT = 0;  // choose TRIG_[x-1] as forwarded signal

    ch->SR1.U = 0x7u << (24 - 8); // example pattern
    ch->SR0.U = 8u;

    agc->OUTEN_CTRL.B.OUTEN_CTRL2 = 2;
    agc->GLB_CTRL.B.UPEN_CTRL2    = 2;
    agc->ENDIS_CTRL.B.ENDIS_CTRL2 = 2;
    agc->FUPD_CTRL.B.RSTCN0_CH2 = 2;
}

static void initAtomDir(void)
{
    Ifx_GTM_ATOM_CH *ch = atomCh(dir_pin->channel);

    ch->CTRL.B.MODE   = 3;
    ch->CTRL.B.ARU_EN = 0;
    ch->CTRL.B.ACB    = 1;
    ch->CTRL.B.OSM    = 1;
    ch->CTRL.B.SL     = 1;

    // Triggered clock from CH0 trigger
    ch->CTRL.B.ECLK_SRC   = 1;
    ch->CTRL.B.CLK_SRC_SR = 5;

    //forward trigger
    //ch->CTRL.B.TRIGOUT    = 0;  // TRIG_[1] forwards TRIG_[0]
    //ch->CTRL.B.EXTTRIGOUT = 0;  // choose TRIG_[x-1] as forwarded signal

    ch->SR1.U = 0x7Fu << (24 - 8); // example pattern
    ch->SR0.U = 8u;


    // 1. Enable the CCU0TC interrupt (triggers when CN0 reaches CM0)
    ch->IRQ.EN.B.CCU0TC_IRQ_EN = 1;
    ch->IRQ.EN.B.CCU1TC_IRQ_EN = 1;
    ch->IRQ.MODE.B.IRQ_MODE = 0x2; // Pulse-Notify mode (standard for AURIX GTM IRQs)

    IfxSrc_init(&SRC_GTM_ATOM0_1, IfxSrc_Tos_cpu0, ISR_PRIORITY_ATOM0_CH1_CCU0TC);
    IfxSrc_enable(&SRC_GTM_ATOM0_1);
    // 2. Route the interrupt to CPU0 (Assuming ATOM0 and Channel 3)
    // Make sure SRC_GTM_ATOM0_3 matches your actual ATOM instance and channel!

    agc->OUTEN_CTRL.B.OUTEN_CTRL3 = 2;
    agc->GLB_CTRL.B.UPEN_CTRL3    = 2;
    agc->ENDIS_CTRL.B.ENDIS_CTRL3 = 2;
    agc->FUPD_CTRL.B.RSTCN0_CH3 = 2;
}

static void initTimStartBitDetect(void)
{
    Ifx_GTM_TIM_CH *ch = timCh(rx_pin->channel);

    ch->CTRL.B.TIM_EN = 0;

    ch->CTRL.B.TIM_MODE = 0x2;  // TIEM
    ch->CTRL.B.DSL      = 1;    // rising edge active
    ch->CTRL.B.ISL      = 0;    // use DSL
    ch->CTRL.B.CICTRL   = 1;    // use TIM_IN[x]
    ch->ECTRL.B.EXT_CAP_SRC = 0x3; //use TIM_IN[x] as trigger

    ch->CTRL.B.TIM_EN   = 1;

    /*
    // Enable NEWVAL interrupt for CH1
    ch->IRQ.EN.B.NEWVAL_IRQ_EN = 1;
    // Pulse-Notify mode is convenient (00 level, 01 pulse, 10 pulse-notify, 11 single-pulse)
    ch->IRQ.MODE.B.IRQ_MODE = 0x2;
    // Route to CPU0 with priority
    IfxSrc_init(&SRC_GTM_TIM0_4, IfxSrc_Tos_cpu0, ISR_PRIORITY_TIM0_CH4_NEWVAL);
    IfxSrc_enable(&SRC_GTM_TIM0_4);
    */
}

static void initAtomSlaveSelect(void)
{
    Ifx_GTM_ATOM_CH *ch = atomCh(ss_pin->channel);

    // Basic Configuration
    ch->CTRL.B.MODE       = 2;   // SOMP
    ch->CTRL.B.CLK_SRC_SR = 0;   // CMU_CLK0
    ch->CTRL.B.OSM        = 1;   // one-shot
    ch->CTRL.B.SL         = 1;   // pulse level HIGH (idle LOW)
    ch->CTRL.B.RST_CCU0   = 0;   // must be 0 when using OSM_TRIG
    ch->CTRL.B.OSM_TRIG   = 1;   // START one-shot on trigger
    ch->CTRL.B.EXT_TRIG   = 1;   // signal TIM_EXT_CAPTURE[x] is selected

    // --- The Immediate 26-Cycle Pulse Setup ---
    uint32 pulseCycles = 26*PERIOD;
    uint32 totalCycles = pulseCycles + 2; // CM0 must be > CM1

    // 1. Set Active and Shadow Compare Registers
    ch->CM1.U = pulseCycles;     // Pulse ends at 26
    ch->SR1.U = pulseCycles;

    ch->CM0.U = totalCycles;     // Period is 28
    ch->SR0.U = totalCycles;

    // 2. Pre-load the counter to 1 tick before rollover
    ch->CN0.U = totalCycles +1; // Starts at 27. Rolls to 0 on first tick.

    // Enable Outputs
    agc->OUTEN_CTRL.B.OUTEN_CTRL5 = 2;
    agc->ENDIS_CTRL.B.ENDIS_CTRL5 = 2;

    // Force shadow update just to be perfectly safe
    agc->FUPD_CTRL.B.FUPD_CTRL5 = 2;
}

static void initPins(void)
{
    IfxGtm_PinMap_setAtomTout(ss_pin, IfxPort_OutputMode_pushPull, IfxPort_PadDriver_cmosAutomotiveSpeed1);
    IfxGtm_PinMap_setAtomTout(tx_clk_pin,   IfxPort_OutputMode_pushPull, IfxPort_PadDriver_cmosAutomotiveSpeed1);
    IfxGtm_PinMap_setAtomTout(tx_pin,   IfxPort_OutputMode_pushPull, IfxPort_PadDriver_cmosAutomotiveSpeed1);
    IfxGtm_PinMap_setAtomTout(dir_pin,    IfxPort_OutputMode_pushPull, IfxPort_PadDriver_cmosAutomotiveSpeed1);
    IfxGtm_PinMap_setAtomTout(rx_clk_pin,    IfxPort_OutputMode_pushPull, IfxPort_PadDriver_cmosAutomotiveSpeed1);

    IfxGtm_PinMap_setTimTin(rx_pin, IfxPort_InputMode_noPullDevice);
}
