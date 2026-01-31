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

// ---------------------- CONFIGURATION & DEFINES (Moved to Top) ----------------------
#define FREQUENCY   1e5
#define SYS_FREQ    100e6
#define PERIOD      ((uint32)(SYS_FREQ / FREQUENCY))

#define RX_SEG_BITS     13u
#define RX_SEG_CNTS     (RX_SEG_BITS - 1u)   // 12
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
static const IfxGtm_Atom_ToutMap *clock_pin = &IfxGtm_ATOM0_0_TOUT53_P21_2_OUT;
static const IfxGtm_Atom_ToutMap *tx_pin    = &IfxGtm_ATOM0_1_TOUT47_P22_0_OUT;
static const IfxGtm_Atom_ToutMap *dir_pin   = &IfxGtm_ATOM0_2_TOUT2_P02_2_OUT;
static const IfxGtm_Atom_ToutMap *cmu_pin   = &IfxGtm_ATOM0_3_TOUT3_P02_3_OUT;
static const IfxGtm_Tim_TinMap   *rx_pin    = &IfxGtm_TIM0_0_P02_0_IN;

#define DBG_DMA_PIN     &MODULE_P14,6
#define DBG_START_PIN     &MODULE_P02,5

// ---------------------- Global Vars ----------------------
static IfxDma_Dma dma;
static IfxDma_Dma_Channel dmaCh;

static volatile uint32 rxSeg[RX_SEGS];

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
static void initAtomClock(void);
static void initAtomTx(void);
static void initAtomDir(void);
static void initAtomDebug(void);
static void initTimRx(void);
static void initTimRxTrigger(void);
static void initPins(void);
static void initDmaRx(void);

// Requested API
void prepareModeTransmission(void);
void fireTransmission(void);

// =========================================================
// INTERRUPT SERVICE ROUTINES
// =========================================================
#define ISR_PRIORITY_TIM0_CH0_NEWVAL  11

IFX_INTERRUPT(timRxWordDoneISR, 0, ISR_PRIORITY_TIM0_CH0_NEWVAL);
void timRxWordDoneISR(void)
{
    IfxPort_togglePin(DBG_DMA_PIN);
}

// 1. DMA Interrupt: Fires after 2 segments are received (TCOUNT reaches 0)
IFX_INTERRUPT(dmaRxISR, 0, ISR_PRIORITY_DMA_RX);

void dmaRxISR(void)
{
    IfxPort_togglePin(DBG_DMA_PIN);
    /*
    //IfxPort_togglePin(DBG_DMA_PIN);
    printf("rxSeg0 raw = 0x%08lx, rxSeg1 raw = 0x%08lx\n",
           (unsigned long)rxSeg[0],
           (unsigned long)rxSeg[1]);


    uint32 gpr0 = MODULE_GTM.TIM[0].CH0.GPR0.U;
    printf("TIM GPR0 = 0x%08lx\n", (unsigned long)gpr0);

    // 1. Immediately stop the Shift Timer (TIM0_CH0) and Clock
    MODULE_GTM.TIM[0].CH0.CTRL.B.TIM_EN = 0;
    MODULE_GTM.CMU.CLK_EN.U &= ~(1u << 1);

    // 2. Debug toggle


    // 4. Prepare system for the NEXT run (Reset DMA, Reset Flags)
        prepareModeTransmission();
    */
}

// 2. Start Interrupt: Fires on start bit edge (TIM0_CH1)
IFX_INTERRUPT(timRxStartISR, 0, ISR_PRIORITY_RX_START);

void timRxStartISR(void)
{
    IfxPort_togglePin(DBG_START_PIN);

    IfxSrc_clearRequest(&SRC_GTM_TIM0_1);

    /* Now start bit clock + shifter */
    MODULE_GTM.CMU.CLK_EN.U |= (1u << 1);
    MODULE_GTM.TIM[0].CH0.CTRL.B.TIM_EN = 1;
}


// =========================================================
// API & Logic
// =========================================================

/**
 * Stop timers, reset state, and prepare for the next transmission.r
 * Called inside ISR and can also be called from main/task context.
 */
void prepareModeTransmission(void)
{
    /* 0) stop shifter + stop bit clock so nothing can create requests */
    MODULE_GTM.TIM[0].CH0.CTRL.B.TIM_EN = 0;
    MODULE_GTM.CMU.CLK_EN.U &= ~(1u << 1);

    /* 2) clear pending service request on the TIM0_0 SRC line */
    IfxSrc_clearRequest(&SRC_GTM_TIM0_0);
    IfxSrc_clearRequest(&SRC_GTM_TIM0_1);
    MODULE_GTM.TIM[0].CH1.IRQ.NOTIFY.B.NEWVAL = 1;   // clear CH1 NEWVAL too

    /* your existing output disables are fine */
    agc->OUTEN_STAT.U = 0x55;
    MODULE_GTM.CMU.CLK_EN.U = 0x5;
}



/**
 * Start a single transmission frame.
 * Enables clocks + enables ATOM channels + sets initial counters, then HOST_TRIG.
 */
void fireTransmission(void)
{

    atom->CH0.CN0.U = PERIOD;
    atom->CH1.CN0.U = 0;
    atom->CH2.CN0.U = PERIOD*8;
    atom->CH3.CN0.U = 2*PERIOD;

    // ideally: force update for all channels here

    agc->OUTEN_CTRL.U = 0xAA;          // outputs on (or do this via OUTEN_CTRL with UPEN)
    agc->GLB_CTRL.B.HOST_TRIG = 1;     // now start is synchronous and deterministic
    MODULE_GTM.CMU.CLK_EN.U = 0xA;     // enable clocks FIRST
}

// =========================================================
// Initialization
// =========================================================
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
    initAtomClock();
    initAtomDebug();

    initTimRx();
    initDmaRx();
    initTimRxTrigger();

    initPins();
    IfxPort_setPinModeOutput(DBG_DMA_PIN, IfxPort_OutputMode_pushPull, IfxPort_OutputIdx_general);
    IfxPort_setPinModeOutput(DBG_START_PIN, IfxPort_OutputMode_pushPull, IfxPort_OutputIdx_general);

    agc->GLB_CTRL.B.HOST_TRIG = 1; //update all shadows

    // Pre-arm the DMA for the very first run
    prepareModeTransmission();

    fireTransmission();
}

static void initDmaRx(void)
{
    IfxDma_Dma_Config dmaCfg;
    IfxDma_Dma_initModuleConfig(&dmaCfg, &MODULE_DMA);
    IfxDma_Dma_initModule(&dma, &dmaCfg);

    IfxDma_Dma_ChannelConfig chCfg;
    IfxDma_Dma_initChannelConfig(&chCfg, &dma);

    chCfg.channelId = DMA_CH_RX;

    chCfg.sourceAddress = (uint32)&MODULE_GTM.TIM[0].CH0.GPR0.U;
    chCfg.sourceAddressIncrementStep      = IfxDma_ChannelIncrementStep_1;
    chCfg.sourceAddressIncrementDirection = IfxDma_ChannelIncrementDirection_positive;
    chCfg.sourceCircularBufferEnabled     = TRUE;
    chCfg.sourceAddressCircularRange      = IfxDma_ChannelIncrementCircular_4;  // keep source fixed (32-bit)


    // DESTINATION: Buffer in RAM
    chCfg.destinationAddress = IFXCPU_GLB_ADDR_DSPR(IfxCpu_getCoreId(), &rxSeg[0]);
    chCfg.destinationAddressIncrementStep      = IfxDma_ChannelIncrementStep_1;
    chCfg.destinationAddressIncrementDirection = IfxDma_ChannelIncrementDirection_positive;
    chCfg.destinationCircularBufferEnabled     = FALSE;

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

// =========================================================
// GTM base + CMU
// =========================================================
static void initGtmBase(void)
{
    IfxGtm_enable(gtm);

    float32 mod = IfxGtm_Cmu_getModuleFrequency(gtm);
    IfxGtm_Cmu_setGclkFrequency(gtm, mod);
}

static void initGtmCmu(void)
{
    float32 mod = IfxGtm_Cmu_getModuleFrequency(gtm);

    // CMU_CLK0 = fast (for debug, etc.)
    IfxGtm_Cmu_setClkFrequency(gtm, IfxGtm_Cmu_Clk_0, mod);

    // CMU_CLK1 = bit clock (100 kHz)
    IfxGtm_Cmu_setClkFrequency(gtm, IfxGtm_Cmu_Clk_1, FREQUENCY);

    // Do not enable clocks here; fireTransmission() will enable them
}

// =========================================================
// ATOM channels
// =========================================================
static void initAtomClock(void)
{
    Ifx_GTM_ATOM_CH *ch = atomCh(clock_pin->channel);

    ch->CTRL.B.MODE       = 2;   // SOMP
    ch->CTRL.B.SL         = 0;
    ch->CTRL.B.CLK_SRC_SR = 0;
    ch->CTRL.B.UDMODE     = 0;
    ch->CTRL.B.OSM        = 0;

    ch->SR1.U = PERIOD / 2;
    ch->SR0.U = PERIOD;

    ch->CTRL.B.TRIGOUT = 1;

    agc->OUTEN_CTRL.B.OUTEN_CTRL0 = 2;  // enable output
    agc->GLB_CTRL.B.UPEN_CTRL0    = 2;  // enable update
    agc->ENDIS_CTRL.B.ENDIS_CTRL0 = 2;  // enable operation
    //agc->FUPD_CTRL.B.RSTCN0_CH0 = 2;
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
    //ch->CTRL.B.TRIGOUT    = 0;  // TRIG_[1] forwards TRIG_[0]
    //ch->CTRL.B.EXTTRIGOUT = 0;  // choose TRIG_[x-1] as forwarded signal

    ch->SR1.U = 0x7u << (24 - 8); // example pattern
    ch->SR0.U = 7u;

    agc->OUTEN_CTRL.B.OUTEN_CTRL1 = 2;
    agc->GLB_CTRL.B.UPEN_CTRL1    = 2;
    agc->ENDIS_CTRL.B.ENDIS_CTRL1 = 2;
    agc->FUPD_CTRL.B.RSTCN0_CH1 = 2;
}

static void initAtomDir(void)
{
    Ifx_GTM_ATOM_CH *ch = atomCh(dir_pin->channel);

    ch->CTRL.B.MODE       = 2;   // SOMP
    ch->CTRL.B.SL         = 1;
    ch->CTRL.B.CLK_SRC_SR = 0;
    ch->CTRL.B.UDMODE     = 0;
    ch->CTRL.B.OSM        = 1;

    ch->SR1.U = PERIOD * 7;
    ch->SR0.U = PERIOD * 9;

    ch->CTRL.B.TRIGOUT = 1;

    agc->OUTEN_CTRL.B.OUTEN_CTRL2 = 2;  // enable output
    agc->GLB_CTRL.B.UPEN_CTRL2    = 2;  // enable update
    agc->ENDIS_CTRL.B.ENDIS_CTRL2 = 2;  // enable operation
    //agc->FUPD_CTRL.B.RSTCN0_CH2 = 2;
}

static void initAtomDebug(void)
{
    Ifx_GTM_ATOM_CH *ch = atomCh(cmu_pin->channel);

    ch->CTRL.B.MODE       = 2;
    ch->CTRL.B.SL         = 0;
    ch->CTRL.B.CLK_SRC_SR = 1;

    ch->SR1.U = 1;
    ch->SR0.U = 2;

    agc->OUTEN_CTRL.B.OUTEN_CTRL3 = 2;
    agc->GLB_CTRL.B.UPEN_CTRL3    = 2;
    agc->ENDIS_CTRL.B.ENDIS_CTRL3 = 2;
    agc->FUPD_CTRL.B.RSTCN0_CH3 = 2;
}

static void initTimRx(void) {
    Ifx_GTM_TIM_CH *ch = timCh(0);
    IfxGtm_PinMap_setTimTin(rx_pin, IfxPort_InputMode_noPullDevice);

    ch->CTRL.B.TIM_EN = 0;
    ch->CTRL.B.TIM_MODE = 0x6; // TSM Mode

    // Crucial: Load GPR1 with the number of bits to shift
    // This reloads CNTS automatically for the second segment
    ch->GPR1.B.GPR1 = RX_SEG_CNTS;
    ch->CNTS.B.CNTS = RX_SEG_CNTS;

    ch->CTRL.B.ISL = 0;
    ch->CTRL.B.DSL = 1;
    ch->CTRL.B.CLK_SEL = 1; // CMU_CLK1

    ch->IRQ.EN.B.NEWVAL_IRQ_EN = 1;
    ch->IRQ.MODE.B.IRQ_MODE = 0x2; // Pulse mode is often better for DMA triggers

    //IfxSrc_init(&SRC_GTM_TIM0_0, IfxSrc_Tos_cpu0, ISR_PRIORITY_TIM0_CH0_NEWVAL);
    IfxSrc_init(&SRC_GTM_TIM0_0, IfxSrc_Tos_dma, DMA_CH_RX);
    IfxSrc_enable(&SRC_GTM_TIM0_0);
}

static void initTimRxTrigger(void) {
    Ifx_GTM_TIM_CH *ch = timCh(1);
    ch->CTRL.B.TIM_EN = 0; ch->CTRL.B.TIM_MODE = 0x2; ch->CTRL.B.OSM = 1;
    ch->CTRL.B.CICTRL = 1; ch->CTRL.B.DSL = 1;
    ch->IRQ.EN.B.NEWVAL_IRQ_EN = 1; ch->IRQ.NOTIFY.B.NEWVAL = 1;
    IfxSrc_init(&SRC_GTM_TIM0_1, IfxSrc_Tos_cpu0, ISR_PRIORITY_RX_START);
    IfxSrc_enable(&SRC_GTM_TIM0_1);
    ch->CTRL.B.TIM_EN = 1;
}

static void initPins(void) {
    IfxGtm_PinMap_setAtomTout(clock_pin, IfxPort_OutputMode_pushPull, IfxPort_PadDriver_cmosAutomotiveSpeed1);
    IfxGtm_PinMap_setAtomTout(cmu_pin, IfxPort_OutputMode_pushPull, IfxPort_PadDriver_cmosAutomotiveSpeed1);
    IfxGtm_PinMap_setAtomTout(dir_pin,   IfxPort_OutputMode_pushPull, IfxPort_PadDriver_cmosAutomotiveSpeed1);
    IfxGtm_PinMap_setAtomTout(tx_pin,    IfxPort_OutputMode_pushPull, IfxPort_PadDriver_cmosAutomotiveSpeed1);
}
