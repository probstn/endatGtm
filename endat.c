#include "IfxGtm.h"
#include "IfxGtm_Atom.h"
#include "IfxGtm_Tim.h"
#include "IfxPort.h"
#include "IfxGtm_PinMap.h"
#include "IfxSrc.h"
#include "IfxCpu_Irq.h"
#include <stdio.h>
#include "IfxStm.h"
#include "IfxPort.h"
#include "IfxPort_PinMap.h"

// ---------------------- HW handles ----------------------
static Ifx_GTM *gtm = &MODULE_GTM;
static Ifx_GTM_ATOM     *atom = NULL_PTR;
static Ifx_GTM_ATOM_AGC *agc  = NULL_PTR;
static Ifx_GTM_TIM      *tim  = NULL_PTR;

// ---------------------- Pin maps ------------------------
static const IfxGtm_Atom_ToutMap *clock_pin = &IfxGtm_ATOM0_0_TOUT53_P21_2_OUT;
static const IfxGtm_Atom_ToutMap *tx_pin    = &IfxGtm_ATOM0_1_TOUT47_P22_0_OUT;
static const IfxGtm_Atom_ToutMap *dir_pin   = &IfxGtm_ATOM0_2_TOUT2_P02_2_OUT;
static const IfxGtm_Atom_ToutMap *debug_pin = &IfxGtm_ATOM0_3_TOUT3_P02_3_OUT;
static const IfxGtm_Tim_TinMap   *rx_pin    = &IfxGtm_TIM0_0_P02_0_IN;

#define DBG_PIN     &MODULE_P14,6

// ---------------------- Timing --------------------------
#define FREQUENCY   1e5
#define SYS_FREQ    100e6
#define PERIOD      ((uint32)(SYS_FREQ / FREQUENCY))

#define RX_BITS     24u
#define RX_CNTS     (RX_BITS - 1u)

#define ISR_PRIORITY_RX  10

// ---------------------- Helpers --------------------------
static inline Ifx_GTM_ATOM_CH *atomCh(uint8 ch)
{
    return IfxGtm_Atom_Ch_getChannelPointer(atom, ch);
}

static inline Ifx_GTM_TIM_CH *timCh(uint8 ch)
{
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

// Requested API
void prepareModeTransmission(void);
void fireTransmission(void);

// ---------------------- ISR ------------------------------
IFX_INTERRUPT(timRxISR, 0, ISR_PRIORITY_RX);

static volatile uint8 wait_done = 0;

#define ISR_PRIORITY_RX_START  9
IFX_INTERRUPT(timRxStartISR, 0, ISR_PRIORITY_RX_START);

void timRxStartISR(void)
{
    IfxPort_togglePin(DBG_PIN);

    // Clear CH1 NEWVAL
    MODULE_GTM.TIM[0].CH1.IRQ.NOTIFY.B.NEWVAL = 1;

    // Reset CH0 shift progress
    MODULE_GTM.TIM[0].CH0.CNTS.B.CNTS = (RX_BITS - 1);
    MODULE_GTM.TIM[0].CH0.CNT.U       = 0;            // clear shift register counter field

    // Start sampling clock and shifting
    MODULE_GTM.CMU.CLK_EN.U |= (1u << 1);             // enable CMU_CLK1
    MODULE_GTM.TIM[0].CH0.CTRL.B.TIM_EN = 1;
}

void timRxISR(void)
{
    IfxPort_togglePin(DBG_PIN);
    MODULE_GTM.TIM[0].CH0.IRQ.NOTIFY.B.NEWVAL = 1;

    // Stop sampling
    MODULE_GTM.TIM[0].CH0.CTRL.B.TIM_EN = 0;
    MODULE_GTM.CMU.CLK_EN.U &= ~(1u << 1);
    prepareModeTransmission();

    uint32 rawData = MODULE_GTM.TIM[0].CH0.CNT.U & 0x00FFFFFFu;

    // Print raw bits

/*
    for (int i = 0; i < 24; i++)
        printf("%u", (rawData >> i) & 1u);
    printf("\n");

*/
    uint8  startBit = (rawData >> 0) & 1u;
    uint8  faultBit = (rawData >> 1) & 1u;
    uint32 position = (rawData >> 2)  & 0x7FFFFu;

    if (startBit != 1)
    {
        printf("Invalid frame\n");
    }
    else
    {
        if (!faultBit)
            printf("%lu\n", position);

            //printf("Fault\n");
    }
    // Re-arm start detector
    MODULE_GTM.TIM[0].CH1.CTRL.B.TIM_EN = 0;
    MODULE_GTM.TIM[0].CH1.IRQ.NOTIFY.B.NEWVAL = 1;
    MODULE_GTM.TIM[0].CH1.CTRL.B.TIM_EN = 1;

}

// =========================================================
// Init
// =========================================================
void init(void)
{
    atom = &gtm->ATOM[IfxGtm_Atom_0];
    agc  = &atom->AGC;
    tim  = &gtm->TIM[IfxGtm_Tim_0];

    initGtmBase();
    initGtmCmu();

    // Configure blocks (but do not start “transmission” yet)
    initAtomTx();
    initAtomDir();
    initAtomClock();
    initAtomDebug();

    initTimRx();
    initTimRxTrigger();

    initPins();

    // Put system into a known idle/prepared state
    IfxPort_setPinModeOutput(DBG_PIN, IfxPort_OutputMode_pushPull, IfxPort_OutputIdx_general);
    while(1)
    {

        fireTransmission();
        IfxStm_wait(IfxStm_getTicksFromMilliseconds(&MODULE_STM0, 100));
    }
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
    ch->SR0.U = 8u;

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
    Ifx_GTM_ATOM_CH *ch = atomCh(debug_pin->channel);

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

// =========================================================
// TIM channels
// =========================================================
static void initTimRx(void)
{
    Ifx_GTM_TIM_CH *ch = timCh(0);

    IfxGtm_PinMap_setTimTin(rx_pin, IfxPort_InputMode_noPullDevice);

    ch->CTRL.B.TIM_EN   = 0;
    ch->CTRL.B.TIM_MODE = 0x6;     // TSSM
    ch->CTRL.B.ISL      = 0;       // sample from FOUTx (RX input)
    ch->CTRL.B.DSL      = 1;       // shift direction (pick what matches your bit order)

    // IMPORTANT: do NOT use external capture for “start” here
    ch->CTRL.B.EXT_CAP_EN = 0;

    // Select shift clock source = CMU_CLK1.
    // On many TC3xx setups, CLK_SEL is used when CNTS[17:16]=00 (default path).
    ch->CTRL.B.CLK_SEL = 1;        // CMU_CLK1

    // Program “24 bits”
    ch->CNTS.U = 0;
    ch->CNTS.B.CNTS = (RX_BITS - 1);   // CNTS[7:0] = 23

    // NEWVAL at end of frame
    ch->IRQ.EN.B.NEWVAL_IRQ_EN = 1;
    ch->IRQ.NOTIFY.B.NEWVAL    = 1;

    IfxSrc_init(&SRC_GTM_TIM0_0, IfxSrc_Tos_cpu0, ISR_PRIORITY_RX);
    IfxSrc_enable(&SRC_GTM_TIM0_0);

    // Leave disabled for now; we enable it on start edge
    // ch->CTRL.B.TIM_EN = 1;
}

static void initTimRxTrigger(void)
{
    Ifx_GTM_TIM_CH *ch = timCh(1);

    ch->CTRL.B.TIM_EN   = 0;
    ch->CTRL.B.TIM_MODE = 0x2;   // TIEM
    ch->CTRL.B.OSM      = 1;     // one-shot

    // Use same physical RX input as CH0
    ch->CTRL.B.CICTRL   = 1;     // previous channel input (CH0)

    // Select which edge is “start”
    ch->CTRL.B.DSL      = 1;     // (example) falling edge / or rising depending on your line
    ch->CTRL.B.ISL      = 0;     // keep default unless you specifically need it

    ch->IRQ.EN.B.NEWVAL_IRQ_EN = 1;
    ch->IRQ.NOTIFY.B.NEWVAL    = 1;

    IfxSrc_init(&SRC_GTM_TIM0_1, IfxSrc_Tos_cpu0, ISR_PRIORITY_RX_START);
    IfxSrc_enable(&SRC_GTM_TIM0_1);

    ch->CTRL.B.TIM_EN = 1;
}

// =========================================================
// Pins
// =========================================================
static void initPins(void)
{
    IfxGtm_PinMap_setAtomTout(clock_pin, IfxPort_OutputMode_pushPull, IfxPort_PadDriver_cmosAutomotiveSpeed1);
    IfxGtm_PinMap_setAtomTout(debug_pin, IfxPort_OutputMode_pushPull, IfxPort_PadDriver_cmosAutomotiveSpeed1);
    IfxGtm_PinMap_setAtomTout(dir_pin,   IfxPort_OutputMode_pushPull, IfxPort_PadDriver_cmosAutomotiveSpeed1);
    IfxGtm_PinMap_setAtomTout(tx_pin,    IfxPort_OutputMode_pushPull, IfxPort_PadDriver_cmosAutomotiveSpeed1);
}

// =========================================================
// Requested functions
// =========================================================

/**
 * Stop timers, reset state, and prepare for the next transmission.r
 * Called inside ISR and can also be called from main/task context.
 */
void prepareModeTransmission(void)
{
    agc->OUTEN_STAT.U = 0x55; //disable first 4

    MODULE_GTM.CMU.CLK_EN.U = 0x5; //disable cmu0 & cmu1
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
