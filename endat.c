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
#define FREQUENCY   1e6
#define SYS_FREQ    96e6
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
static void initTimRx_Tdu_2x13(void);
static void initTimRx_Tssm(void);
static void initPins(void);
static void initDmaRx(void);
unsigned int MakeCrcPos(unsigned int clocks, unsigned int error1, unsigned int error2, unsigned int endat22, unsigned long highpos, unsigned long lowpos);

// Requested API
void prepareModeTransmission(void);
void fireTransmission(void);

// =========================================================
// INTERRUPT SERVICE ROUTINES
// =========================================================
#define ISR_PRIORITY_TIM0_CH0_NEWVAL  11

static volatile uint32 pos_frame = 0;
volatile int captured_words = 0;

IFX_INTERRUPT(timRxWordDoneISR, 0, ISR_PRIORITY_TIM0_CH0_NEWVAL);
void timRxWordDoneISR(void)
{
    //if(captured_words > 1) return;
    IfxPort_togglePin(DBG_DMA_PIN);
    //uint16_t word = (tim->CH0.GPR1.U >> 11) & 0x1FFF;
    //pos_frame |= (captured_words++ == 0) ? (word) : (word << 13);
    IfxPort_togglePin(DBG_DMA_PIN);
}

// 1. DMA Interrupt: Fires after 2 segments are received (TCOUNT reaches 0)
IFX_INTERRUPT(dmaRxISR, 0, ISR_PRIORITY_DMA_RX);

void dmaRxISR(void)
{
    IfxPort_togglePin(DBG_DMA_PIN);

    /*

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

    //IfxSrc_clearRequest(&SRC_GTM_TIM0_1);

    /* Now start bit clock + shifter */
    //MODULE_GTM.CMU.CLK_EN.U |= (1u << 1);
    //MODULE_GTM.TIM[0].CH0.CTRL.B.TIM_EN = 1;
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

    atom->CH0.CN0.U = 0;
    atom->CH1.CN0.U = 0;
    atom->CH2.CN0.U = 0;
    agc->GLB_CTRL.B.HOST_TRIG = 1;     // now start is synchronous and deterministic
    //agc->GLB_CTRL.B.HOST_TRIG = 1;     // now start is synchronous and deterministic
}

uint8_t reverse5(uint8_t b) {
    b = ((b & 0xF) << 4) | ((b & 0xF0) >> 4); // Swap nibbles (only low 4 relevant first)
    // Actually simpler for 5 bits:
    uint8_t r = 0;
    if (b & 0x01) r |= 0x10; // Bit 0 -> Bit 4
    if (b & 0x02) r |= 0x08; // Bit 1 -> Bit 3
    if (b & 0x04) r |= 0x04; // Bit 2 -> Bit 2
    if (b & 0x08) r |= 0x02; // Bit 3 -> Bit 1
    if (b & 0x10) r |= 0x01; // Bit 4 -> Bit 0
    return r;
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

    initTimRx_Tssm();
    initTimRx_Tdu_2x13();
    initDmaRx();

    initPins();
    IfxPort_setPinModeOutput(DBG_DMA_PIN, IfxPort_OutputMode_pushPull, IfxPort_OutputIdx_general);
    IfxPort_setPinModeOutput(DBG_START_PIN, IfxPort_OutputMode_pushPull, IfxPort_OutputIdx_general);

    //agc->GLB_CTRL.B.HOST_TRIG = 1; //update all shadows

    // Pre-arm the DMA for the very first run
    //prepareModeTransmission();

    fireTransmission();

    while(true)
    {

        if(captured_words > 1)
        {
            printf("%X\n", pos_frame & 0x3FFFFFF);
            bool startbit = pos_frame & 1u;
            bool faultbit = (pos_frame >> 1) & 1u;

            uint32 position = (pos_frame >> 2) & 0x7FFFF;
            //printf("%X\n", position);

            uint8 crc = (pos_frame >> 21) & 0x1F; // 5-bit CRC
            //uint8_t crc_calc = MakeCrcPos(19, faultbit, 0, 0, 0, position);
            //printf("%.5X\n", crc);
            //printf("%d %d %d %d\n", startbit, faultbit, position, crc);
            break;
        }
    }
}

unsigned int MakeCrcPos(unsigned int clocks, unsigned int error1, unsigned int error2, unsigned int endat22, unsigned long highpos, unsigned long lowpos)
{
    unsigned int ff[5]; // Zustand der 5 Flip-Flops
    unsigned int code[66]; // Datenbit-Array
    unsigned int ex; // Hilfsvariable
    unsigned int crc = 0; // ermittelter CRC-Code
    signed int i; // Laufvariable fC<r Schleifen

    for(i = 0; i < 5; i++) // alle Flip-Flops auf 1 setzen
        ff[i] = 1;
    if (endat22) // alarm-Bits ins code-Array einlesen
    {
        code[0] = error1;
        code[1] = error2;
    }
    else
        code[1] = error1;
    for(i = 2; i < 34; i++) // lowpos-Bits ins code-Array einlesen
    {
        code[i] = (lowpos & 0x00000001L) ? 1 : 0;
        lowpos >>= 1;
    }
    for(i = 34; i < 66; i++) // highpos-Bits ins code-Array einlesen
    {
        code[i] = (highpos & 0x00000001L) ? 1 : 0;
        highpos >>= 1;
    }
    for(i = (endat22 ? 0 : 1); i <= (clocks+1); i++)
    {   // CRC berechnen, analog zur
        ex = ff[4] ^ code[i]; // beschriebenen Generator-Hardware
        ff[4] = ff[3];
        ff[3] = ff[2] ^ ex;
        ff[2] = ff[1];
        ff[1] = ff[0] ^ ex;
        ff[0] = ex;
    }
    for(i = 4; i >= 0; i--) // CRC in Variable ablegen
    {
        ff[i] = ff[i] ? 0 : 1; // Bits invertieren
        crc <<= 1;
        crc |= ff[i];
    }
    return crc;
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
    // CMU_CLK0 = fast (for debug, etc.)
    IfxGtm_Cmu_setClkFrequency(gtm, IfxGtm_Cmu_Clk_0, SYS_FREQ);
    MODULE_GTM.CMU.CLK_EN.U = 0xA;     // enable clocks FIRST

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
    ch->CTRL.B.CLK_SRC_SR = 0; //CMU0

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
    ch->CTRL.B.TRIGOUT    = 0;  // TRIG_[1] forwards TRIG_[0]
    ch->CTRL.B.EXTTRIGOUT = 0;  // choose TRIG_[x-1] as forwarded signal

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

    agc->OUTEN_CTRL.B.OUTEN_CTRL2 = 2;
    agc->GLB_CTRL.B.UPEN_CTRL2    = 2;
    agc->ENDIS_CTRL.B.ENDIS_CTRL2 = 2;
    agc->FUPD_CTRL.B.RSTCN0_CH2 = 2;
}

// You must choose these based on your protocol timing:
#define BIT_TICKS     16u   // example: sample every 16 ticks of CMU_TDU_CLK
#define PHASE_TICKS   8u    // example: first sample 8 ticks after start edge

static void initTimRx_Tdu_2x13(void)
{
    Ifx_GTM_TIM_CH *ch = timCh(0);

    // Disable channel while programming TDU/ECTRL
    ch->CTRL.B.TIM_EN = 0;

    // ------------- Select which edges drive the TDU "active edge" -------------
    // CTRL.TOCTRL: 01 = enabled for rising edge only (your pasted field)
    ch->CTRL.B.TOCTRL = 0x1;   // startbit is rising edge

    // ------------- TDU clock + slicing -------------
    // TDUV.TCS: 001 = CMU_CLK1 (your pasted table)
    ch->TDUV.B.TCS = 0x1;

    // TDUV.SLICING: 10 = 3x8-bit counters (your pasted table)
    ch->TDUV.B.SLICING = 0x2;

    // Use tdu_sample_evt as Timeout Clock for TO_CNT (TO_CNT2 still uses CMU_CLK(TCS))
    // This creates: TO_CNT2 (CMU) -> tdu_sample_evt; TO_CNT counts those sample events.
    ch->TDUV.B.TCS_USE_SAMPLE_EVT = 1;

    // Ensure TO_CNT1 is clocked on tdu_word_evt (default behavior when TDU_SAME_CNT_CLK=0)
    ch->TDUV.B.TDU_SAME_CNT_CLK = 0;

    // ------------- Program compare values -------------
    // With SLICING=10 and TCS_USE_SAMPLE_EVT=1:
    // - TO_CNT2 clocked by CMU_CLK1: its compare defines sample cadence
    // - TO_CNT clocked by tdu_sample_evt: its compare defines word cadence
    // - TO_CNT1 clocked by tdu_word_evt: its compare defines frame cadence

    // Make tdu_sample_evt happen EVERY 1 CMU_CLK1 tick => 16 MHz sample pulses
    // (compare at 0 means "every tick" in typical compare-to-zero style; if your silicon behaves as N+1,
    // set to 0 for 1-tick period, set to (period-1) for N-tick period.)
    ch->TDUV.B.TOV2 = 0;                    // sample period = 1 tick of CMU_CLK1 (16 MHz)

    // Make tdu_word_evt after 13 samples
    ch->TDUV.B.TOV  = (RX_WORD_LENGTH_BITS - 1);   // 12 => 13 sample events

    // Make tdu_frame_evt after 2 words
    ch->TDUV.B.TOV1 = (RX_SEGS - 1);        // 1 => 2 word events

    // ------------- Start/Stop/Resync policy -------------
    // Start once on first active edge selected by TOCTRL (rising) (your pasted encoding)
    ch->ECTRL.B.TDU_START = 0x3;

    // Stop on tdu_frame_evt (after 2 words) (your pasted encoding)
    ch->ECTRL.B.TDU_STOP  = 0x2;

    // Resync: simplest deterministic choice for slicing!=11 is 0000:
    // resets counters on each active edge selected by TOCTRL etc. (your Table 46)
    // This ensures the chain always starts aligned to the detected start edge.
    ch->ECTRL.B.TDU_RESYNC = 0x0;

    // Optional: Make TODET_IRQ represent "frame done" (tdu_frame_evt)
    // This gives you a single “frame done” notification source (IRQ/DMA) without touching NEWVAL.
    ch->ECTRL.B.TODET_IRQ_SRC = 0x2; // 10 = tdu_frame_evt (from your pasted table)

    // Re-enable channel
    ch->CTRL.B.TIM_EN = 1;
}

static void initTimRx_Tssm(void)
{
    Ifx_GTM_TIM_CH *ch = timCh(0);

    IfxGtm_PinMap_setTimTin(rx_pin, IfxPort_InputMode_noPullDevice);

    ch->CTRL.B.TIM_EN = 0;

    // --- TSSM setup ---
    ch->CTRL.B.TIM_MODE   = 0x6;   // TSSM
    ch->CTRL.B.DSL        = 1;     // shift-right (new bit into CNT[23])
    ch->CTRL.B.ISL        = 0;     // use filtered input (F_OUT) as data source
    ch->CTRL.B.CNTS_SEL   = 0;     // in TSSM: shift-out source selection (leave default unless needed)

    // Capture CNT into GPR1 on NEWVAL
    ch->CTRL.B.EGPR1_SEL  = 0;
    ch->CTRL.B.GPR1_SEL   = 0x3;   // CNT -> GPR1

    // Word length = 13 bits
    ch->CNTS.U = 0;
    ch->CNTS.B.CNTS = (RX_WORD_LENGTH_BITS - 1);

    // --- External capture mode: shifting only on EXT_CAP pulses ---
    ch->CTRL.B.EXT_CAP_EN = 1;     // “input event changes ignored; only sensitive to external capture pulses”

    // EXT_CAP source = local tdu_sample_evt (0xC)  (from your ECTRL table)
    ch->ECTRL.B.EXT_CAP_SRC = 0xC;

    // No fast ISR:
    ch->IRQ.EN.B.NEWVAL_IRQ_EN = 0;
    ch->IRQ.EN.B.TODET_IRQ_EN  = 0;  // we'll use DMA/one IRQ later if desired

    ch->CTRL.B.TIM_EN = 1;
}

static void initPins(void) {
    IfxGtm_PinMap_setAtomTout(clock_pin, IfxPort_OutputMode_pushPull, IfxPort_PadDriver_cmosAutomotiveSpeed1);
    IfxGtm_PinMap_setAtomTout(cmu_pin, IfxPort_OutputMode_pushPull, IfxPort_PadDriver_cmosAutomotiveSpeed1);
    IfxGtm_PinMap_setAtomTout(dir_pin,   IfxPort_OutputMode_pushPull, IfxPort_PadDriver_cmosAutomotiveSpeed1);
    IfxGtm_PinMap_setAtomTout(tx_pin,    IfxPort_OutputMode_pushPull, IfxPort_PadDriver_cmosAutomotiveSpeed1);
}
