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
#include <Qspi/SpiSlave/IfxQspi_SpiSlave.h>


// ---------------------- CONFIGURATION & DEFINES (Moved to Top) ----------------------
#define FREQUENCY   1e5
#define SYS_FREQ    100e6
#define PERIOD      ((uint32)(SYS_FREQ / FREQUENCY))

// ---------------------- HW handles ----------------------
static Ifx_GTM *gtm = &MODULE_GTM;
static Ifx_GTM_ATOM     *atom = NULL_PTR;
static Ifx_GTM_ATOM_AGC *agc  = NULL_PTR;
static Ifx_GTM_TIM      *tim  = NULL_PTR;






//---------------------------PINS---------------------------
static const IfxGtm_Atom_ToutMap *ss_pin        =  &IfxGtm_ATOM0_0_TOUT0_P02_0_OUT;
static const IfxGtm_Atom_ToutMap *dir_pin       =  &IfxGtm_ATOM0_1_TOUT1_P02_1_OUT;
static const IfxGtm_Atom_ToutMap *tx_pin        =  &IfxGtm_ATOM0_2_TOUT2_P02_2_OUT;
static const IfxGtm_Atom_ToutMap *tx_clk_pin    =  &IfxGtm_ATOM0_3_TOUT3_P02_3_OUT;
static const IfxGtm_Atom_ToutMap *rx_clk_pin    =  &IfxGtm_ATOM0_4_TOUT4_P02_4_OUT;

static const IfxGtm_Tim_TinMap   *rx_startbit    = &IfxGtm_TIM0_0_P33_4_IN;

#define DBG_START_PIN     &MODULE_P02,7





// ----------------------- QSPI SLAVE (RX only, 26-bit framed by SLS) ----------------
#define SPI_WORDS_PER_FRAME   1u

static uint32 spiRxWord32 = 0;                 // received 26-bit word in LSBs
static volatile boolean spiWordReady = FALSE;  // set when 1-word exchange finished

static void armSpiReceive26(void);
static void print26(uint32 w);

static IfxQspi_SpiSlave spi;

// priorities (keep yours or define them)
#define IFX_INTPRIO_QSPI4_TX  1
#define IFX_INTPRIO_QSPI4_RX  2
#define IFX_INTPRIO_QSPI4_ER  5

IFX_INTERRUPT(qspi4TxISR, 0, IFX_INTPRIO_QSPI4_TX)
{
    IfxQspi_SpiSlave_isrTransmit(&spi);
}

IFX_INTERRUPT(qspi4RxISR, 0, IFX_INTPRIO_QSPI4_RX)
{
    IfxQspi_SpiSlave_isrReceive(&spi);

    // Exchange finished => driver not busy anymore
    if (IfxQspi_SpiSlave_getStatus(&spi) != IfxQspi_Status_busy)
    {
        spiWordReady = TRUE;
    }
}

IFX_INTERRUPT(qspi4ErISR, 0, IFX_INTPRIO_QSPI4_ER)
{
    IfxQspi_SpiSlave_isrError(&spi);
}

static inline boolean isSlsActive(void)
{
    // SLS is typically active-low. Your pin is pullDown currently, but the master drives it.
    // Return TRUE while CS asserted (LOW).
    return (IfxPort_getPinState(&MODULE_P33, 10) == 0) ? TRUE : FALSE;
}

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
void fireTransmission(void);
static void initAtomRxClock(void);
static void initAtomSlaveSelect(void);
static void initSPISlave(void);

#define ISR_PRIORITY_ATOM0_CH1_CCU0TC  10
IFX_INTERRUPT(atomDirShiftCompleteIsr, 0, ISR_PRIORITY_ATOM0_CH1_CCU0TC);

void atomDirShiftCompleteIsr(void)
{
//    initTimStartBitDetect();

    IfxPort_togglePin(DBG_START_PIN);
    initTimStartBitDetect();
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
//    initDmaRx();
    initSPISlave();
    initPins();

    IfxPort_setPinModeOutput(DBG_START_PIN, IfxPort_OutputMode_pushPull, IfxPort_OutputIdx_general);

    fireTransmission();

    while (TRUE)
    {
        if (spiWordReady)
        {
            spiWordReady = FALSE;

            // Ensure the frame is actually finished (CS returned inactive)
            while (isSlsActive())
            {
                /* spin */
            }

            print26(spiRxWord32);

            // arm for next framed 26-bit word
            armSpiReceive26();
        }
    }
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

    agc->OUTEN_CTRL.B.OUTEN_CTRL3 = 2;  // enable output
    agc->GLB_CTRL.B.UPEN_CTRL3    = 2;  // enable update
    agc->ENDIS_CTRL.B.ENDIS_CTRL3 = 2;  // enable operation
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
//    ch->CN0.U = PERIOD/2-1;

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

    //Attention: Interrupt sources != Atom channels! multiple atom channels share same interrupt source.
    IfxSrc_init(&SRC_GTM_ATOM0_0, IfxSrc_Tos_cpu0, ISR_PRIORITY_ATOM0_CH1_CCU0TC);
    IfxSrc_enable(&SRC_GTM_ATOM0_0);
    // 2. Route the interrupt to CPU0 (Assuming ATOM0 and Channel 3)
    // Make sure SRC_GTM_ATOM0_3 matches your actual ATOM instance and channel!

    agc->OUTEN_CTRL.B.OUTEN_CTRL1 = 2;
    agc->GLB_CTRL.B.UPEN_CTRL1    = 2;
    agc->ENDIS_CTRL.B.ENDIS_CTRL1 = 2;
    agc->FUPD_CTRL.B.RSTCN0_CH1 = 2;
}

static void initTimStartBitDetect(void)
{
    Ifx_GTM_TIM_CH *ch = timCh(rx_startbit->channel);

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
    ch->CN0.U = totalCycles - 1; // Starts at 27. Rolls to 0 on first tick.

    agc->OUTEN_CTRL.B.OUTEN_CTRL0 = 2;
    agc->ENDIS_CTRL.B.ENDIS_CTRL0 = 2;
    agc->FUPD_CTRL.B.FUPD_CTRL0 = 2;
}

static void initSPISlave(void)
{
    IfxQspi_SpiSlave_Config cfg;
    IfxQspi_SpiSlave_initModuleConfig(&cfg, &MODULE_QSPI4);

    cfg.maximumBaudrate = 16000000;

    // --- 26-bit words ---
    cfg.protocol.dataWidth    = 26;
    cfg.protocol.dataHeading  = IfxQspi_DataHeading_msbFirst;  // change if your master is LSB first
    cfg.protocol.clockPolarity = IfxQspi_ClockPolarity_idleLow; // set to match master (CPOL)
    cfg.protocol.shiftClock    = IfxQspi_ShiftClock_shiftTransmitDataOnLeadingEdge; // match master (CPHA)

    // --- Interrupts (depends on iLLD version) ---
#if defined(IFXQSPIS_SPISLAVE_CONFIG_HAS_BASE) || defined(IfxQspi_SpiSlave_BaseConfig) /* if your headers indicate base */
    cfg.base.txPriority  = IFX_INTPRIO_QSPI4_TX;
    cfg.base.rxPriority  = IFX_INTPRIO_QSPI4_RX;
    cfg.base.erPriority  = IFX_INTPRIO_QSPI4_ER;
    cfg.base.isrProvider = IfxSrc_Tos_cpu0;
#else
    cfg.txPriority  = IFX_INTPRIO_QSPI4_TX;
    cfg.rxPriority  = IFX_INTPRIO_QSPI4_RX;
    cfg.erPriority  = IFX_INTPRIO_QSPI4_ER;
    cfg.isrProvider = IfxSrc_Tos_cpu0;
#endif

    // --- Pins (QSPI4) ---
    static const IfxQspi_SpiSlave_Pins slavePins = {
        &IfxQspi4_SCLKA_P33_11_IN, IfxPort_InputMode_pullDown,      // SCLK
        &IfxQspi4_MTSRA_P33_12_IN, IfxPort_InputMode_pullDown,      // MOSI (Master->Slave)
        &IfxQspi4_MRST_P33_13_OUT, IfxPort_OutputMode_pushPull,     // MISO (ignored in RX-only)
        &IfxQspi4_SLSIA_P33_10_IN, IfxPort_InputMode_pullUp,        // SLS (CS) - recommend pullUp for active-low CS
        IfxPort_PadDriver_cmosAutomotiveSpeed3
    };
    cfg.pins = &slavePins;

    IfxQspi_SpiSlave_initModule(&spi, &cfg);

    // Arm first 26-bit frame (1 word)
    armSpiReceive26();
}

static void armSpiReceive26(void)
{
    spiWordReady = FALSE;
    spiRxWord32  = 0;

    // exactly one 26-bit "word" per CS frame
    IfxQspi_SpiSlave_exchange(&spi, NULL_PTR, &spiRxWord32, SPI_WORDS_PER_FRAME);
}
static void print26(uint32 w)
{
    w &= ((1u << 26) - 1u);

    printf("RX26: 0b");
    for (int b = 25; b >= 0; --b)
    {
        printf("%u", (unsigned)((w >> b) & 1u));
    }
    printf(" (0x%08" PRIX32 ")\n", w);
}

void fireTransmission(void)
{
    atom->CH1.CN0.U = 0;
    atom->CH2.CN0.U = 0;
    agc->GLB_CTRL.B.HOST_TRIG = 1;     // now start is synchronous and deterministic
}
static void initPins(void)
{
    IfxGtm_PinMap_setAtomTout(ss_pin, IfxPort_OutputMode_pushPull, IfxPort_PadDriver_cmosAutomotiveSpeed1);
    IfxGtm_PinMap_setAtomTout(tx_clk_pin,   IfxPort_OutputMode_pushPull, IfxPort_PadDriver_cmosAutomotiveSpeed1);
    IfxGtm_PinMap_setAtomTout(tx_pin,   IfxPort_OutputMode_pushPull, IfxPort_PadDriver_cmosAutomotiveSpeed1);
    IfxGtm_PinMap_setAtomTout(dir_pin,    IfxPort_OutputMode_pushPull, IfxPort_PadDriver_cmosAutomotiveSpeed1);
    IfxGtm_PinMap_setAtomTout(rx_clk_pin,    IfxPort_OutputMode_pushPull, IfxPort_PadDriver_cmosAutomotiveSpeed1);

    IfxGtm_PinMap_setTimTin(rx_startbit, IfxPort_InputMode_noPullDevice);
}
/*
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
*/
