#include "IfxGtm.h"
#include "IfxGtm_Atom.h"
#include "IfxGtm_Tim.h"
#include "IfxGtm_Atom_Pwm.h"
#include "IfxPort.h"
#include <stdio.h>
#include "endat.h"

#define CLOCK_PIN   IfxGtm_ATOM0_0_TOUT8_P02_8_OUT
#define TX_PIN    IfxGtm_ATOM0_1_TOUT1_P02_1_OUT
#define DIR_PIN   IfxGtm_ATOM0_2_TOUT2_P02_2_OUT
#define RX_PIN  IfxGtm_TIM0_0_P02_0_IN
#define DEBUG_PIN IfxGtm_ATOM0_3_TOUT3_P02_3_OUT

#define DATA_CMU 0
#define CLOCK_CMU 1

#define FREQUENCY   100000.0f
#define SYS_FREQ    100000000.0f
#define PERIOD      (uint32)(SYS_FREQ/FREQUENCY)
#define DIRECTON_PERIOD PERIOD*14

static Ifx_GTM *gtm = &MODULE_GTM;

static void initClock(void)
{
    Ifx_GTM_ATOM *atom = &gtm->ATOM[CLOCK_PIN.atom];
    Ifx_GTM_ATOM_AGC *agc  = &atom->AGC;


    atom->CH0.CTRL.B.MODE      = 2;   // SOMP
    atom->CH0.CTRL.B.SL        = 0;
    atom->CH0.CTRL.B.CLK_SRC_SR= 0;
    atom->CH0.CTRL.B.UDMODE    = 0;
    atom->CH0.CTRL.B.OSM = 0;

    atom->CH0.SR1.U = PERIOD/2;  // duty compare (CCU1)
    atom->CH0.SR0.U = PERIOD;    // period compare (CCU0)

    atom->CH0.CTRL.B.TRIGOUT   = 1;   // <-- CCU1 compare match drives TRIG[x]
                                     // (instead of 1 = CCU0 compare match)

    agc->OUTEN_CTRL.B.OUTEN_CTRL0 = 2;
    agc->GLB_CTRL.B.UPEN_CTRL0    = 2;
    agc->ENDIS_CTRL.B.ENDIS_CTRL0 = 2;
    //agc->GLB_CTRL.B.HOST_TRIG     = 1;
}
//010111100110000111000111010

//1111001100001110001

static void initTx(void)
{
    Ifx_GTM_ATOM     *atom = &gtm->ATOM[TX_PIN.atom];
    Ifx_GTM_ATOM_AGC *agc  = &atom->AGC;

    IfxGtm_PinMap_setAtomTout(&TX_PIN, IfxPort_OutputMode_pushPull, IfxPort_PadDriver_cmosAutomotiveSpeed4);
    atom->CH1.CTRL.B.MODE = 3;
    atom->CH1.CTRL.B.ARU_EN = 0;
    atom->CH1.CTRL.B.ACB = 0;
    atom->CH1.CTRL.B.OSM = 0;
    atom->CH1.CTRL.B.SL = 1;
    // 2. Configure Clock Source to be the Trigger from Channel 0
    // According to Manual: If ECLK_SRC=1 and CLK_SRC_SR=5 (0b101), TRIG[x-1] is selected.
    atom->CH1.CTRL.B.ECLK_SRC = 1;
    atom->CH1.CTRL.B.CLK_SRC_SR = 5;

    atom->CH1.SR1.U = 0x7;
    atom->CH1.SR0.U = 5;

    agc->OUTEN_CTRL.B.OUTEN_CTRL1 = 2; //10 enable output ch1
    agc->GLB_CTRL.B.UPEN_CTRL1 = 2; // 0b10: Enable Update for CH0
    agc->ENDIS_CTRL.B.ENDIS_CTRL1 = 2; // 0b10: Enable Operation for CH0
    //agc->GLB_CTRL.B.HOST_TRIG = 1; //force trigger via cpu
}

static void initDir(void)
{
    Ifx_GTM_ATOM *atom = &gtm->ATOM[DIR_PIN.atom];
    Ifx_GTM_ATOM_AGC *agc  = &atom->AGC;

    // 1. Set Mode to SOMP (PWM) and Enable One-Shot Mode (OSM)
    atom->CH2.CTRL.B.MODE = 2;   // SOMP
    atom->CH2.CTRL.B.OSM  = 1;   // One-Shot Mode

    // 2. Set Signal Level
    // SL = 1: Pulse is High, Idle is Low.
    atom->CH2.CTRL.B.SL = 1;

    // 3. Configure Clock Source (CMU_CLK0)
    atom->CH2.CTRL.B.CLK_SRC_SR = 0;
    atom->CH2.CTRL.B.UDMODE     = 0;

    // 4. Set Timing in Shadow Registers
    // SR0 defines the full cycle (Delay + Pulse) -> 9 Periods
    atom->CH2.SR0.U = PERIOD * 9;
    // SR1 defines the Pulse End (Pulse Width) -> 8 Periods
    atom->CH2.SR1.U = PERIOD * 7;

    // 5. Enable Output and Update
    agc->OUTEN_CTRL.B.OUTEN_CTRL2 = 2; // Enable Output
    agc->GLB_CTRL.B.UPEN_CTRL2    = 2; // Enable Update for CH2
    agc->ENDIS_CTRL.B.ENDIS_CTRL2 = 2; // Enable Channel

    // --- CRITICAL FIX: FORCE SHADOW TRANSFER ---
    // This loads SR0 -> CM0 and SR1 -> CM1 immediately.
    // Without this, CM0/CM1 are 0, and the counter has nowhere to count.
    agc->GLB_CTRL.B.HOST_TRIG = 1;
}

static void initRx(void)
{
    Ifx_GTM_TIM *tim = &MODULE_GTM.TIM[0];

    // Disable during config
    tim->CH1.CTRL.B.TIM_EN = 0;

    tim->CH1.CTRL.B.TIM_MODE = 0x6;     // TSSM
    tim->CH1.CTRL.B.CLK_SEL  = 1;       // your CMU_CLK1 choice (baud clock)
    tim->CH1.CNTS.U = 23;               // bits to shift (as you had)

    // CH1 uses TIM_IN(x-1) => TIM_IN0 (same as RX pin on CH0)
    tim->CH1.CTRL.B.CICTRL = 1;

    // TSSM-specific meaning differs, but keep consistent with your setup
    tim->CH1.CTRL.B.ISL = 0;
    tim->CH1.CTRL.B.DSL = 1;

    tim->CH1.IRQ.EN.B.NEWVAL_IRQ_EN = 1;
    tim->CH1.IRQ.NOTIFY.B.NEWVAL    = 1;

    // DO NOT enable yet. We will enable after TIEM triggers.
}

static void initRxStartBitDetector(void)
{
    Ifx_GTM_TIM *tim = &MODULE_GTM.TIM[0];

    // Physical pin -> TIM0_CH0 input
    IfxGtm_PinMap_setTimTin(&RX_PIN, IfxPort_InputMode_pullUp);

    // 0) Make sure channel is disabled while configuring
    tim->CH0.CTRL.B.TIM_EN = 0;

    // 1) TIEM mode
    tim->CH0.CTRL.B.TIM_MODE = 0x2;   // TIEM

    // 2) Trigger on rising edge only
    tim->CH0.CTRL.B.ISL = 0;          // use DSL
    tim->CH0.CTRL.B.DSL = 1;          // rising edges are active

    // 3) One-shot: stop after first active edge event
    tim->CH0.CTRL.B.OSM = 1;

    // 4) Use the real input transitions (not external capture)
    tim->CH0.CTRL.B.EXT_CAP_EN = 0;

    // 5) (Optional) filter: enable only if you expect glitches
    // tim->CH0.CTRL.B.FLT_EN = 1;

    // 6) (Optional) capture a timestamp when the edge happens
    // Put TBU_TS0 into GPR0 so you can log it
    tim->CH0.CTRL.B.GPR0_SEL = 0x0;   // TS0 (basic setting; depends on EGPR0_SEL in your device)
    tim->CH0.CTRL.B.EGPR0_SEL = 0;

    // 7) Enable NEWVAL interrupt
    tim->CH0.IRQ.EN.B.NEWVAL_IRQ_EN = 1;
    tim->CH0.IRQ.NOTIFY.B.NEWVAL = 1;  // clear pending

    // 8) Enable last
    tim->CH0.CTRL.B.TIM_EN = 1;
}

static void debugCMU0(void)
{
    Ifx_GTM_ATOM *atom = &gtm->ATOM[DEBUG_PIN.atom];
    Ifx_GTM_ATOM_AGC *agc  = &atom->AGC;



    atom->CH3.CTRL.B.MODE      = 2;   // SOMP
    atom->CH3.CTRL.B.SL        = 0;
    atom->CH3.CTRL.B.CLK_SRC_SR= 1;
    atom->CH3.CTRL.B.UDMODE    = 0;
    atom->CH3.CTRL.B.OSM = 0;

    atom->CH3.SR1.U = 1;  // duty compare (CCU1)
    atom->CH3.SR0.U = 2;    // period compare (CCU0)

    agc->OUTEN_CTRL.B.OUTEN_CTRL3 = 2;
    agc->GLB_CTRL.B.UPEN_CTRL3    = 2;
    agc->ENDIS_CTRL.B.ENDIS_CTRL3 = 2;
    //agc->GLB_CTRL.B.HOST_TRIG     = 1;
}

void init(void)
{

    IfxGtm_enable(gtm);
    float32 mod = IfxGtm_Cmu_getModuleFrequency(gtm);
    IfxGtm_Cmu_setGclkFrequency(gtm, mod);

    IfxGtm_Cmu_setClkFrequency(gtm, IfxGtm_Cmu_Clk_0, mod);
    IfxGtm_Cmu_setClkFrequency(gtm, IfxGtm_Cmu_Clk_1, FREQUENCY);



    initTx();
    initDir();
    initClock();
    initRx();
    initRxStartBitDetector();
    debugCMU0();

    // --- START SEQUENCE SYNCHRONIZATION ---

    Ifx_GTM_ATOM *atom = &gtm->ATOM[DIR_PIN.atom];

    // 1. Trigger the Direction Pin (Start One-Shot)
    // To get a delay of 1 PERIOD, we must offset the counter start.
    // Formula: Delay = CM0 - Start_CN0
    // Start_CN0 = CM0 - Delay
    // Start_CN0 = (9 * PERIOD) - (1 * PERIOD) = 8 * PERIOD
    atom->CH2.CN0.U = PERIOD * 7;

    // 2. Trigger the Clock (Master)
    // Host Trigger starts the Clock generation immediately after Dir is primed
    atom->CH3.CN0.U = 0;
    gtm->ATOM[0].AGC.GLB_CTRL.B.HOST_TRIG = 1;

    MODULE_GTM.CMU.CLK_EN.U = (2u << 0) | (2u << 2);
    IfxGtm_PinMap_setAtomTout(&CLOCK_PIN, IfxPort_OutputMode_pushPull, IfxPort_PadDriver_cmosAutomotiveSpeed4);
    IfxGtm_PinMap_setAtomTout(&DEBUG_PIN, IfxPort_OutputMode_pushPull, IfxPort_PadDriver_cmosAutomotiveSpeed4);
    IfxGtm_PinMap_setAtomTout(&DIR_PIN, IfxPort_OutputMode_pushPull, IfxPort_PadDriver_cmosAutomotiveSpeed4);



    /*
    while (1)
    {
        // First rising edge detector (TIEM CH0)
        if (MODULE_GTM.TIM[0].CH0.IRQ.NOTIFY.B.NEWVAL)
        {
            MODULE_GTM.TIM[0].CH0.IRQ.NOTIFY.B.NEWVAL = 1;

            // (Optional) read timestamp captured into GPR0
            uint32 ts0 = MODULE_GTM.TIM[0].CH0.GPR0.U;

            // --- Your "trigger" action for the scope ---
            // e.g. toggle a port pin or force an ATOM output
            // IfxPort_togglePin(TRIG_PORT, TRIG_PIN);

            printf("Startbit rising edge detected, TS0=0x%06X\n", ts0 & 0x00FFFFFF);
            // Start the serial shifter now
            MODULE_GTM.TIM[0].CH1.CTRL.B.TIM_EN = 1;
        }

        // Your existing "data ready" (TSSM CH1)
        if (MODULE_GTM.TIM[0].CH1.IRQ.NOTIFY.B.NEWVAL)
        {
            MODULE_GTM.TIM[0].CH1.IRQ.NOTIFY.B.NEWVAL = 1;

            uint32 raw = MODULE_GTM.TIM[0].CH1.GPR0.U; // (often shift result lives in GPRx in TSSM)
            printf("RX word: 0x%08X\n", raw);
        }
    }
    */
}
