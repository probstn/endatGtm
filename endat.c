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

    IfxGtm_PinMap_setAtomTout(&CLOCK_PIN, IfxPort_OutputMode_pushPull, IfxPort_PadDriver_cmosAutomotiveSpeed4);

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

    IfxGtm_PinMap_setAtomTout(&DIR_PIN, IfxPort_OutputMode_pushPull, IfxPort_PadDriver_cmosAutomotiveSpeed4);

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
    Ifx_GTM_TIM *tim = &gtm->TIM[RX_PIN.tim];
    IfxGtm_PinMap_setTimTin(&RX_PIN, IfxPort_InputMode_pullUp);

    // 1. Enable TIM Channel
    tim->CH0.CTRL.B.TIM_EN = 1;

    // 2. Set Mode to TSSM (TIM Serial Shift Mode) -> 0x6 (110b)
    tim->CH0.CTRL.B.TIM_MODE = 0x6;

    // 3. Configure Internal Shift Clock
    // We are NOT using an external clock pin, so we must match the Baud Rate using CMU_CLK.
    // CLK_SEL = 1 selects CMU_CLK_1 (which we set to 100kHz in main)
    tim->CH0.CTRL.B.CLK_SEL = 1;

    // 4. Configure TSSM Control (CNTS)
    // [7:0]   = Word Length (Bits to shift before interrupt)
    // [17:16] = Shift Clock Source (00 = Use CLK_SEL)
    tim->CH0.CNTS.U = 23;

    // 5. General Settings
    tim->CH0.CTRL.B.ISL = 0; // Use input pin F_OUT for data
    tim->CH0.CTRL.B.DSL = 1; // 1 = Shift Right (Matches TX SL=1)

    // 6. Enable Interrupt Notification for New Value
    tim->CH0.IRQ.EN.B.NEWVAL_IRQ_EN = 1;
    tim->CH0.IRQ.NOTIFY.B.NEWVAL    = 1; // Clear any pending flag
}

void init(void)
{
    IfxGtm_enable(gtm);

    float32 mod = IfxGtm_Cmu_getModuleFrequency(gtm);
    IfxGtm_Cmu_setGclkFrequency(gtm, mod);

    IfxGtm_Cmu_setClkFrequency(gtm, IfxGtm_Cmu_Clk_0, mod);
    IfxGtm_Cmu_setClkFrequency(gtm, IfxGtm_Cmu_Clk_1, FREQUENCY+1000);

    IfxGtm_Cmu_enableClocks(gtm, IFXGTM_CMU_CLKEN_CLK0 | IFXGTM_CMU_CLKEN_CLK1);

    initTx();
    initDir();
    initClock();
    initRx();

    // Startup Delay
    for(int i = 0; i < 1e6; i++) __nop();

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
    gtm->ATOM[0].AGC.GLB_CTRL.B.HOST_TRIG = 1;
    printf("Init done\n");

    while(1)
    {
        if (gtm->TIM[RX_PIN.tim].CH0.IRQ.NOTIFY.B.NEWVAL)
        {
            // GPR1 contains: [ECNT (8 bits) | DATA (24 bits)]
            uint32 rawValue = gtm->TIM[RX_PIN.tim].CH0.CNT.U;

            // Mask out the top 8 bits to see just the data
            uint32 dataOnly = rawValue & 0x00FFFFFF;

            gtm->TIM[RX_PIN.tim].CH0.IRQ.NOTIFY.B.NEWVAL = 1;

            printf("Raw: 0x%08X | Data: 0x%06X\n", rawValue, dataOnly);
        }
    }
}
