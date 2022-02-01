//#############################################################################
//
// FILE:   adc_ex1_soc_epwm.c
//
// TITLE:  ADC ePWM Triggering
//
//! \addtogroup bitfield_example_list
//! <h1>ADC ePWM Triggering</h1>
//!
//! This example sets up ePWM1 to periodically trigger a conversion on ADCA.
//!
//! \b External \b Connections \n
//!  - A1 should be connected to a signal to convert
//!
//! \b Watch \b Variables \n
//! - \b adcAResults - A sequence of analog-to-digital conversion samples from
//!   pin A1. The time between samples is determined based on the period
//!   of the ePWM timer.
//!
//
//#############################################################################
//
//
// $Copyright:
// Copyright (C) 2021 Texas Instruments Incorporated - http://www.ti.com/
//
// Redistribution and use in source and binary forms, with or without 
// modification, are permitted provided that the following conditions 
// are met:
// 
//   Redistributions of source code must retain the above copyright 
//   notice, this list of conditions and the following disclaimer.
// 
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the 
//   documentation and/or other materials provided with the   
//   distribution.
// 
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// $
//#############################################################################

//
// Included Files
//
#include "F28x_Project.h"
#include "math.h"

//
// Defines
//
#define RESULTS_BUFFER_SIZE     256
#define N_r 5000.0 // aka TBPRD; 5000 --> 10 kHz || 50,000 --> f_1 = f_PWM = 1 kHz & f_SAMP = 2 kHz --> Nyquist freq = 2 kHz
#define pi 3.141592653589

// NOTES:
// f_1 = 1 kHz
// Nyquist freq = 2 * f_1 = 2 kHz

//
// Globals
//

uint16_t adcAResults[RESULTS_BUFFER_SIZE];   // Buffer for results
uint16_t index;                              // Index into result buffer
volatile uint16_t bufferFull;                // Flag to indicate buffer is full

uint16_t temp_ADC = 0;
float duty = 1; // duty cycle
float dt = 0.0;
float Wn = 100*2*pi; // 100*2*pi/90.9; // angular frequency
float m = 0.5;

float fsamp = 20000; // 220
float Ts = 0;


//
// Function Prototypes
//
void initADC(void);
void initDAC(void);
void initEPWM(void);
void initADCSOC(void);
__interrupt void adcA1ISR(void);

//
// Main
//
void main(void)
{
    //
    // Initialize device clock and peripherals
    //
    InitSysCtrl();

    //
    // Initialize GPIO
    //
    InitGpio();

    //
    // Disable CPU interrupts
    //
    DINT;

    //
    // Initialize the PIE control registers to their default state.
    // The default state is all PIE interrupts disabled and flags
    // are cleared.
    //
    InitPieCtrl();

    //
    // Disable CPU interrupts and clear all CPU interrupt flags:
    //
    IER = 0x0000;
    IFR = 0x0000;

    //
    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    //
    InitPieVectTable();

    //
    // Map ISR functions
    //
    EALLOW;
    PieVectTable.ADCA1_INT = &adcA1ISR;     // Function for ADCA interrupt 1
    EDIS;

    //
    // Configure the ADC and power it up
    //
    initADC();

    //
    // Configure the ePWM
    //
    initEPWM();

    //
    // Configure the DAC and power it up
    //
    initDAC();

    //
    // Setup the ADC for ePWM triggered conversions on channel 1
    //
    initADCSOC();

    //
    // Enable global Interrupts and higher priority real-time debug events:
    //
    IER |= M_INT1;  // Enable group 1 interrupts

    EINT;           // Enable Global interrupt INTM
    ERTM;           // Enable Global realtime interrupt DBGM

    //
    // Initialize results buffer
    //
    for(index = 0; index < RESULTS_BUFFER_SIZE; index++)
    {
        adcAResults[index] = 0;
    }

    index = 0;
    bufferFull = 0;

    //
    // Enable PIE interrupt
    //
    PieCtrlRegs.PIEIER1.bit.INTx1 = 1;

    //
    // Sync ePWM
    //
    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;

    //
    // Take conversions indefinitely in loop
    //
    while(1)
    {

        // TOEDIT
        EPwm1Regs.CMPA.bit.CMPA = duty * N_r;


    }
}

//
// initADC - Function to configure and power up ADCA.
//
void initADC(void)
{
    //
    // Setup VREF as internal
    //
    SetVREF(ADC_ADCA, ADC_INTERNAL, ADC_VREF3P3);

    EALLOW;

    //
    // Set ADCCLK divider to /4
    //
    AdcaRegs.ADCCTL2.bit.PRESCALE = 0; // 0000: ADCCLK = Input Clock / 1.0

    //
    // Set pulse positions to late
    //
    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;

    //
    // Power up the ADC and then delay for 1 ms
    //
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    EDIS;

    DELAY_US(1000);
}

//
// initEPWM - Function to configure ePWM1 to generate the SOC.
//
void initEPWM(void)
{
    // Emulation Allow --> enable write access to protected space (i.e., protected registers)
    EALLOW;

    /////////////////////////////////////////////
    /// TIME BASED CONFIGURATION
    /////////////////////////////////////////////

    /*
     *  TBCLK --> time duration of a single step of the carrier
     *  EPWMCLK --> system clock (i.e., always 100 MHz)
     *
     *  TBCLK = EPWMCLK / (HSPCLKDIV x CLKDIV)
     *      ==> (1 / 100 MHz) = (1 / 100 MHz) / (1 x 1)
     *
     */

    // Time-Based Control Register Configuration
        // High Speed Clock Divider --> High Speed Time Base Clock Pre-Scale Bits
    EPwm1Regs.TBCTL.bit.HSPCLKDIV = 0b000;  // 0b000: 1
        // Clock Divider --> Time Base Clock Pre-Scale Bits
    EPwm1Regs.TBCTL.bit.CLKDIV = 0b000;     // 0b000: 1 (default on reset)
        // Counter Mode
    EPwm1Regs.TBCTL.bit.CTRMODE = 0b10;     // 0b10: Up-down count mode


    /*
     * T_PWM --> period of the carrier signal which is also the period of the PWM signal
     *
     * T_PWM = 2 * N * T_tbclk = 2 * TBPRD * TBCLK
     *      ==> (1 / 10kHz) = 2 * TBPRD * (1 / 100MHz)
     *      ==> TBPRD = 5000
     */

    // Time-Based Period --> determines the period of the time-base counter --> sets the PWM frequency
    EPwm1Regs.TBPRD = N_r;


    /////////////////////////////////////////////
    /// ACTION QUALIFIER CONTROL CONFIGURATION
    /////////////////////////////////////////////

    // AQCTLA --> Action Qualifier Control for group A
        // CAU --> CMPA on Up-count --> Action when TBCTR = CMPA on up count (compare with value stored in register A on up-count)
    EPwm1Regs.AQCTLA.bit.CAU = 0b01;    // 0b11: Toggle EPWMxA output (low forced high & high forced low) || 0b01: Clear: force EPWMxA output low
        // CAD --> CMPA on Down-count --> Action when TBCTR = CMPA on down count (compare with value stored in register A on down-count)
    EPwm1Regs.AQCTLA.bit.CAD = 0b10;    // 0b11: Toggle EPWMxA output (low forced high & high forced low) || 0b10: Set: force EPWMxA output high.


    /////////////////////////////////////////////
    /// DEAD BAND CONFIGURATION
    /////////////////////////////////////////////

    // DBCTL --> Dead-Band Control
        // Dead-Band Input Mode Control
    EPwm1Regs.DBCTL.bit.IN_MODE = 0b00;     // 0b00: EPWMxA In (from the action-qualifier) is the source for both falling-edge and rising-edge delay.
        // Polarity Select Control
    EPwm1Regs.DBCTL.bit.POLSEL = 0b10;      // 0b10: Active high complementary (AHC). EPWMxB is inverted
        // Dead-Band Output Mode Control
    EPwm1Regs.DBCTL.bit.OUT_MODE = 0b11;    // 0b11: DBM is fully enabled (i.e. both RED and FED active)


    /*
     * DBxED = xED / T_TBCLK        where x = {R --> Rising, F --> Falling}
     *      ==> DBxED = (100 ns) / (1 / 100 MHz)
     *      ==> DBxED = 10
     */

    // Dead-Band Edge Delay
        // Dead-Band Falling Edge Delay
    EPwm1Regs.DBFED.bit.DBFED = 10; // 0b1010 in binary
        // Dead-Band Rising Edge Delay
    EPwm1Regs.DBRED.bit.DBRED = 10; // 0b1010 in binary

    /////////////////////////////////////////////
    /// EVENT TRIGGER CONFIGURATION
    /////////////////////////////////////////////

    // TBPRD --> max value we count up to

    // Event Trigger Selection Register
        // Start of Conversion A Enable --> Enable the ADC Start of Conversion A (EPWMxSOCA) Pulse
    EPwm1Regs.ETSEL.bit.SOCAEN = 0b1;       // 1: Enable EPWMxSOCA pulse.
        // Start of Conversion A Select --> EPWMxSOCA Selection Options
    EPwm1Regs.ETSEL.bit.SOCASEL = 0b011;    // 011: Enable event time-base counter equal to zero or period (TBCTR = 0x00 or TBCTR = TBPRD). This mode is useful in updown count mode.

    // Event Trigger Pre-Scale Register
        // Start of Conversion A Period --> ePWM ADC Start-of-Conversion A Event (EPWMxSOCA) Period Select
    EPwm1Regs.ETPS.bit.SOCAPRD = 0b01;  // 01: Generate the EPWMxSOCA pulse on the first event: ETPS[SOCACNT] = 0,1
         // Interrupt Period --> ePWM Interrupt (EPWMx_INT) Period Select
    EPwm1Regs.ETPS.bit.INTPRD = 0b01;   // 01: Generate an interrupt on the first event INTCNT = 01 (first event)


    /////////////////////////////////////////////
    /// GPIO CONFIGURATION
    /////////////////////////////////////////////

    // GPIO A Peripheral Mux (GPIO0 to GPIO15)
        //
    GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1;
        //
    GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 1;

    // GPIO A Pull-Up Disable (GPIO0 to GPIO31)
        // Disable Pull-up Resistor for GPIO A, pin 0
    GpioCtrlRegs.GPAPUD.bit.GPIO0 = 0b1;
        // Disable Pull-up Resistor for GPIO A, pin 1
    GpioCtrlRegs.GPAPUD.bit.GPIO1 = 0b1;

    // Emulation Disable --> disable write access to protected space (i.e., protected registers)
    EDIS;

}

//
// initADCSOC - Function to configure ADCA's SOC0 to be triggered by ePWM1.
//
void initADCSOC(void)
{
    //
    // Select the channels to convert and the end of conversion flag
    //
    EALLOW;

//    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 1;     // SOC0 will convert pin A1
//                                           // 0:A0  1:A1  2:A2  3:A3
//                                           // 4:A4   5:A5   6:A6   7:A7
//                                           // 8:A8   9:A9   A:A10  B:A11
//                                           // C:A12  D:A13  E:A14  F:A15
//    AdcaRegs.ADCSOC0CTL.bit.ACQPS = 9;     // Sample window is 10 SYSCLK cycles
//    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 5;   // Trigger on ePWM1 SOCA

    /////////////////////////////////////////////
    // ADC SOC CONFIGURATION
    /////////////////////////////////////////////

    // Channel Select --> Specify SOC module for each ADC channel in use
        // SOC 0 will convert ADCINA3 (i.e., ADC A, channel 3)
    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 3; // 3h ADCIN3
        // SOC 1 will convert ADCINA5 (i.e., ADC A, channel 5)
    AdcaRegs.ADCSOC1CTL.bit.CHSEL = 5; // 5h ADCIN5
        // SOC 2 will convert ADCINA6 (i.e., ADC A, channel 6)
    AdcaRegs.ADCSOC2CTL.bit.CHSEL = 6; // 6h ADCIN6

    // Acquisition Pre-scale --> Sample and hold window of SOC modules
    AdcaRegs.ADCSOC0CTL.bit.ACQPS = 9; // 009h Sample window is 9 system clock cycles wide
    AdcaRegs.ADCSOC1CTL.bit.ACQPS = 9; // 009h Sample window is 9 system clock cycles wide
    AdcaRegs.ADCSOC2CTL.bit.ACQPS = 9; // 009h Sample window is 9 system clock cycles wide

    // Trigger Select --> Determines which trigger will initiate a conversion
    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 5; // 05h: ADCTRIG5 - ePWM1, ADCSOCA
    AdcaRegs.ADCSOC1CTL.bit.TRIGSEL = 5; // 05h: ADCTRIG5 - ePWM1, ADCSOCA
    AdcaRegs.ADCSOC2CTL.bit.TRIGSEL = 5; // 05h: ADCTRIG5 - ePWM1, ADCSOCA

    /////////////////////////////////////////////
    // INTERRUPT CONFIGURATION
    /////////////////////////////////////////////

    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 0; // End of SOC0 will set INT1 flag
    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;   // Enable INT1 flag
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; // Make sure INT1 flag is cleared

    EDIS;
}


void initDAC(void)
{
    EALLOW;

    // DAC Control
        // DAC reference select --> Selects which voltage references are used by the DAC
    DacaRegs.DACCTL.bit.DACREFSEL = 0b1; // 1 ADC VREFHI/VSSA are the reference voltages
        // Determines when the DACVALA register is updated with the value from DACVALS.
    DacaRegs.DACCTL.bit.LOADMODE = 0b0;  // 0 Load on next SYSCLK
        // DAC gain mode select. Selects the gain mode for the buffered output
    DacaRegs.DACCTL.bit.MODE = 0b1; // 1 Gain is 2

    // DAC Output Enable Register
    DacaRegs.DACOUTEN.bit.DACOUTEN = 0b1; // 1 DAC output is enabled

    // DAC Value Register - Shadow
    DacaRegs.DACVALS.all = 0; // Shadow output code to be loaded into DACVALA

    EDIS;
}

//
// adcA1ISR - ADC A Interrupt 1 ISR
//
__interrupt void adcA1ISR(void)
{
    //
    // Add the latest result to the buffer
    // ADCRESULT0 is the result register of SOC0
    adcAResults[index++] = AdcaResultRegs.ADCRESULT0;

    // CUSTOM ASSIGNMENT
    temp_ADC = AdcaResultRegs.ADCRESULT0;
    DacaRegs.DACVALS.all = temp_ADC;

    //
    // Set the bufferFull flag if the buffer is full
    //
    if(RESULTS_BUFFER_SIZE <= index)
    {
        index = 0;
        bufferFull = 1;
    }

    // CUSTOM CODE

    Ts = 1/fsamp;

    dt = dt + Ts;
    if (0.5 * Wn*dt >= 2*pi){
        dt=0;
    }
    duty = 0.5*(1 + m*cos(0.5*Wn*dt));

    EPwm1Regs.CMPA.bit.CMPA = duty * N_r;

    // CUSTOM CODE

    //
    // Clear the interrupt flag
    //
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;

    //
    // Check if overflow has occurred
    //
    if(1 == AdcaRegs.ADCINTOVF.bit.ADCINT1)
    {
        AdcaRegs.ADCINTOVFCLR.bit.ADCINT1 = 1; //clear INT1 overflow flag
        AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear INT1 flag
    }

    //
    // Acknowledge the interrupt
    //
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

//
// End of File
//
