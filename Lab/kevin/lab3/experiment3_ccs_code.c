// Experiment #3
// EE 458_533
// Kevin Egedy, David Babin

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
// $TI Release: F28004x Support Library v1.08.00.00 $
// $Release Date: Mon Dec 23 17:24:05 IST 2019 $
// $Copyright:
// Copyright (C) 2019 Texas Instruments Incorporated - http://www.ti.com/
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


//////////////////////////////////////////////////////////////////////////////////
/// INCLUDED FILES
//////////////////////////////////////////////////////////////////////////////////

#include "F28x_Project.h"
#include "math.h"

//////////////////////////////////////////////////////////////////////////////////
/// DEFINES
//////////////////////////////////////////////////////////////////////////////////

// System Constants
#define pi 3.141592653589f
//#define T_samp 0.0001f //0.00005f
#define al 2.094395102393195f
#define N 5000 // N_r
#define f_sw 10000.0f // switching frequency

// Gain crossover frequency of dq-axis controller [rad/s]
#define w_cg    3.5714e3f
// Gain crossover frequency of speed controller [rad/s]
#define w_cg_w  357.14f // w_cg_w = w_cg / 10;
// Combined stator & mutual inductance [H]
#define L    380.6e-6f // 429.7e-6f
// Effective stator resistance
#define R_L  292.5e-3f
// Net moment of inertia of the bike and the rider
#define J   0.1127f
// Cumulative friction coefficient of the shaft and road
#define B   0.001f

// Stator DQ-axis Current PI Controller Parameters
float Kp = w_cg * L;   // 1.5346;
float Ki = w_cg * R_L; // 1.0446e3;

//// Gain crossover frequency of speed controller [rad/s]
//float w_cg_w = w_cg / 10;

// Speed Controller Parameters
float Kp_w = w_cg_w * J; // 40.25
float Ki_w = w_cg_w * B; // 0.3571

// Stator Current D-axis Controller Saturation Limits
    // Integrator Saturation Limits
#define Y_sd_int_min -50
#define Y_sd_int_max  50
    // Net Saturation Limits
#define Y_sd_min 0      // -50
#define Y_sd_max 50

// Stator Current Q-axis Controller Saturation Limits
    // Integrator Saturation Limits
#define Y_sq_int_min -50
#define Y_sq_int_max  50
    // Net Saturation Limits
#define Y_sq_min 0
#define Y_sq_max 50

// Speed Controller Saturation Limits (all limits in mph)
    // Integrator Saturation Limits
#define Y_w_int_min -15 // 487.383 electrical rad/s = 20.3076 mechanical rad/s = 15 mph
#define Y_w_int_max  15
    // Net Saturation Limits
#define Y_w_min 0
#define Y_w_max 15 // 487.383 electrical rad/s = 20.3076 mechanical rad/s = 15 mph

// Peak flux linkage
#define lambda_m 0.045f

//////////////////////////////////////////////////////////////////////////////////
/// GLOBALS
//////////////////////////////////////////////////////////////////////////////////

// Sampling
float f_samp = 2 * (float)f_sw;
float T_samp = 1 / ( 2 * (float)f_sw );

// Stator angular frequency
float w_s = 2*pi*5;             // Set stator voltage waveform to 5Hz frequency

float wt = 0;


// Duty Ratio
    // Fixed Duty Ratio Value
float duty = 0.5;
    // Modulation Index for Sinusoidal Duty Ratio
float mod_index = 0.5; // Modulation index in open loop is our control parameter. Tune this to vary stator voltage magnitude

float angle = 0;

// Stator Variables
    // ABC Stator Currents
float I_sa;
float I_sb;
float I_sc;
    // DQ Stator Currents
float I_sd;
float I_sq;
    // ABC Stator Voltages
float V_sa;
float V_sb;
float V_sc;
    // DQ Stator Voltages
float V_sd;
float V_sq;
    // DQ Back EMF
float E_bd; // D-axis
float E_bq; // Q-axis

// angular electrical frequency of rotor [rad/s]
float Vout = 0;
float V_in = 50;
float th_act = 0;
float Imax = 100; // make sure this is right

// DQ-axis Reference Currents
float I_sd_ref = 0;
float I_sq_ref = 1;

// Control System Variables
    // D-axis Control System Variables
float I_sd_error        = 0;
float I_sd_error_prev   = 0;
float Y_sd_prop;
float Y_sd_int;
float Y_sd_int_prev     = 0;
float Y_sd;
    // Q-axis Control System Variables
float I_sq_error        = 0;
float I_sq_error_prev   = 0;
float Y_sq_prop;
float Y_sq_int;
float Y_sq_int_prev     = 0;
float Y_sq;

float w_mph_ref = 15; // 487.383 electrical rad/s = 20.3076 mechanical rad/s = 15 mph
float w_e;
float w_m;
float w_mph;
float w_e_error        = 0;
float w_e_error_prev   = 0;
float Y_w_prop;
float Y_w_int;
float Y_w_int_prev     = 0;
float Y_w;

float T_e = 0;
float poles = 2 * 24;

// Functionality Enable
bool enable_feed_forward = true;
bool enable_anti_windup = true;

//////////////////////////////////////////////////////////////////////////////////
/// FUNCTION PROTOTYPES
//////////////////////////////////////////////////////////////////////////////////

void initADC(void);
void initEPWM(void);
void initADCSOC(void);
void setup1GPIO(void);
__interrupt void adcA1ISR(void);


//////////////////////////////////////////////////////////////////////////////////
/// MAIN
//////////////////////////////////////////////////////////////////////////////////

void main(void) {

    /// Initialize device clock and peripherals
    InitSysCtrl();

    /// Initialize GPIO
    InitGpio();

    /// Disable CPU interrupts
    //DINT;

    /// Initialize the PIE control registers to their default state. The default state is all PIE interrupts disabled and flags are cleared.
    InitPieCtrl();

    /// Disable CPU interrupts and clear all CPU interrupt flags:
    IER = 0x0000;
    IFR = 0x0000;

    /// Initialize the PIE vector table with pointers to the shell Interrupt Service Routines (ISR).
    InitPieVectTable();

    /// Map ISR functions
    EALLOW;
        PieVectTable.ADCA1_INT = &adcA1ISR;     // Function for ADCA interrupt
    EDIS;

    /// Configure the ADC and power it up
    initADC();

    /// Configure the ePWM
    initEPWM();

    /// Setup the ADC for ePWM triggered conversions on channel 1
    initADCSOC();

    /// Enable global Interrupts and higher priority real-time debug
    IER |= M_INT1;  // Enable group 1 interrupts
    EINT;           // Enable Global interrupt INTM
    ERTM;           // Enable Global realtime interrupt DBGM

    /// Enable PIE interrupt
    PieCtrlRegs.PIEIER1.bit.INTx1 = 1;

    /// Sync ePWM
    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;

    // Take conversions indefinitely in loop
    setup1GPIO();
    while(1) { }

}

//
// initADC - Function to configure and power up ADCA.
//
void initADC(void)
{
    // Setup VREF as internal
    //
    SetVREF(ADC_ADCA, ADC_INTERNAL, ADC_VREF3P3);
    EALLOW;
    // Set ADCCLK divider to /4
    //
    AdcaRegs.ADCCTL2.bit.PRESCALE = 6;
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
    // Use EPWM2,3,4 for motor control, set them the same as EPWM1 (done or you)
    EALLOW;
    EPwm1Regs.ETSEL.bit.SOCAEN = 1;     // Enable SOC on A group
    EPwm1Regs.ETSEL.bit.SOCASEL = 1;    // Select SOC when TBCTR = PRD or ZERO
    EPwm1Regs.ETPS.bit.SOCAPRD = 1;     // Generate pulse on 1st event
    EPwm1Regs.ETPS.bit.INTPRD = ET_1ST;              // generate interrupt once for each PWM cycle
    EPwm1Regs.TBPRD = N; //period-1;             // PWM frequency = 1 /period
    EPwm1Regs.CMPA.bit.CMPA = duty*N;   // set duty 50% initially
    EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;
    EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;
    EPwm1Regs.TBCTL.bit.CLKDIV = TB_DIV1;
    EPwm1Regs.AQCTLA.bit.CAU = AQ_CLEAR;        //EPWM1A goes low whenCTR=CMPA register when CTR decrementing
    EPwm1Regs.AQCTLA.bit.CAD = AQ_SET;          //EPWM1A goes hi whenCTR=CMPA register when CTR incrementing

    //Deadband settings
    EPwm1Regs.DBCTL.bit.IN_MODE =0x0; // ePWM-A for both RED and FED
    EPwm1Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE; // enable Dead-bandmodule
    EPwm1Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC; // active Hi complementary
    EPwm1Regs.DBFED.all = 10; //Falling edge delay - based on sysclk (ePWMtable 2-14)
    EPwm1Regs.DBRED.all = 10; //Rising edge delay - based on sysclk (ePWMtable

    // Motor PWM
    EPwm2Regs.TBPRD = N; //period-1;             // PWM frequency = 1 /period
    EPwm2Regs.CMPA.bit.CMPA = duty*N;   // set duty 50% initially
    EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;
    EPwm2Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;
    EPwm2Regs.TBCTL.bit.CLKDIV = TB_DIV1;
    EPwm2Regs.AQCTLA.bit.CAU = AQ_CLEAR;        //EPWM1A goes low whenCTR=CMPA register when CTR decrementing
    EPwm2Regs.AQCTLA.bit.CAD = AQ_SET;          //EPWM1A goes hi when CTR=CMPA register when CTR incrementing
    EPwm2Regs.AQCTLA.bit.ZRO = 0;
    EPwm2Regs.AQCTLA.bit.PRD = 0;

    //Deadband settings
    EPwm2Regs.DBCTL.bit.IN_MODE =0x0; // ePWM-A for both RED and FED
    EPwm2Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE; // enable Dead-band module
    EPwm2Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC; // active Hi complementary
    EPwm2Regs.DBFED.all = 10; //Falling edge delay - based on sysclk (ePWM table 2-14)
    EPwm2Regs.DBRED.all = 10; //Rising edge delay - based on sysclk (ePWMtable

    EPwm3Regs.TBPRD = N; //period-1;             // PWM frequency = 1 / period
    EPwm3Regs.CMPA.bit.CMPA = duty*N;   // set duty 50% initially

    EPwm3Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;

    EPwm3Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;
    EPwm3Regs.TBCTL.bit.CLKDIV = TB_DIV1;

    EPwm3Regs.AQCTLA.bit.CAU = AQ_CLEAR;        //EPWM1A goes low whenCTR=CMPA register when CTR decrementing
    EPwm3Regs.AQCTLA.bit.CAD = AQ_SET;          //EPWM1A goes hi whenCTR=CMPA register when CTR incrementing

    //Deadband settings
    EPwm3Regs.DBCTL.bit.IN_MODE =0x0; // ePWM-A for both RED and FED
    EPwm3Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE; // enable Dead-bandmodule
    EPwm3Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC; // active Hi complementary
    EPwm3Regs.DBFED.all = 10; //Falling edge delay - based on sysclk (ePWMtable 2-14)
    EPwm3Regs.DBRED.all = 10; //Rising edge delay - based on sysclk (ePWMtable)

    EPwm4Regs.TBPRD = N; //period-1;             // PWM frequency = 1 /period
    EPwm4Regs.CMPA.bit.CMPA = duty*N;   // set duty 50% initially

    EPwm4Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;

    EPwm4Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;
    EPwm4Regs.TBCTL.bit.CLKDIV = TB_DIV1;

    EPwm4Regs.AQCTLA.bit.CAU = AQ_CLEAR;        //EPWM1A goes low whenCTR=CMPA register when CTR decrementing
    EPwm4Regs.AQCTLA.bit.CAD = AQ_SET;          //EPWM1A goes hi when CTR=CMPA register when CTR incrementing

    //Deadband settings
    EPwm4Regs.DBCTL.bit.IN_MODE = 0x0; // ePWM-A for both RED and FED
    EPwm4Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE; // enable Dead-band module
    EPwm4Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC; // active Hi complementary
    EPwm4Regs.DBFED.all = 10; //Falling edge delay - based on sysclk (ePWMtable 2-14)
    EPwm4Regs.DBRED.all = 10; //Rising edge delay - based on sysclk (ePWMtable)
    EDIS;
}

//
// initADCSOC - Function to configure ADCA's SOC0 to be triggered byePWM1.
//
void initADCSOC(void)
{
    //
    // Select the channels to convert and the end of conversion flag
    //
    //Use 5 ADC Channels for Motor Control. A3,A5,A6 for Iabc, and A9,A1for theta, we
    //Done for you
    EALLOW;
    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 3;
    AdcaRegs.ADCSOC1CTL.bit.CHSEL = 5;
    AdcaRegs.ADCSOC2CTL.bit.CHSEL = 6;
    AdcaRegs.ADCSOC3CTL.bit.CHSEL = 9;// SOC0 will convert pin A1
        // 0:A0  1:A1  2:A2  3:A3
        // 4:A4   5:A5   6:A6   7:A7
        // 8:A8   9:A9   A:A10  B:A11
        // C:A12  D:A13  E:A14  F:A15
    AdcaRegs.ADCSOC4CTL.bit.CHSEL = 1;// SOC0 will convert pin A1
    // 0:A0  1:A1  2:A2   3:A3
    // 4:A4   5:A5   6:A6  7:A7
    // 8:A8   9:A9   A:A10  B:A11
    // C:A12  D:A13  E:A14 F:A15
    AdcaRegs.ADCSOC0CTL.bit.ACQPS = 9;     // Sample window is 10 SYSCLKcycles
    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 5;   // Trigger on ePWM1 SOCA
    AdcaRegs.ADCSOC1CTL.bit.ACQPS = 9;     // Sample window is 10 SYSCLKcycles
    AdcaRegs.ADCSOC1CTL.bit.TRIGSEL = 5;
    AdcaRegs.ADCSOC2CTL.bit.ACQPS = 9;     // Sample window is 10 SYSCLKcycles
    AdcaRegs.ADCSOC2CTL.bit.TRIGSEL = 5;
    AdcaRegs.ADCSOC3CTL.bit.ACQPS = 9;     // Sample window is 10 SYSCLKcycles
    AdcaRegs.ADCSOC3CTL.bit.TRIGSEL = 5;
    AdcaRegs.ADCSOC4CTL.bit.ACQPS = 9;
    AdcaRegs.ADCSOC4CTL.bit.TRIGSEL = 5;
    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 0; // End of SOC0 will set INT1flag
    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;   // Enable INT1 flag
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; // Make sure INT1 flag iscleared
    EDIS;
}

//
// adcA1ISR - ADC A Interrupt 1 ISR
//
interrupt void adcA1ISR(void) {

    // Clear the interrupt flag
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;


    //////////////////////////////////////////////////////////////////////////////////
    /// OPEN LOOP CONTROL
    //////////////////////////////////////////////////////////////////////////////////

//    float f_div = (2*pi*f_samp) / w_s; // f_div = f_samp/10
//
//    if (wt > 2*pi) wt = 0;
//
//    wt = wt + (2*pi/f_div);
//
//    //EPwm1Regs.CMPA.bit.CMPA = duty*N;
//    EPwm2Regs.CMPA.bit.CMPA = ( 0.5 + 0.5*mod_index*sin(wt)      ) * N;
//    EPwm3Regs.CMPA.bit.CMPA = ( 0.5 + 0.5*mod_index*sin(wt - al) ) * N;
//    EPwm4Regs.CMPA.bit.CMPA = ( 0.5 + 0.5*mod_index*sin(wt + al) ) * N;


    //////////////////////////////////////////////////////////////////////////////////
    /// CLOSED LOOP CONTROL
    //////////////////////////////////////////////////////////////////////////////////

    /// Sample Data
    //////////////////////////////////////////////////////////////////////////////////

    // Read in data from ADC pins
    I_sa =  (float)((AdcaResultRegs.ADCRESULT0*3.3/4096) - 3.3/2)*(2*Imax/3.3);
    I_sb =  (float)((AdcaResultRegs.ADCRESULT1*3.3/4096) - 3.3/2)*(2*Imax/3.3);
    I_sc =  (float)((AdcaResultRegs.ADCRESULT2*3.3/4096) - 3.3/2)*(2*Imax/3.3);

    angle = (float)AdcaResultRegs.ADCRESULT3*(3.3/4096)*7/3.3;
    w_e =   (float)AdcaResultRegs.ADCRESULT4*(3.3/4096)*1000/3.3;

    // Convert from electrical rad/s to mechanical mph
    w_m = w_e * 2 / poles;
    w_mph = w_m * 2.23694 * (13.0 / 39.37);

    // Express w_e in mph
    w_e = w_mph;

    /// Stator Currents Domain Transformation: ABC --> DQ
    //////////////////////////////////////////////////////////////////////////////////

    I_sd = 0.66667  * ( I_sa*cos(angle) + I_sb*cos(angle - al) + I_sc*cos(angle + al) );
    I_sq = -0.66667 * ( I_sa*sin(angle) + I_sb*sin(angle - al) + I_sc*sin(angle + al) );

    /// Speed Controller
    //////////////////////////////////////////////////////////////////////////////////

    // D-axis Current Error
    w_e_error = w_mph_ref - w_e;

    // PI Controller
        // Error propagated through the proportional controller
    Y_w_prop = w_e_error * Kp_w;
        // Error propagated through the integral controller
    Y_w_int = 0.5 * T_samp * (w_e_error + w_e_error_prev) * Ki_w + Y_w_int_prev;

    // Saturate Integral Controller Output
    if (Y_w_int < Y_w_int_min) Y_w_int = Y_w_int_min;
    else if (Y_w_int > Y_w_int_max) Y_w_int = Y_w_int_max;
    else if (Y_w_int == Y_w_int_max && enable_anti_windup) Y_w_int = 0; // anti-windup

    // Update stator d-axis values
        // Update the integrator
    Y_w_int_prev = Y_w_int;
        // update the current error
    w_e_error_prev = w_e_error;

    // D-axis stator current PI Controller Output
    Y_w = Y_w_prop + Y_w_int;

    // Saturate PI Controller Output
    if (Y_w < Y_w_min) Y_w = Y_w_min;
    else if ( Y_w > Y_w_max ) Y_w = Y_w_max;

    T_e = Y_w;

    I_sq_ref = 4 / (3 * lambda_m * poles) * T_e;

    /// D-axis Stator Current Controller
    //////////////////////////////////////////////////////////////////////////////////

    // D-axis Current Error
    I_sd_error = I_sd_ref - I_sd;

    // PI Controller
        // Error propagated through the proportional controller
    Y_sd_prop = I_sd_error * Kp;
        // Error propagated through the integral controller
    Y_sd_int = 0.5 * T_samp * (I_sd_error + I_sd_error_prev) * Ki + Y_sd_int_prev;

    // Saturate Integral Controller Output
    if (Y_sd_int < Y_sd_int_min) Y_sd_int = Y_sd_int_min;
    else if (Y_sd_int > Y_sd_int_max) Y_sd_int = Y_sd_int_max;
    else if (Y_sd_int == Y_sd_int_max && enable_anti_windup) Y_sd_int = 0; // anti-windup

    // Update stator d-axis values
        // Update the integrator
    Y_sd_int_prev = Y_sd_int;
        // update the current error
    I_sd_error_prev = I_sd_error;

    // D-axis stator current PI Controller Output
    Y_sd = Y_sd_prop + Y_sd_int;

    // Saturate PI Controller Output
    if (Y_sd < Y_sd_min) Y_sd = Y_sd_min;
    else if ( Y_sd > Y_sd_max ) Y_sd = Y_sd_max;

    // D-axis Back EMF
    E_bd = 0;

    // D-axis Voltage
    if (enable_feed_forward) V_sd = Y_sd - (L*w_e*I_sq) + E_bd;
    else V_sd = Y_sd;

    /// Q-axis Stator Current Controller
    //////////////////////////////////////////////////////////////////////////////////

    // Q-axis Current Error
    I_sq_error = I_sq_ref - I_sq;

    // PI Controller
        // Error propagated through the proportional controller
    Y_sq_prop = I_sq_error * Kp;
        // Error propagated through the integral controller
    Y_sq_int = 0.5 * T_samp * (I_sq_error + I_sq_error_prev) * Ki + Y_sq_int_prev;

    // Saturate Integral Controller Output
    if (Y_sq_int < Y_sq_int_min) Y_sq_int = Y_sq_int_min;
    else if (Y_sq_int > Y_sq_int_max) Y_sq_int = Y_sq_int_max;
    else if (Y_sq_int == Y_sq_int_max && enable_anti_windup) Y_sq_int = 0; // anti-windup

    // Update stator q-axis values
        // Update the integrator
    Y_sq_int_prev = Y_sq_int;
        // update the current error
    I_sq_error_prev = I_sq_error;

    // Q-axis stator current PI Controller Output
    Y_sq = Y_sq_prop + Y_sq_int;

    // Saturate PI Controller Output
    if (Y_sq < Y_sq_min) Y_sq = Y_sq_min;
    else if (Y_sq > Y_sq_max) Y_sq = Y_sq_max;

    // Q-axis Back EMF
    E_bq = lambda_m * w_e;

    // Q-axis Voltage
    if (enable_feed_forward) V_sq = Y_sq + E_bq + (L*w_e*I_sd);
    else V_sq = Y_sq;

    /// Stator Voltage Domain Transformation: DQ --> ABC
    //////////////////////////////////////////////////////////////////////////////////

    // DQ to ABC
    V_sa = ( V_sd*cos(angle)      - V_sq*sin(angle)      ) / V_in;
    V_sb = ( V_sd*cos(angle - al) - V_sq*sin(angle - al) ) / V_in;
    V_sc = ( V_sd*cos(angle + al) - V_sq*sin(angle + al) ) / V_in;

    /// Load Duty Ratio
    //////////////////////////////////////////////////////////////////////////////////

    EPwm2Regs.CMPA.bit.CMPA = (0.5 + 0.5*V_sa) * N;
    EPwm3Regs.CMPA.bit.CMPA = (0.5 + 0.5*V_sb) * N;
    EPwm4Regs.CMPA.bit.CMPA = (0.5 + 0.5*V_sc) * N;

    //////////////////////////////////////////////////////////////////////////////////
    /// ADDITIONAL CONFIGURATION
    //////////////////////////////////////////////////////////////////////////////////

    /// update all variables here
    if(1 == AdcaRegs.ADCINTOVF.bit.ADCINT1)
    {
        AdcaRegs.ADCINTOVFCLR.bit.ADCINT1 = 1; //clear INT1 overflow flag
        AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear INT1 flag
    }

    /// Acknowledge the interrupt
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

void setup1GPIO(void)
{
    //
    // Example 1: Basic Pinout.
    // This basic pinout includes:
    // PWM1-3, ECAP1, ECAP2, TZ1-TZ4, SPI-A, EQEP1, SCI-A, CAN-A, I2C
    // and a number of I/O pins
    // These can be combined into single statements for improved
    // code efficiency.
    //
    //
    // Enable PWM1-3 on GPIO0-GPIO5
    //
    // Set up GPIOs here (done for you)
    EALLOW;
    GpioCtrlRegs.GPAPUD.bit.GPIO0 = 1;    // Disable pull-up on GPIO0

    GpioCtrlRegs.GPAPUD.bit.GPIO1 = 1;    // Disable pull-up on GPIO1

    GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1;   // Configure GPIO0 as EPWM1A
    GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 1;   // Configure GPIO1 as EPWM1B
    GpioCtrlRegs.GPAPUD.bit.GPIO2 = 1;    // Disable pull-up on GPIO0
                                            //(EPWM2A)
    GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 1;   // Configure GPIO0 as EPWM2A
    GpioCtrlRegs.GPAPUD.bit.GPIO3 = 1;    // Disable pull-up on GPIO0

    GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 1;   // Configure GPIO0 as EPWM2A
    GpioCtrlRegs.GPAPUD.bit.GPIO4 = 1;    // Disable pull-up on GPIO0

    GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 1;   // Configure GPIO0 as EPWM3A
    GpioCtrlRegs.GPAPUD.bit.GPIO5 = 1;    // Disable pull-up on GPIO0

    GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 1;   // Configure GPIO0 as EPWM3A
    GpioCtrlRegs.GPAPUD.bit.GPIO6 = 1;    // Disable pull-up on GPIO0

    GpioCtrlRegs.GPAMUX1.bit.GPIO6 = 1;   // Configure GPIO0 as EPWM4A
    GpioCtrlRegs.GPAPUD.bit.GPIO7 = 1;    // Disable pull-up on GPIO0

    GpioCtrlRegs.GPAMUX1.bit.GPIO7 = 1;   // Configure GPIO0 as EPWM4B
    GpioCtrlRegs.GPBMUX1.bit.GPIO33= 0;  // GPIO6 = GPIO6
    GpioCtrlRegs.GPBDIR.bit.GPIO33 = 0;   // GPIO6 = input
    GpioCtrlRegs.GPBMUX1.bit.GPIO34= 0;  // GPIO6 = GPIO6
    GpioCtrlRegs.GPBDIR.bit.GPIO34 = 0;   // GPIO6 = input.
    GpioCtrlRegs.GPBMUX1.bit.GPIO35= 0;  // GPIO6 = GPIO6
    GpioCtrlRegs.GPBDIR.bit.GPIO35 = 0;   // GPIO6 = input
    EDIS;
}
//
// End of File
//
