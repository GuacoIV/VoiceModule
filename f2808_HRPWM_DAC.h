/**********************************************************************
* File: f2808_HRPWM_DAC.h
* Device: TMS320F2808
* Author: David M. Alter, Texas Instruments Inc.
* Description: Include file for f2808_HRPWM_DAC project.  Include this
*   file in all C-source files.
* History:
*   10/26/04 - original (D. Alter)
**********************************************************************/

#ifndef f2808_HRPWM_DAC_H
#define f2808_HRPWM_DAC_H


//*** USER SELECTABLE OPTIONS *****************************************

//--------------------------------------------
// OPTION #1: PWM frequency.
//
//   DAC1_period = [(100 MHz)/(Desired PWM frequency)] - 1
//
#define DAC1_period		19				// Period for 5 MHz asymmetric PWM

//--------------------------------------------
// OPTION #2: PWM pin.
//
//   DAC1_pin = 1  for ePWM1A pin
//            = 2  for ePWM2A pin
//            = 3  for ePWM3A pin
//            = 4  for ePWM4A pin (not available on F2801)
//
#define DAC1_pin		1				// Use ePWM1A pin

//--------------------------------------------
// OPTION #3: HRPWM calibration module.
//
//   HRPWM_CAL_CH = 1  for ePWM1 module
//                = 2  for ePWM2 module
//                = 3  for ePWM3 module
//                = 4  for ePWM4 module (not available on F2801)
//
#define HRPWM_CAL_CH	3				// Use ePWM3 as HRPWM calibrator

//---------------------------------------------------------------------------
// OPTION #4: DAC output waveform.  Set OUTPUT to select a waveform:
//
#define SQUARE 0						// Square wave
#define SINE 1							// Sine wave
#define CONSTANT 2						// Constant

#define OUTPUT SINE						// Set to SQUARE, SINE, or CONSTANT

//---------------------------------------------------------------------------
// Option #5: DAC output amplitude and offset.
//
// User must ensure that (AMPLITUDE + OFFSET) <= 3.3.  Note that for the CONSTANT
// output waveform, OFFSET has no effect.  The constant level is determined solely
// by the AMPLITUDE.
//
#define AMPLITUDE 2.0					// Voltage.  Set between 0 and 3.3 decimal, inclusive
#define OFFSET    0.5					// Voltage.  Set between 0 and 3.3 decimal, inclusive

//---------------------------------------------------------------------------
// Option #6: Timer0 period.
//
// These constants can be modified to change the update rate of the PWM/DAC
// output.  This will change the frequency of the square or sine wave output.
// For demo purposes, there is really no need to modify these.
//
#define Timer0_Period	399L					// 399 = 250 kHz w/ 100MHz SYSCLKOUT
#define Timer0_Prescale 0						// 0 => precaler is /1



//*** END USER SELECTABLE OPTIONS *************************************



//---------------------------------------------------------------------------
// Other constants (The user should not change these)
//
#define Q15_SCALE	32768				// Scale for mapping fractions to Q15
#define PWM_VOLTAGE	3.3					// PWM voltage


//---------------------------------------------------------------------------
// Include Standard C Language Header Files
//
#include <string.h>

//---------------------------------------------------------------------------
// Include any other Header Files
//
//#include "DSP280x_DefaultIsr.h"			// ISR definitions (for non-BIOS projects only)
#include "Dac.h"						// ePWM DAC driver
#include "SFO.h"						// HRPWM scale factor optimization

//---------------------------------------------------------------------------
// Function Prototypes
//
extern void    DelayUs(Uint16);
extern Uint16  DisableInts(void);
extern void    InitSysCtrl(void);
extern void    InitPieCtrl(void);
extern void    InitGpio(void);
extern void    InitFlash(void);
extern void    InitTimer0(void);
extern void    RestoreInts(Uint16);
extern void    SetDBGIER(Uint16);

//---------------------------------------------------------------------------
// Global symbols defined in the linker command file
//
extern Uint16 secureRamFuncs_loadstart;
extern Uint16 secureRamFuncs_loadend;
extern Uint16 secureRamFuncs_runstart;

//---------------------------------------------------------------------------
// Global symbols defined in source files
//
extern const struct PIE_VECT_TABLE PieVectTableInit;
extern const unsigned int sine256Q15[];

//---------------------------------------------------------------------------
#endif  // end of f2808_HRPWM_DAC_H definition

/*** end of file *****************************************************/
