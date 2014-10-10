/**********************************************************************
* File: DAC.h
* Device: TMS320F2808
* Author: David M. Alter, Texas Instruments Inc.
* Description: Include file for ePWM DAC driver code.
* History:
*   07/25/05 - original (D. Alter)
**********************************************************************/

#ifndef DAC_H
#define DAC_H


//---------------------------------------------------------------------------
// Function Prototypes
//
extern void   InitDac(Uint16, Uint16);
extern void   WriteDac(Uint16, int16);


//---------------------------------------------------------------------------
// Global Variable References
//
extern int16 MEP_ScaleFactor[5];			// HRPWM micro edge positioning scale factor


//---------------------------------------------------------------------------
#endif  // end of DAC_H definition

/*** end of file *****************************************************/
