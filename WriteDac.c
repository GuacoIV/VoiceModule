/**********************************************************************
* Function: WriteDac()                                                *
* Description: Sets the output on a PWM/DAC channel.                  *
* Device: TMS320F2808, TMS320F2806, TMS320F2801                       *
* Author: David M. Alter, Texas Instruments Inc.                      *
* Include files: DSP280x_Device.h                                     *
* Function Prototype: void WriteDac(Uint16, int16);                   *
* Useage: WriteDac(channel, duty_frac);                               *
* Input Parameters: channel = ePWM module number to be initialized    *
*                   duty_frac = duty cycle percentage (Q15 format)    *
* Return Value: none                                                  *
* Notes:                                                              *
* 1) On F2808 and F2806 devices, 'channel' can be 1, 2, 3, or 4.      *
*    On F2801 and UCD9501 devices, 'channel' can be 1, 2, or 3.       *
*    There is no error checking in the function if an invalid channel *
*    is specified, and may cause a runtime software crash.            *
* 2) 'duty_frac' is in Q15 fractional format.  Only positive values   *
*    make sense, in the range 0x0000 (i.e. 0%) to 0x7FFF (~100%).     *
*    Negative values will produce erroneous results on the PWM pin.   *
* 3) This function has been written assuming PWM goes active on CAU   *
*    match, and goes inactive on ZRO match.  Because of this, the     *
*    working duty fraction is computed as 100% minus the passed duty  *
*    fraction parameter.  Although other configurations are possible  *
*    that directly work with the passed duty fraction, the one used   *
*    here was seen to have the best performance in terms of offset    *
*    errors.                                                          *
* 4) This function uses the DSP280x Peripheral Header files, TI       *
*    literature #SPRC191.                                             *
* History:                                                            *
*   02/07/06 - original (D. Alter)                                    *
**********************************************************************/

#include "DSP28x_Project.h"  				// Peripheral address definitions
#include "f2808_HRPWM_DAC.h"				// Include file specific to this project

int16 MEP_ScaleFactor[5] = { 0, 0, 0, 0, 0 }; // HRPWM Scale Factor values for ePWM1-4

void WriteDac(Uint16 channel, int16 duty_frac)
{
volatile struct EPWM_REGS *EPwmRegs;		// pointer to ePWM regs of specified channel
Uint32 width;
Uint16 frac;
Uint16 CMPALR;
Uint16 CMPAHR;

// Assign the pointer to the selected ePWM channel
	EPwmRegs = (volatile struct EPWM_REGS *)((Uint32)&EPwm1Regs + (0x0040u)*(channel - 1));

// Compute (100% - fraction)
	duty_frac = 0x7fff - duty_frac;

// Compute the lo and hi resolution compare values
	width = (Uint32)(duty_frac) * (Uint32)(EPwmRegs->TBPRD + 1);								// Q15 * Q0 = I17Q15
	CMPALR = (Uint16)(width >> 15);																// Isolate integer part (Q0)
	frac = (Uint16)(width & 0x7FFF);															// Isolate fractional part (Q15)
	CMPAHR = (Uint16)(((Uint32)frac * (Uint32)MEP_ScaleFactor[HRPWM_CAL_CH]) >> 7) + 0x0180;	// Compute the CMPAHR (Q0)

// Impose the HRPWM range limitations
	if(CMPALR < 3) CMPAHR = 0;						// Force CMPAHR to zero if below CMPALR lower limit
	if(CMPALR > (EPwmRegs->TBPRD - 2)) CMPAHR = 0;	// Force CMPAHR to zero if beyond CMPALR upper limit

// Write to the 32-bit compare register
	EPwmRegs->CMPA.all = ((Uint32)CMPALR << 16) + (Uint32)CMPAHR;

} // end WriteDac()


/*** end of file *****************************************************/
