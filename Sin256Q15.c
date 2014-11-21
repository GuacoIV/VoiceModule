/**********************************************************************
* File: sine256Q15.c                                                  *
* Description: 256 point sine wave in I1Q15 format.  Positive valued  *
*   only.  The values are amplitude and offset scaled.                *
* Devices: TMS320F28xx                                                *
* Author: David M. Alter, Texas Instruments Inc.                      *
* History:                                                            *
*   02/07/06 - original (D. Alter)                                    *
**********************************************************************/

#include "DSP28x_Project.h"				// Peripheral address definitions
//#include "f2808_HRPWM_DAC.h"				// Include file specific to this project

// AMPLITUDE, OFFSET, Q15_SCALE, and PWM_VOLTAGE are defined in f2808_HRPWM_DAC.h
//(2.0/3.3)+(32768*0.5/3.3) = 4965.4545454545454545454545454545 //.606 + 4964.8484
//#define scale     (AMPLITUDE/PWM_VOLTAGE)+(Q15_SCALE*OFFSET/PWM_VOLTAGE)

/*********************************************************************/
const unsigned int sine256Q15[] = {
16384, 16786, 17187, 17589, 17989,
18389, 18787, 19184, 19580, 19973,
20364, 20753, 21139, 21523, 21903,
22280, 22653, 23023, 23388, 23750,
24107, 24459, 24806, 25149, 25486,
25817, 26143, 26463, 26777, 27085,
27386, 27681, 27968, 28249, 28523,
28789, 29048, 29299, 29543, 29778,
30006, 30225, 30436, 30639, 30832,
31018, 31194, 31361, 31520, 31669,
31809, 31940, 32062, 32174, 32276,
32369, 32452, 32526, 32590, 32644,
32688, 32723, 32747, 32762, 32767,
32762, 32747, 32723, 32688, 32644,
32590, 32526, 32452, 32369, 32276,
32174, 32062, 31940, 31809, 31669,
31520, 31361, 31194, 31018, 30832,
30639, 30436, 30225, 30006, 29778,
29543, 29299, 29048, 28789, 28523,
28249, 27968, 27681, 27386, 27085,
26777, 26463, 26143, 25817, 25486,
25149, 24806, 24459, 24107, 23750,
23388, 23023, 22653, 22280, 21903,
21523, 21139, 20753, 20364, 19973,
19580, 19184, 18787, 18389, 17989,
17589, 17187, 16786, 16384, 15981,
15580, 15178, 14778, 14378, 13980,
13583, 13187, 12794, 12403, 12014,
11628, 11244, 10864, 10487, 10114,
 9744,  9379,  9017,  8660,  8308,
 7961,  7618,  7281,  6950,  6624,
 6304,  5990,  5682,  5381,  5086,
 4799,  4518,  4244,  3978,  3719,
 3468,  3224,  2989,  2761,  2542,
 2331,  2128,  1935,  1749,  1573,
 1406,  1247,  1098,   958,   827,
  705,   593,   491,   398,   315,
  241,   177,   123,    79,    44,
   20,     5,     0,     5,    20,
   44,    79,   123,   177,   241,
  315,   398,   491,   593,   705,
  827,   958,  1098,  1247,  1406,
 1573,  1749,  1935,  2128,  2331,
 2542,  2761,  2989,  3224,  3468,
 3719,  3978,  4244,  4518,  4799,
 5086,  5381,  5682,  5990,  6304,
 6624,  6950,  7281,  7618,  7961,
 8308,  8660,  9017,  9379,  9744,
10114, 10487, 10864, 11244, 11628,
12014, 12403, 12794, 13187, 13583,
13980, 14378, 14778, 15178, 15580,
15981
};

/*** end of file *****************************************************/
