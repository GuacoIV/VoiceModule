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
#include "f2808_HRPWM_DAC.h"				// Include file specific to this project

// AMPLITUDE, OFFSET, Q15_SCALE, and PWM_VOLTAGE are defined in f2808_HRPWM_DAC.h
#define scale     (AMPLITUDE/PWM_VOLTAGE)+(Q15_SCALE*OFFSET/PWM_VOLTAGE)

/*********************************************************************/
const unsigned int sine256Q15[] = {
16384u*scale, 16786u*scale, 17187u*scale, 17589u*scale, 17989u*scale,
18389u*scale, 18787u*scale, 19184u*scale, 19580u*scale, 19973u*scale,
20364u*scale, 20753u*scale, 21139u*scale, 21523u*scale, 21903u*scale,
22280u*scale, 22653u*scale, 23023u*scale, 23388u*scale, 23750u*scale,
24107u*scale, 24459u*scale, 24806u*scale, 25149u*scale, 25486u*scale,
25817u*scale, 26143u*scale, 26463u*scale, 26777u*scale, 27085u*scale,
27386u*scale, 27681u*scale, 27968u*scale, 28249u*scale, 28523u*scale,
28789u*scale, 29048u*scale, 29299u*scale, 29543u*scale, 29778u*scale,
30006u*scale, 30225u*scale, 30436u*scale, 30639u*scale, 30832u*scale,
31018u*scale, 31194u*scale, 31361u*scale, 31520u*scale, 31669u*scale,
31809u*scale, 31940u*scale, 32062u*scale, 32174u*scale, 32276u*scale,
32369u*scale, 32452u*scale, 32526u*scale, 32590u*scale, 32644u*scale,
32688u*scale, 32723u*scale, 32747u*scale, 32762u*scale, 32767u*scale,
32762u*scale, 32747u*scale, 32723u*scale, 32688u*scale, 32644u*scale,
32590u*scale, 32526u*scale, 32452u*scale, 32369u*scale, 32276u*scale,
32174u*scale, 32062u*scale, 31940u*scale, 31809u*scale, 31669u*scale,
31520u*scale, 31361u*scale, 31194u*scale, 31018u*scale, 30832u*scale,
30639u*scale, 30436u*scale, 30225u*scale, 30006u*scale, 29778u*scale,
29543u*scale, 29299u*scale, 29048u*scale, 28789u*scale, 28523u*scale,
28249u*scale, 27968u*scale, 27681u*scale, 27386u*scale, 27085u*scale,
26777u*scale, 26463u*scale, 26143u*scale, 25817u*scale, 25486u*scale,
25149u*scale, 24806u*scale, 24459u*scale, 24107u*scale, 23750u*scale,
23388u*scale, 23023u*scale, 22653u*scale, 22280u*scale, 21903u*scale,
21523u*scale, 21139u*scale, 20753u*scale, 20364u*scale, 19973u*scale,
19580u*scale, 19184u*scale, 18787u*scale, 18389u*scale, 17989u*scale,
17589u*scale, 17187u*scale, 16786u*scale, 16384u*scale, 15981u*scale,
15580u*scale, 15178u*scale, 14778u*scale, 14378u*scale, 13980u*scale,
13583u*scale, 13187u*scale, 12794u*scale, 12403u*scale, 12014u*scale,
11628u*scale, 11244u*scale, 10864u*scale, 10487u*scale, 10114u*scale,
 9744u*scale,  9379u*scale,  9017u*scale,  8660u*scale,  8308u*scale,
 7961u*scale,  7618u*scale,  7281u*scale,  6950u*scale,  6624u*scale,
 6304u*scale,  5990u*scale,  5682u*scale,  5381u*scale,  5086u*scale,
 4799u*scale,  4518u*scale,  4244u*scale,  3978u*scale,  3719u*scale,
 3468u*scale,  3224u*scale,  2989u*scale,  2761u*scale,  2542u*scale,
 2331u*scale,  2128u*scale,  1935u*scale,  1749u*scale,  1573u*scale,
 1406u*scale,  1247u*scale,  1098u*scale,   958u*scale,   827u*scale,
  705u*scale,   593u*scale,   491u*scale,   398u*scale,   315u*scale,
  241u*scale,   177u*scale,   123u*scale,    79u*scale,    44u*scale,
   20u*scale,     5u*scale,     0u*scale,     5u*scale,    20u*scale,
   44u*scale,    79u*scale,   123u*scale,   177u*scale,   241u*scale,
  315u*scale,   398u*scale,   491u*scale,   593u*scale,   705u*scale,
  827u*scale,   958u*scale,  1098u*scale,  1247u*scale,  1406u*scale,
 1573u*scale,  1749u*scale,  1935u*scale,  2128u*scale,  2331u*scale,
 2542u*scale,  2761u*scale,  2989u*scale,  3224u*scale,  3468u*scale,
 3719u*scale,  3978u*scale,  4244u*scale,  4518u*scale,  4799u*scale,
 5086u*scale,  5381u*scale,  5682u*scale,  5990u*scale,  6304u*scale,
 6624u*scale,  6950u*scale,  7281u*scale,  7618u*scale,  7961u*scale,
 8308u*scale,  8660u*scale,  9017u*scale,  9379u*scale,  9744u*scale,
10114u*scale, 10487u*scale, 10864u*scale, 11244u*scale, 11628u*scale,
12014u*scale, 12403u*scale, 12794u*scale, 13187u*scale, 13583u*scale,
13980u*scale, 14378u*scale, 14778u*scale, 15178u*scale, 15580u*scale,
15981u*scale
};

/*** end of file *****************************************************/
