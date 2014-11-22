/*
 * Audio.h
 *
 *  Created on: Nov 12, 2014
 *      Author: Edward
 */

#ifndef AUDIO_H_
#define AUDIO_H_

struct Audio
{
	const unsigned int *data;
	unsigned int length;
} beepLow, beepHigh, sine;

//1000Hz~1.wav
const unsigned int wavFile [] =
{
		18058, 22136, 27183, 27748, 25191, 19743, 13185, 7704, 4931, 5788,
		9993, 16219, 22496, 26842, 27881, 25287, 19878, 13365, 7806, 4958,
		5722, 9856, 16054, 22355, 26769, 27901, 25391, 20035, 13524, 7918,
		4987, 5659, 9721, 15889, 22213, 26695, 27918, 25494, 20191, 13685,
		8031, 5018, 5597, 9586, 15724
};

//Beep5~1.wav
const unsigned int wavFile2 [] =
{
		15325, 13841, 13482, 12829, 12990, 12437, 11273, 14850, 15263, 15965,
		15612, 15534, 15597, 15527, 15020, 13679, 13034, 10313, 6460, 8589,
		12819, 13273, 13853, 14723, 16598, 17603, 18343, 18059, 20501, 24006,
		23504, 23282, 23244, 19990, 16846, 17298, 17213, 17069, 17513, 15695,
		11970, 12017, 11426, 11496, 11102, 11176, 10558, 13693, 16164, 15740,
		19098, 22987, 22931, 23171, 23344, 23471, 23743, 22898, 22003, 20593,
		20780, 15150, 10172, 10894, 10586, 11732, 12505, 13121, 14027, 13954,
		13911, 12981, 14167, 16333, 15281, 18218, 17166, 15762, 15040, 14682,
		14950, 15677, 17199, 16942, 14222, 16241, 17457, 19471, 17888, 13362,
		13241, 12098, 11819, 10552, 11205, 14365, 14162, 15597, 17345, 18699,
		20708, 21596, 26034, 29382, 29223, 27803, 22266, 20759, 19106, 17493,
		15945
};

//Beep5High~1.wav
const unsigned int wavFile3[] =
{
		14281, 11892, 12430, 15335, 15679, 15737, 15481, 15626, 15142, 13634,
		12513, 7608, 8033, 13077, 13311, 14478, 16222, 17954, 18067, 19382,
		23731, 23409, 23458, 21202, 16940, 17420, 16967, 17572, 15485, 11820,
		11886, 11227, 11420, 10618, 12204, 15928, 16006, 20729, 23270, 22922,
		23516, 23453, 23518, 21681, 21119, 18635, 11279, 10499, 11010, 12004,
		13133, 13741, 14259, 13114, 14054, 15799, 16289, 17957, 15920, 14945,
		14825, 15132, 17262, 16150, 14824, 17045, 19342, 16897, 13002, 12681,
		11666, 10464, 12821, 14492, 15377, 17707, 19722, 21374, 25935, 29630,
		28677, 23620, 20291, 18799, 16469, 14599, 13330
};

void init_audio()
{
	sine.data = wavFile;
	sine.length = 45;

	beepLow.data = wavFile2;
	beepLow.length = 111;

	beepHigh.data = wavFile3;
	beepHigh.data = 87;
}

#endif /* AUDIO_H_ */
