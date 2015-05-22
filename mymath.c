#include <stdlib.h>
#include <avr/pgmspace.h>
#include "mymath.h"

// discrete mathematics

// Sinus with argument in degree at an angular resolution of 1 degree and a discretisation of 13 bit.
const uint16_t pgm_sinlookup[91] PROGMEM = {0, 143, 286, 429, 571, 714, 856, 998, 1140, 1282, 1423, 1563, 1703, 1843, 1982, 2120, 2258, 2395, 2531, 2667, 2802, 2936, 3069, 3201, 3332, 3462, 3591, 3719, 3846, 3972, 4096, 4219, 4341, 4462, 4581, 4699, 4815, 4930, 5043, 5155, 5266, 5374, 5482, 5587, 5691, 5793, 5893, 5991, 6088, 6183, 6275, 6366, 6455, 6542, 6627, 6710, 6791, 6870, 6947, 7022, 7094, 7165, 7233, 7299, 7363, 7424, 7484, 7541, 7595, 7648, 7698, 7746, 7791, 7834, 7875, 7913, 7949, 7982, 8013, 8041, 8068, 8091, 8112, 8131, 8147, 8161, 8172, 8181, 8187, 8191, 8192};

int16_t c_sin_8192(int16_t angle)
{
	int8_t m,n;
	int16_t sinus;

	// avoid negative angles
	if (angle < 0)
	{
		m = -1;
		angle = abs(angle);
	}
	else m = +1;

	// fold angle to intervall 0 to 359
	angle %= 360;

	// check quadrant
	if (angle <= 90) n=1; // first quadrant
	else if ((angle > 90) && (angle <= 180)) {angle = 180 - angle; n = 1;} // second quadrant
	else if ((angle > 180) && (angle <= 270)) {angle = angle - 180; n = -1;} // third quadrant
	else {angle = 360 - angle; n = -1;}	//fourth quadrant
	// get lookup value
	sinus = pgm_read_word(&pgm_sinlookup[angle]);
	// calculate sinus value
	return (sinus * m * n);
}

// Cosinus with argument in degree at an angular resolution of 1 degree and a discretisation of 13 bit.
int16_t c_cos_8192(int16_t angle)
{
	return (c_sin_8192(90 - angle));
}
