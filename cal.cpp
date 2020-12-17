#include <Arduino.h> // map() 이나 constrain() 등을 사용하기 위해 필요

static int servoCalData[] = { // 높이가 -44mm, 44mm 사이일 때의 duty 값
	2210, 2150, 2090, 2030, 1970, 1910, 1860, 1810, 1760, 1710,
	1660, 1610, 1570, 1525, 1480, 1435, 1390, 1345, 1300, 1250,
	1200, 1145, 1075,
};

struct irpair {
	float volt;
	float offset;
};

static struct irpair irCalData[] = { // 반비례 함수와 측정치 간의 오차
	{640, 0.0}, {472, 4.63945765}, {351, -9.53885028},
	{305, 0.0}, {269, 6.40550557}, {240, 9.18857020},
	{213, 0.0},
};


int getServoDuty(float height) // 0.1mm to us
{
	int section;
	double x1, y1, y2;
	height = constrain(height, -440, 440);
	section = map(height, -440, 440, 0, 23);
	section == 23 && section--;
	x1 = (section - 11) * 40;
	y1 = servoCalData[section];
	y2 = servoCalData[section + 1];

	return (int)( (y2 - y1) / 40.0 * (height - x1) + y1 ) - 405; // 기어가 부러져서 못내려감 :(
}

float getIRDistance(float volt) // voltage to mm
{
	int section;
	float x1, x2, y1, y2, dist;
	volt = constrain(volt, 213, 640);
	for (section = 5; section > 0; section--)
		if (volt <= irCalData[section].volt)
			break;
	x1 = irCalData[section].volt;
	x2 = irCalData[section + 1].volt;
	y1 = irCalData[section].offset;
	y2 = irCalData[section + 1].offset;
	dist = 66860.43794036 / (volt - 51.33744855) - 13.58024691;
	dist += (y2 - y1) / (x2 - x1) * (volt - x1) + y1; // add offset
	return dist;
}

/* ir calibration data
640: 100
472: 150
351: 200
305: 250
269: 300
240: 350
213: 400

NEU: 168
44	:	1075
40	:	1145
36	:	1200
32	:	1250
28	:	1300
24	:	1345
20	:	1390
16	:	1435
12	:	1480
8	:	1525
4	:	1570
0	:	1610
-4	:	1660
-8	:	1710
-12	:	1760
-16	:	1810
-20	:	1860
-24	:	1910
-28	:	1970
-32	:	2030
-36	:	2090
-40	:	2150
-44	:	2210
*/
