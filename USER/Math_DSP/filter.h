#ifndef __filter_H
#define __filter_H
#include <math.h>
#include <string.h>
#include "stdint.h"

#define PI 3.1415927f
#define Moving_Len  10
////////////////////////////////////////
//¿¨¶ûÂüÂË²¨Æ÷
typedef struct{
	float x;
	float p;
	float A;
	float H;
	float q;
	float r;
	float gain;
}kalman1_state;


void kalman1_init(kalman1_state *state, float init_x, float init_p);
float kalman1_filter(kalman1_state *state, float z_measure);

///////////////////////////////////////////////////////////////////////////////////
//»¬¶¯¾ùÖµÂË²¨Æ÷
int16_t MovMiddle(int16_t input);

////////////////////////////////////////////////////////////////////////

/*  LPF 1st filter   */
typedef struct{
		float old_data;
		float new_data;
		float factor;
}Filter_LPF_1;
float LPF_1_Filter_1(Filter_LPF_1 *LPF_1);
float LPF_1_Filter_2(Filter_LPF_1 *LPF_1,float dt);


typedef struct {
	
	uint16_t cnt;
	int16_t input;
	int16_t *average;

	uint8_t max_cnt;
}MovAverage;

//¿¹¸ÉÈÅĞÍ»¬¶¯¾ùÖµÂË²¨Æ÷
int16_t AntiPulse_MovingAverage_Filter(MovAverage *_MovAverage);


//¾ùÖµÂË²¨Æ÷
short Moving_filter(short data);
///////////////////////////////////////////////////////////////////////////////////
#endif
