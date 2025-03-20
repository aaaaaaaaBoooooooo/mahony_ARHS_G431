#include "filter.h"
#include "HANA_math.h"
////////////////////////////////////////////////////



 //һά������
float kalman_1(kalman_1_struct *ekf,float input) 
{
	ekf->Now_P = ekf->LastP + ekf->Q;
	ekf->Kg = ekf->Now_P / (ekf->Now_P + ekf->R);
	ekf->out = ekf->out + ekf->Kg * (input - ekf->out);
	ekf->LastP = (1-ekf->Kg) * ekf->Now_P ;
	return ekf->out;
}



//һά�������˲���ʼ��
void kalman1_init(kalman1_state *state, float init_x, float init_p)
{
    state->x = init_x;
    state->p = init_p;
    state->A = 1;
    state->H = 1;
    state->q = 2e2;//10e-6;  /* predict noise convariance */
    state->r = 5e2;//10e-5;  /* measure error convariance */
}
float kalman1_filter(kalman1_state *state, float z_measure)
{
    /* Predict */
    state->x = state->A * state->x;
    state->p = state->A * state->A * state->p + state->q;  /* p(n|n-1)=A^2*p(n-1|n-1)+q */

    /* Measurement */
    state->gain = state->p * state->H / (state->p * state->H * state->H + state->r);
    state->x = state->x + state->gain * (z_measure - state->H * state->x);
    state->p = (1 - state->gain * state->H) * state->p;

    return state->x;
}



/////////////////////////////////////////////////
//��ͨ�˲�
float LPF_1_Filter_1(Filter_LPF_1 *LPF_1)
{
	return LPF_1->old_data * (1 - LPF_1->factor) + LPF_1->new_data *  LPF_1->factor;
}


float LPF_1_Filter_2(Filter_LPF_1 *LPF_1,float dt)
{
	 return LPF_1->old_data + (dt /( 1 / ( 2 * PI * LPF_1->factor ) + dt)) * (LPF_1->new_data - LPF_1->old_data);    
}

/*====================================================================================================*
**���� : �������ͻ�����ֵ�˲�
**���� : ÿ�β�����һ�������ݷ�����У���N�����ݽ�������ƽ������
**���� : 
**ݔ�� : None
**��ע : None
**====================================================================================================*/
/*====================================================================================================*/
int16_t AntiPulse_MovingAverage_Filter(MovAverage *_MovAverage)
{
		uint8_t i;	
		int32_t sum=0;
		int16_t max=0;
		int16_t min=0xffff;
	    _MovAverage->max_cnt = 10;
			_MovAverage->average[_MovAverage->cnt] = _MovAverage->input;	
			_MovAverage->cnt++;			
			if(_MovAverage->cnt==_MovAverage->max_cnt)
			{
				_MovAverage->cnt=0;
			}	
			for(i=0;i<_MovAverage->max_cnt;i++)
			{
					if(_MovAverage->average[i]>max)
							max = _MovAverage->average[i];
					else if(_MovAverage->average[i]<min)
							min = _MovAverage->average[i];
					sum += _MovAverage->average[i];
			}
		return ((sum-max-min)/(_MovAverage->max_cnt-2));                                    
}


char filter_Init=0;
char filter_Place=0;
int16_t data_sum=0;
int16_t Moving_data[Moving_Len];

short Moving_filter(int16_t data)//�������ھ�ֵ�˲�
{
	char i;

	data_sum=0;
	
	if(filter_Init == 0)
	{

		Moving_data[filter_Place]=data;
		
		filter_Place++;
		if(filter_Place>=Moving_Len)
		{
			filter_Init=1;
			filter_Place=0;
		}
		return 0;
	}
	
	else
	{

		Moving_data[filter_Place]=data; 
		
		filter_Place++;
		
		if(filter_Place>=Moving_Len)filter_Place=0;		

		for(i=0;i<Moving_Len;i++)
		{

			data_sum+=Moving_data[i];	
		}
		return data_sum/Moving_Len;
	}	
}

///////////////////////////            END        //////////////////////////////

