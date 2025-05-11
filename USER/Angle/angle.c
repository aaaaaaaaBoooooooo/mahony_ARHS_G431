/********************************Copyright (c)**********************************`
** �ļ�����: angle.c
** ������Ա: aBo
** ��������: 2025-5-04
** �ĵ�����: ��������̬������غ���ʵ��
**����Mahony��Ԫ����6/9��IMU��̬����
  ��������������������ٶ�ģֵ�仯�Ķ�̬PIϵ��
  ��������Ӧ�����ھ�����̬�龰������������ٶȼƵ����ų̶�
  �����˶�̬�龰�£����ڻ�����ٶ�Ӱ�죬���ٶȼƽ�����̬�����
  ******************************************************
  ͬʱ���ڿ������˲�˼�룬ͨ����ʷ���ݵ�б�ʣ�Ԥ�⵱ǰ���ݣ����봫��������ֵ�����ں�
  �ﵽ�ھ�̬�龰�½��������ǲ����������ڶ�̬�龰�²���ʧ��Ӧ�ٶȵ�����
  ******************************************************
  ͨ���Լ��ٶȼƽ�������У׼�������ƫ�Ͳ���ϵ��������������ٶȼ�������֮��Ĳ������
  ******************************************************
  ͨ������������������Ʋ����ش������������õ����Ժ���Ƕȣ����������ǻ��ֵõ��ĺ����
  ����һ���ں��˲����ﵽ���ʹ����Ʋ�������ͬʱ�����˻���Ư�Ƶ�Ч��
********************************End of Head************************************/
#include "angle.h"
#include "delay.h"
#include "icm45686_task.h"
#include "filter.h"
#include "HANA_math.h"
#include "stdio.h"
#include "bmm350_task.h" 
#include "usart.h"
#define Gyro_NUM	500 // ���ٶȲ�������  ��MCU�ڴ��С�޸�(������̬�ֲ���������ԣ�����������50)
#define Acc_NUM   100 // ���ٶȲ�������	��MCU�ڴ��С�޸�(������̬�ֲ���������ԣ�����������50)

AHRS_TypeDef my_ahrs;//��̬��������


/*******************************************************************************
** ��������: IMU_Calibration()
** ��������: �����Ǿ�̬��ƫУ���Լ���̬�������ٶȻ�ȡ
** ����˵��: : 
** ����˵��: None
** ������Ա: JLB
** ��������: 2023-11-29
**------------------------------------------------------------------------------
** �޸���Ա:aBo
** �޸�����:2025-5-04
** �޸�����:�����������ٶ�����У׼������ʼ����������Ԫ����ʼ��
**------------------------------------------------------------------------------
********************************************************************************/
void IMU_Calibration(void)
{
	uint16_t i=0;
	vector3float_t gyro[Gyro_NUM];//���ٶȼ�ԭʼ����
	vector3float_t gyro_avg = {0.0f,0.0f,0.0f};//���ٶȼ�ƽ��ֵ
	vector3float_t gyro_var = {0.0f,0.0f,0.0f};//��Ž��ٶȼƷ���

	
	vector3float_t acc[Acc_NUM];//���ٶȼ�ԭʼ����
	vector3float_t acc_avg = {0.0f,0.0f,0.0f};//���ٶȼ�ƽ��ֵ
	vector3float_t acc_var = {0.0f,0.0f,0.0f};//��ż��ٶȼƷ���

	do
	{
		/****��������begin****/
		gyro_avg.x = 0;
		gyro_avg.y = 0;
		gyro_avg.z = 0;
		gyro_var.x = 0;
		gyro_var.y = 0;
		gyro_var.z = 0;
		my_ahrs.IMU_Data.gyro_offset.x = 0;
		my_ahrs.IMU_Data.gyro_offset.y = 0;
		my_ahrs.IMU_Data.gyro_offset.z = 0;
		acc_avg.x = 0;
		acc_avg.y = 0;
		acc_avg.z = 0;
		acc_var.x = 0;
		acc_var.y = 0;
		acc_var.z = 0;
		my_ahrs.IMU_Data.acc_static_value = 0;	
		/****��������end****/
		
		for(i=0; i<Gyro_NUM; i++)//�ɼ����������ݼ����ֵ
		{
			/***��ȡIMUԭʼ���ٶ�����***/
			icm45686_data_update();

			/***��λת��Ϊ��/s***/
			gyro[i].x = myIMU_data.gyro_x*Gyro_Gain;
			gyro[i].y = myIMU_data.gyro_y*Gyro_Gain;
			gyro[i].z = myIMU_data.gyro_z*Gyro_Gain;

			/***�ۼ����***/
			gyro_avg.x += gyro[i].x;
			gyro_avg.y += gyro[i].y;
			gyro_avg.z += gyro[i].z;

			delay_ms(1);//���ݽ���Ƶ�ʣ����ò�����Ϊ1KHz
			
		}
		/***��ƽ��,�õ����ٶ���Ư***/
		gyro_avg.x = gyro_avg.x/Gyro_NUM;
		gyro_avg.y = gyro_avg.y/Gyro_NUM;
		gyro_avg.z = gyro_avg.z/Gyro_NUM;

		
		for(i=0;i<Acc_NUM;i++)//�ɼ����ٶ����ݼ����ֵ
		{
			
			/***��ȡIMUԭʼ���ٶ�����***/
			icm45686_data_update();

			/***��λת��Ϊ g(m/s^2)***/
			acc[i].x = myIMU_data.acc_x*Acc_Gain;
			acc[i].y = myIMU_data.acc_y*Acc_Gain;
			acc[i].z = myIMU_data.acc_z*Acc_Gain;

			/***�ۼ����***/
			acc_avg.x += acc[i].x;
			acc_avg.y += acc[i].y;
			acc_avg.z += acc[i].z;

			delay_ms(1);//���ݽ���Ƶ�ʣ����ò�����Ϊ1KHz		
		}
		/***��ƽ��,�õ����ٶȾ�̬��������Ϊʵ���������ٶ�***/
		acc_avg.x = acc_avg.x/Acc_NUM;
		acc_avg.y = acc_avg.y/Acc_NUM;
		acc_avg.z = acc_avg.z/Acc_NUM;

		
		/***���㷽�� *ȷ��У׼��ʱ���Ǿ�ֹ״̬�ģ�ͬʱ��ô�������������ˮƽ***/
		for(i=0; i<Gyro_NUM; i++)
		{
			gyro_var.x += (float) (1.0f/(Gyro_NUM-1)) * (gyro[i].x - gyro_avg.x) * (gyro[i].x - gyro_avg.x);
			gyro_var.y += (float) (1.0f/(Gyro_NUM-1)) * (gyro[i].y - gyro_avg.y) * (gyro[i].y - gyro_avg.y);
			gyro_var.z += (float)	(1.0f/(Gyro_NUM-1)) * (gyro[i].z - gyro_avg.z) * (gyro[i].z - gyro_avg.z);
		}
		for(i=0; i<Acc_NUM; i++)
		{
			acc_var.x += (float) (1.0f/(Acc_NUM-1)) * (acc[i].x - acc_avg.x) * (acc[i].x - acc_avg.x);
			acc_var.y += (float) (1.0f/(Acc_NUM-1)) * (acc[i].y - acc_avg.y) * (acc[i].y - acc_avg.y);
			acc_var.z += (float) (1.0f/(Acc_NUM-1)) * (acc[i].z - acc_avg.z) * (acc[i].z - acc_avg.z);
		}
		my_ahrs.IMU_Data.gyro_var.x = gyro_var.x;
		my_ahrs.IMU_Data.gyro_var.y = gyro_var.y;
		my_ahrs.IMU_Data.gyro_var.z = gyro_var.z;
		my_ahrs.IMU_Data.acc_var.x = acc_var.x;
		my_ahrs.IMU_Data.acc_var.y = acc_var.y;
		my_ahrs.IMU_Data.acc_var.z = acc_var.z;		
		printf("gyro_var\nx:%f\ny:%f\nz:%f\n",my_ahrs.IMU_Data.gyro_var.x,my_ahrs.IMU_Data.gyro_var.y,my_ahrs.IMU_Data.gyro_var.z);
		printf("acc_var\nx:%f\ny:%f\nz:%f\n",my_ahrs.IMU_Data.acc_var.x,my_ahrs.IMU_Data.acc_var.y,my_ahrs.IMU_Data.acc_var.z);
		

		/***�жϲ����澲ֹʱ����ƫ***/
		if( gyro_var.x<VAR_GyX && gyro_var.y<VAR_GyY && gyro_var.z<VAR_GyZ&&acc_var.x<VAR_AcX && acc_var.y<VAR_AcY &&acc_var.z<VAR_AcZ)//�����㹻С
		{
			/***��Ư��ȡbegin***/
			my_ahrs.IMU_Data.gyro_offset.x  = gyro_avg.x;
			my_ahrs.IMU_Data.gyro_offset.y  = gyro_avg.y;
			my_ahrs.IMU_Data.gyro_offset.z  = gyro_avg.z;
			my_ahrs.IMU_Data.acc_static_value		=		1.0f/invSqrt(acc_avg.x*acc_avg.x + acc_avg.y*acc_avg.y +acc_avg.z*acc_avg.z);

			/***��Ư��ȡend***/
			
			printf("gyro_offset:x%f\n  y%f\n  z%f\r\n",my_ahrs.IMU_Data.gyro_offset.x,my_ahrs.IMU_Data.gyro_offset.y,my_ahrs.IMU_Data.gyro_offset.z);
			printf("acc_offset:%f\r\n",my_ahrs.IMU_Data.acc_static_value);
			my_ahrs.is_init_success=1;//��ƯУ׼�ɹ�
			/***��Ԫ����ʼ��***/
			my_ahrs.NumQ.q0 = 1.0f;
			my_ahrs.NumQ.q1 = 0.0f;
			my_ahrs.NumQ.q2 = 0.0f;
			my_ahrs.NumQ.q3 = 0.0f;

#ifdef SIX_CAL
			/***���ٶȼ�����У׼ϵ����ʼ��(ͨ������У׼��ã�һ�������IMU�����Ѿ�У׼������Ӱ���С)***/
			my_ahrs.IMU_Data.acc_offset.x = -0.013347f;
			my_ahrs.IMU_Data.acc_offset.y = 0.009929f;
			my_ahrs.IMU_Data.acc_offset.z = 0.000814f;
			my_ahrs.IMU_Data.acc_Scale_K.x = 0.995170f;
			my_ahrs.IMU_Data.acc_Scale_K.y = 1.000649f;
			my_ahrs.IMU_Data.acc_Scale_K.z = 0.997839f;	
#endif
			return;
		}
		delay_ms(1000);//���Ǿ�ֹ״̬����ʱ�ȴ��ȶ�
	}while(1);

	
}


/*******************************************************************************
** ��������: IMU_DataUpdate(void)
** ��������: IMUԭʼ���ݸ��£����ٶ�һ�׿������˲���IMU���ٶ����ݸ���
** ����˵��: : 
** ����˵��: None
** ������Ա: JLB
** ��������: 2024-1-20
**------------------------------------------------------------------------------
** �޸���Ա:aBo
** �޸�����:2025-5-04
** �޸�����:���ӽ��ٶȼ���������һά�������˲������Ӽ��ٶȼ�����У׼�ӿ�
**------------------------------------------------------------------------------
********************************************************************************/
void IMU_DataUpdate(void)
{
	/***IMUԭʼ���ݸ���***/
	icm45686_data_update();

	/***IMU���ٶ����ݸ���   ��λΪ��/s***/
	my_ahrs.IMU_Data.gyro.x= myIMU_data.gyro_x*Gyro_Gain-my_ahrs.IMU_Data.gyro_offset.x;
	my_ahrs.IMU_Data.gyro.y= myIMU_data.gyro_y*Gyro_Gain-my_ahrs.IMU_Data.gyro_offset.y;
	my_ahrs.IMU_Data.gyro.z= myIMU_data.gyro_z*Gyro_Gain-my_ahrs.IMU_Data.gyro_offset.z;
	
	/***���ٶ�һ�׿������˲�begin***/
	static  kalman1_state ekf[3] = {{0.0f,0.0f,1.0f,1.0f,0.0005f,0.0f},{0.0f,0.0f,1.0f,1.0f,0.0005f,0.0f},{0.0f,0.0f,1.0f,1.0f,0.0005f,0.0f}};//������������ڵ���Ԥ��ֵ���Խ��Խ���δ�����
	static float last_out[3],lastlast_out[3];
	/*****�������������*****/
	ekf[0].r = my_ahrs.IMU_Data.gyro_var.x; //�����ǲ�������
	ekf[1].r = my_ahrs.IMU_Data.gyro_var.y; //�����ǲ�������
	ekf[2].r = my_ahrs.IMU_Data.gyro_var.z; //�����ǲ�������
	/*****������Ԥ��ϵ������*****/
	ekf[0].A = last_out[0] - lastlast_out[0];//�������Ƽ���Ԥ��б��
	ekf[1].A = last_out[1] - lastlast_out[1];//�������Ƽ���Ԥ��б��
	ekf[2].A = last_out[2] - lastlast_out[2];//�������Ƽ���Ԥ��б��
	/*****�������˲����*****/
	kalman1_filter(&ekf[0],my_ahrs.IMU_Data.gyro.x);  //��x����ٶ�һά�������˲�
	my_ahrs.IMU_Data.gyro.x=ekf[0].x;
	kalman1_filter(&ekf[1],my_ahrs.IMU_Data.gyro.y);  //��Y����ٶ�һά�������˲�
	my_ahrs.IMU_Data.gyro.y=ekf[1].x;
	kalman1_filter(&ekf[2],my_ahrs.IMU_Data.gyro.z);  //��Z����ٶ�һά�������˲�	
	my_ahrs.IMU_Data.gyro.z=ekf[2].x;
	/*****��ʷ���ݸ���*****/
	lastlast_out[0] = last_out[0];
	last_out[0] = my_ahrs.IMU_Data.gyro.x;
	lastlast_out[1] = last_out[1];
	last_out[1] = my_ahrs.IMU_Data.gyro.y;
	lastlast_out[2] = last_out[2];
	last_out[2] = my_ahrs.IMU_Data.gyro.z;	
	/***���ٶ�һ�׿������˲�end*****/	

  
	/***IMU���ٶ����ݸ���   ��λΪ g(m/s^2) ***/
#ifdef SIX_CAL
	my_ahrs.IMU_Data.acc.x=(myIMU_data.acc_x*Acc_Gain - my_ahrs.IMU_Data.acc_offset.x)*my_ahrs.IMU_Data.acc_Scale_K.x;//IMU���ٶ����ݸ��� ����У׼(Ӱ���С)      // ��λΪ g(m/s^2)
	my_ahrs.IMU_Data.acc.y=(myIMU_data.acc_y*Acc_Gain - my_ahrs.IMU_Data.acc_offset.y)*my_ahrs.IMU_Data.acc_Scale_K.y ;
	my_ahrs.IMU_Data.acc.z=(myIMU_data.acc_z*Acc_Gain - my_ahrs.IMU_Data.acc_offset.z)*my_ahrs.IMU_Data.acc_Scale_K.z ;
#else
	my_ahrs.IMU_Data.acc.x=(myIMU_data.acc_x)*Acc_Gain;
	my_ahrs.IMU_Data.acc.y=(myIMU_data.acc_y)*Acc_Gain;
	my_ahrs.IMU_Data.acc.z=(myIMU_data.acc_z)*Acc_Gain;
#endif
	

	/***IMU�¶����ݸ���   ��λΪ ���϶� ***/
	my_ahrs.IMU_Data.temperature = (myIMU_data.temp/128.0f)+25.0f;
	
}
/*******************************************************************************
** ��������: GetAngle(float dt) 
** ��������: ����Mahony��Ԫ�������˲�����Ƕ���Ϣ
** ����˵��:  
					dt:����ʱ�� ��λ��s				(�Ƽ���������Ƶ�ʣ�500Hz���ϣ�����Ƶ��һ��Ҫ�ȶ�,����ʱ��dtһ��Ҫ׼ȷ)
** ����˵��: None
** ������Ա: JLB
** ��������: 2024-3-16
**------------------------------------------------------------------------------
** �޸���Ա:ABO
** �޸�����:2025-5-04
** �޸�����:�Ż���Mahony�㷨�������̣�����˶�̬PIϵ�������Ӵ����ƺ���ǻ����ں�
**------------------------------------------------------------------------------
********************************************************************************/
void IMU_GetAngle(float dt) 
{		
	volatile struct V{
				float x;
				float y;
				float z;
				} halfGravity,//���ٶ�ͨ����Ԫ����ת�����õ������������ٶ�����
					Acc,//ʵ�ʼ��ٶȼƲ������������ٶ���������̬ʱ׼ȷ����̬ʱ��׼��
					Gyro,//�����ٶ�������Ľ��ٶ�
					AccGravity;//�������ٶȵ�ʵ��ֵ������ֵ���֮���ģֵ���������

	static struct V GyroIntegError = {0};//�����ֲ�����
	float q0_t,q1_t,q2_t,q3_t;//����������ݴ����	
	float NormQuat; //��һ��ϵ��
	float HalfTime = dt * 0.5f;//�����ʱ��->���ٳ˷�����
	float INV_TAU = 0.15f;//Mahony�˲�PI������ʱ�䳣���ĵ���
	float twoKpDef = 8.584f *INV_TAU ;//���ٶȺͼ��ٶ��ں��˲�ϵ��  ���ƶԼ��ٶȼƵ����ų̶ȣ�һ�㶯̬�¼��ٶȼƲ�����
	float twoKiDef = 9.210632f*INV_TAU*INV_TAU*dt;//�������ں��˲�ϵ��   һ��������Ư�����Ͳ���Ҫ�����ˣ�����һ����С����


/*****************Mahony�ں��˲������㷨begin********************/

    /*** ֻ�ڼ��ٶȼ�������Чʱ�Ž���������� ***/     
	if(!((my_ahrs.IMU_Data.acc.x == 0.0f) && (my_ahrs.IMU_Data.acc.y == 0.0f) && (my_ahrs.IMU_Data.acc.z == 0.0f))) 
	{
		/***ͨ����Ԫ���õ������������ٶ�����Gravity��һ��----���ٺ��������еĳ˷�����***/ 
		halfGravity.x = (my_ahrs.NumQ.q1 * my_ahrs.NumQ.q3 - my_ahrs.NumQ.q0 * my_ahrs.NumQ.q2);								
		halfGravity.y = (my_ahrs.NumQ.q0 * my_ahrs.NumQ.q1 + my_ahrs.NumQ.q2 * my_ahrs.NumQ.q3);						  
		halfGravity.z = 0.5f-(my_ahrs.NumQ.q1 * my_ahrs.NumQ.q1 + my_ahrs.NumQ.q2 * my_ahrs.NumQ.q2);

		/***�����ٶȼƵõ���ʵ���������ٶ�����Acc��һ��***/
		NormQuat = invSqrt(sq(my_ahrs.IMU_Data.acc.x)+ sq(my_ahrs.IMU_Data.acc.y) +sq(my_ahrs.IMU_Data.acc.z));
		Acc.x = my_ahrs.IMU_Data.acc.x * NormQuat;
		Acc.y = my_ahrs.IMU_Data.acc.y * NormQuat;
		Acc.z = my_ahrs.IMU_Data.acc.z * NormQuat;	
		
#ifdef Dynamic_PI
		/***��̬PIϵ��***/
		float now_G = 1.0f/NormQuat;
		INV_TAU *= expf(-fabs(now_G - my_ahrs.IMU_Data.acc_static_value)/my_ahrs.IMU_Data.acc_static_value);
		twoKiDef = 9.210632f*INV_TAU*INV_TAU*dt;
		twoKpDef = 8.584f *INV_TAU ;
#endif

		/***��ʵ���������ٶ�����Acc�������������ٶ�����Gravity������õ����ߵ����***/
		AccGravity.x = (Acc.y * halfGravity.z - Acc.z * halfGravity.y);
		AccGravity.y = (Acc.z * halfGravity.x - Acc.x * halfGravity.z);
		AccGravity.z = (Acc.x * halfGravity.y - Acc.y * halfGravity.x);
		
		/***�������л���***/
		GyroIntegError.x += AccGravity.x * twoKiDef;
		GyroIntegError.y += AccGravity.y * twoKiDef;
		GyroIntegError.z += AccGravity.z * twoKiDef;
		
		/***���ٶ��ںϼ��ٶ��������ں��˲�***/
		Gyro.x = my_ahrs.IMU_Data.gyro.x * DegtoRad + twoKpDef * AccGravity.x  +  GyroIntegError.x;//������
		Gyro.y = my_ahrs.IMU_Data.gyro.y * DegtoRad + twoKpDef * AccGravity.y  +  GyroIntegError.y;
		Gyro.z = my_ahrs.IMU_Data.gyro.z * DegtoRad + twoKpDef * AccGravity.z  +  GyroIntegError.z;		
	}

	/***һ��������������΢�ַ���, ������Ԫ��***/
	q0_t = (-my_ahrs.NumQ.q1*Gyro.x - my_ahrs.NumQ.q2*Gyro.y - my_ahrs.NumQ.q3*Gyro.z) * HalfTime;
	q1_t = ( my_ahrs.NumQ.q0*Gyro.x - my_ahrs.NumQ.q3*Gyro.y + my_ahrs.NumQ.q2*Gyro.z) * HalfTime;
	q2_t = ( my_ahrs.NumQ.q3*Gyro.x + my_ahrs.NumQ.q0*Gyro.y - my_ahrs.NumQ.q1*Gyro.z) * HalfTime;
	q3_t = (-my_ahrs.NumQ.q2*Gyro.x + my_ahrs.NumQ.q1*Gyro.y + my_ahrs.NumQ.q0*Gyro.z) * HalfTime;
	my_ahrs.NumQ.q0 += q0_t;
	my_ahrs.NumQ.q1 += q1_t;
	my_ahrs.NumQ.q2 += q2_t;
	my_ahrs.NumQ.q3 += q3_t;
	
	/***��λ����Ԫ�� ��֤��Ԫ���ڵ��������б��ֵ�λ����***/
	NormQuat = invSqrt(sq(my_ahrs.NumQ.q0) + sq(my_ahrs.NumQ.q1) + sq(my_ahrs.NumQ.q2) + sq(my_ahrs.NumQ.q3));
	my_ahrs.NumQ.q0 *= NormQuat;
	my_ahrs.NumQ.q1 *= NormQuat;
	my_ahrs.NumQ.q2 *= NormQuat;
	my_ahrs.NumQ.q3 *= NormQuat;	

/*****************Mahony�ں��˲������㷨end********************/

/*****************��̬ŷ���ǻ�ȡbegin*******************************/
	/***������Ԫ����ת�������***/
	float R31 = 2.0f*(my_ahrs.NumQ.q1 * my_ahrs.NumQ.q3 - my_ahrs.NumQ.q0 *my_ahrs.NumQ.q2 ) ;/*����(3,1)��*/
	float R32 = 2.0f*(my_ahrs.NumQ.q2 *my_ahrs.NumQ.q3 + my_ahrs.NumQ.q0 * my_ahrs.NumQ.q1);/*����(3,2)��*/
	float R33 = 1.0f - 2.0f * ((my_ahrs.NumQ.q1 *my_ahrs.NumQ.q1) + (my_ahrs.NumQ.q2 * my_ahrs.NumQ.q2));	/*����(3,3)��*/		
//	float R11 = 1.0f - 2.0f *((my_ahrs.NumQ.q2 *my_ahrs.NumQ.q2) + (my_ahrs.NumQ.q3 * my_ahrs.NumQ.q3));	/*����(1,1)��*/		
//	float R21 = 2.0f*(my_ahrs.NumQ.q1 * my_ahrs.NumQ.q2 - my_ahrs.NumQ.q0 *my_ahrs.NumQ.q3);/*����(2,1)��*/		

	my_ahrs.Angle_Data.pitch  =  atan2f(-R31,sqrtf(R32*R32+R33*R33))* RadtoDeg;	 //������					
	my_ahrs.Angle_Data.roll	= atan2f(R32,R33) * RadtoDeg;	//�����
	//Angle_Data.yaw = atan2f(R21,R11) * RadtoDeg;	//ƫ���� 
#ifdef USE_MMU
static uint8_t mmu_yaw_init_flag = 0; //�����Ƴ�ʼ����־λ
		/*****ʹ�ô����� *****/	
		if(my_at_cmd.mmu_mode)
		{
			if(my_MMU.is_init_ok==1&&my_MMU.angle.yaw!=0.0f&&mmu_yaw_init_flag==0)//�����Ƴ�ʼ���ɹ�
			{
				my_ahrs.Angle_Data.yaw = my_MMU.angle.yaw; //ʹ�ô����Ƶ�ƫ���ǳ�ʼ������
				mmu_yaw_init_flag = 1;
			}
			my_ahrs.Angle_Data.yaw  += Gyro.z*RadtoDeg* dt;//���ٶȻ��ֳ�ƫ����
			/***ƫ����ӳ��Ϊ0~360������Ƽ���***/
			if(my_ahrs.Angle_Data.yaw>360.0f)
					my_ahrs.Angle_Data.yaw -= 360.0f;
			else if(my_ahrs.Angle_Data.yaw<0.0f)
					my_ahrs.Angle_Data.yaw += 360.0f;
		}
		else
		{
			if(fabs(Gyro.z) > 0.01f) //����̫С������Ϊ�Ǹ��ţ�����ƫ������
			{
				/*****��ʹ�ô�����*****/
				my_ahrs.Angle_Data.yaw  += Gyro.z*RadtoDeg* dt;//���ٶȻ��ֳ�ƫ����		
		
			} 			
		}

#else
	if(fabs(Gyro.z) > 0.01f) //����̫С������Ϊ�Ǹ��ţ�����ƫ������
	{
		/*****��ʹ�ô�����*****/
		my_ahrs.Angle_Data.yaw  += Gyro.z*RadtoDeg* dt;//���ٶȻ��ֳ�ƫ����		

	} 
#endif
/*****************��̬ŷ���ǻ�ȡend*******************************/	
}


#ifdef USE_MMU
/*****ʹ�ô����� *****/
void mmu_angle_update()
{
	float Mzx = 0.0f, Mzy = 0.0f;
	uint8_t rslt = 0;
	if(my_at_cmd.mmu_mode)
	{
		/*���������ݸ���*/
		rslt = mmu_data_update();
		if(rslt==BMM350_OK)
		{
			/*������ˮƽƫת����*/
			Mzx = cosf(my_ahrs.Angle_Data.roll*DegtoRad)*my_MMU.Data.x 
						+ sinf(my_ahrs.Angle_Data.roll*DegtoRad)*sinf(my_ahrs.Angle_Data.pitch*DegtoRad)*my_MMU.Data.y
						+ sinf(my_ahrs.Angle_Data.roll*DegtoRad)*cosf(my_ahrs.Angle_Data.pitch*DegtoRad)*my_MMU.Data.z;
			
			Mzy = cosf(my_ahrs.Angle_Data.pitch*DegtoRad)*my_MMU.Data.y - sinf(my_ahrs.Angle_Data.pitch*DegtoRad)*my_MMU.Data.z;
			/*�����ƺ���Ǽ��� 0~360*/
			my_MMU.angle.yaw = 180.0f+atan2f(Mzy,Mzx)*RadtoDeg;
			/*�����ƺ�����������Ǻ�����ںϻ����˲�*/
			if(fabs(my_ahrs.Angle_Data.roll)<10.0f&&fabs(my_ahrs.Angle_Data.pitch)<10.0f)//ˮƽ״̬���ں�
			{
				if(fabs(my_MMU.angle.yaw-my_ahrs.Angle_Data.yaw)>350.0f)//��������ƺ�����������Ǻ���ǲ�ֵ����350�ȣ�˵��������������360��0֮��������
				{
					my_ahrs.Angle_Data.yaw = my_MMU.angle.yaw;//ֱ��ʹ�ô���������
				}
				else/*�����ƺ�����������Ǻ���ǲ�ֵС��350�ȣ�˵����������������*/
				{
					my_ahrs.Angle_Data.yaw += (my_MMU.angle.yaw-my_ahrs.Angle_Data.yaw)*0.1f;
				}
			}
			else//��б״̬���ں�
			{
				if(fabs(my_MMU.angle.yaw-my_ahrs.Angle_Data.yaw)>350.0f)//��������ƺ�����������Ǻ���ǲ�ֵ����350�ȣ�˵��������������360��0֮��������
				{
					my_ahrs.Angle_Data.yaw = my_MMU.angle.yaw;//ֱ��ʹ�ô���������
				}
				else/*�����ƺ�����������Ǻ���ǲ�ֵС��350�ȣ�˵����������������*/
				{
					my_ahrs.Angle_Data.yaw += (my_MMU.angle.yaw-my_ahrs.Angle_Data.yaw)*0.01f;
				}

			}
		}
		else
		{
			return;
		}
	}
	else
	{
		return;
	}

}

#endif
