/********************************Copyright (c)**********************************`
** �ļ�����: angle.h
** ������Ա: aBo
** ��������: 2025-1-09
** �ĵ�����: ��������̬������غ�������
********************************End of Head************************************/

 
#ifndef __ANGLE_H_
#define __ANGLE_H_



/*ͷ�ļ�----------------------------------------------------------------------*/
#include "main.h"
/*�궨��-----------------------------------------------------------------------*/
#define RadtoDeg    	57.29578f				//���ȵ��Ƕ� (���� * 180/3.1415926)
#define DegtoRad    	0.0174533f				//�Ƕȵ����� (�Ƕ� * 3.1415/180)
#define Acc_Gain  		0.0004883f				//���ٶȱ��G (��ʼ�����ٶ�������-+16g			LSB = 2*16/65536.0)
#define Gyro_Gain 		0.0610352f				//���ٶȱ�ɶ�dps (��ʼ��������������+-2000��/s	LSB = 2*2000/65536.0)

#define Gyro_Gr	    	0.0010641f			  	//���ٶȱ�ɻ���(3.1415/180 * LSBg) 
#define VAR_GyX				0.05f		            //������X�ᾲ̬������ֵ   
#define VAR_GyY				0.05f		            //������Y�ᾲ̬������ֵ   
#define VAR_GyZ				0.05f		            //������Z�ᾲ̬������ֵ   
#define VAR_AcX				0.01f		            //���ٶȼ�X�ᾲ̬������ֵ   
#define VAR_AcY				0.01f		            //���ٶȼ�Y�ᾲ̬������ֵ   
#define VAR_AcZ				0.01f		            //���ٶȼ�Z�ᾲ̬������ֵ


#define Dynamic_PI	//ʹ�ܶ�̬PIϵ��
#define SIX_CAL		//ʹ��6��У׼
//#define USE_MMU     //ʹ�ܴ�����


typedef struct
{
	float x;
	float y;
	float z;
}vector3float_t;

typedef struct
{
	int16_t x;
	int16_t y;
	int16_t z;
}vector3int16_t;

typedef volatile struct {
  float q0;
  float q1;
  float q2;
  float q3;
} Quaternion;//��Ԫ��

typedef struct{
	vector3float_t gyro;// ���ٶ�
	vector3float_t acc;// ���ٶ�
	float temperature;//�¶�
	vector3float_t mag;// �ų�ǿ��

	vector3float_t gyro_offset;//��������ƫ
	float acc_static_value;//���ٶȼƾ�̬ģֵ���������ٶȣ�
	vector3float_t acc_Scale_K;//���ٶȼƱ궨ϵ��
	vector3float_t acc_offset;//���ٶȼ���ƫ
	vector3float_t mag_offset;//��������ƫ

	vector3float_t gyro_var;//�����ǲ�������
	vector3float_t acc_var;//���ٶȼƲ�������

	
}IMU_DATA_TypeDef;//������IMU����

typedef struct{
	float roll;
	float pitch;
	float yaw;
}Angle_DATA_TypeDef;//ŷ������̬����

typedef struct{
	IMU_DATA_TypeDef IMU_Data;//������IMU����
	Angle_DATA_TypeDef Angle_Data;//��̬����ŷ���Ƕ�����
	Quaternion NumQ ;//��Ԫ��
	uint8_t is_init_success;
}AHRS_TypeDef;//AHRS��������

extern  AHRS_TypeDef my_ahrs;


/*��������*/
void IMU_Calibration(void);//��Ư����
void IMU_DataUpdate(void);//���ݸ���
void IMU_GetAngle(float dt);//��̬����

#ifdef USE_MMU
/*****ʹ�ô����ƽ��㺽���*****/
void mmu_angle_update(void);
#endif
/********************************End of File************************************/
#endif
