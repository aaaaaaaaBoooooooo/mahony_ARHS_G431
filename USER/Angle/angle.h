/********************************Copyright (c)**********************************
**
**                   (c) Copyright 2025, Main, China, CD.
**                           All Rights Reserved
**
**                           By(ABO From UESTC)
**                           https://www.uestc.edu.cn
**
**----------------------------------文件信息------------------------------------
** 文件名称: angle.h
** 创建人员: Main
** 创建日期: 2025-04-14
** 文档描述: 
**
********************************End of Head************************************/
 
#ifndef __ANGLE_H_
#define __ANGLE_H_



/*头文�?----------------------------------------------------------------------*/
#include "HANA_math.h"
/*宏定�?-----------------------------------------------------------------------*/
#define RadtoDeg    	57.29578f				//弧度到角�? (弧度 * 180/3.1415926)
#define DegtoRad    	0.0174533f				//角度到弧�? (角度 * 3.1415/180)
#define Acc_Gain  		0.0004883f				//加速度变成G (初始化加速度满量�?-+16g LSBa = 2*16/65536.0)
#define Gyro_Gain 		0.0610352f				//角速度变成度dps (初始化陀螺仪满量�?+-2000 LSBg = 2*2000/65536.0)

#define Gyro_Gr	    	0.0010641f			  	//角速度变成弧度(3.1415/180 * LSBg) 
#define VAR_GyX				1.0f		            //1/方差   0.001f
#define VAR_GyY				1.0f		            //1/方差   0.001f
#define VAR_GyZ				1.0f		            //1/方差   0.001f
#define VAR_AcX				0.00001f		            //1/方差   0.001f
#define VAR_AcY				0.00001f		            //1/方差   0.001f
#define VAR_AcZ				0.00001f		            //1/方差   0.001f
#define ALPHA			0.0f					//一阶互补滤波系�?


#define ANGLE_TO_RAD(x)    ((x) * PI / 180.0)                                   // 角度转换为弧�?
#define RAD_TO_ANGLE(x)    ((x) * 180.0 / PI)                                   // 弧度转换为角�?
#define Dynamic_PI  //使能动态PI参数

#define USE_MMU     //使能磁力计


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
} Quaternion;

typedef struct{
	vector3float_t gyro;// 角速度
	vector3float_t acc;// 加速度
	float temperature;//温度
	vector3float_t mag;// 磁场强度
	vector3float_t gyro_offset;//陀螺仪零偏
	float acc_static_value;//加速度计静态模值（重力加速度�?
	vector3float_t mag_offset;//磁力计零�?
	vector3float_t acc_Scale_K;//加速度计标定系数
	vector3float_t acc_offset;//加速度计零偏
}IMU_DATA_TypeDef;//IMU原始数据

typedef struct{
	float roll;
	float pitch;
	float yaw;
}Angle_DATA_TypeDef;//欧拉角姿态数�?

typedef struct{
	Angle_DATA_TypeDef Angle_Data;
	IMU_DATA_TypeDef IMU_Data;
	uint8_t is_init_success;
}AHRS_TypeDef;

extern  AHRS_TypeDef my_ahrs;

/*函数描述-----------------------------------------------------------*/


/*函数声明*/
void IMU_Calibration(void);
void IMU_DataUpdate(void);
void IMU_DataUpdate_DMA(void);
void IMU_GetAngle(float dt);
#ifdef USE_MMU
/*****ʹ�ô����� *****/
void mmu_angle_update(void);
#endif
/********************************End of File************************************/







#endif
