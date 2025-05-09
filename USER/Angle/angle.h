/********************************Copyright (c)**********************************`
** 文件名称: angle.h
** 创建人员: aBo
** 创建日期: 2025-1-09
** 文档描述: 陀螺仪姿态解算相关函数声明
********************************End of Head************************************/

 
#ifndef __ANGLE_H_
#define __ANGLE_H_



/*头文件----------------------------------------------------------------------*/
#include "main.h"
/*宏定义-----------------------------------------------------------------------*/
#define RadtoDeg    	57.29578f				//弧度到角度 (弧度 * 180/3.1415926)
#define DegtoRad    	0.0174533f				//角度到弧度 (角度 * 3.1415/180)
#define Acc_Gain  		0.0004883f				//加速度变成G (初始化加速度满量程-+16g			LSB = 2*16/65536.0)
#define Gyro_Gain 		0.0610352f				//角速度变成度dps (初始化陀螺仪满量程+-2000°/s	LSB = 2*2000/65536.0)

#define Gyro_Gr	    	0.0010641f			  	//角速度变成弧度(3.1415/180 * LSBg) 
#define VAR_GyX				0.05f		            //陀螺仪X轴静态方差阈值   
#define VAR_GyY				0.05f		            //陀螺仪Y轴静态方差阈值   
#define VAR_GyZ				0.05f		            //陀螺仪Z轴静态方差阈值   
#define VAR_AcX				0.01f		            //加速度计X轴静态方差阈值   
#define VAR_AcY				0.01f		            //加速度计Y轴静态方差阈值   
#define VAR_AcZ				0.01f		            //加速度计Z轴静态方差阈值


#define Dynamic_PI	//使能动态PI系数
#define SIX_CAL		//使能6面校准
//#define USE_MMU     //使能磁力计


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
} Quaternion;//四元数

typedef struct{
	vector3float_t gyro;// 角速度
	vector3float_t acc;// 加速度
	float temperature;//温度
	vector3float_t mag;// 磁场强度

	vector3float_t gyro_offset;//陀螺仪零偏
	float acc_static_value;//加速度计静态模值（重力加速度）
	vector3float_t acc_Scale_K;//加速度计标定系数
	vector3float_t acc_offset;//加速度计零偏
	vector3float_t mag_offset;//磁力计零偏

	vector3float_t gyro_var;//陀螺仪测量方差
	vector3float_t acc_var;//加速度计测量方差

	
}IMU_DATA_TypeDef;//处理后的IMU数据

typedef struct{
	float roll;
	float pitch;
	float yaw;
}Angle_DATA_TypeDef;//欧拉角姿态数据

typedef struct{
	IMU_DATA_TypeDef IMU_Data;//处理后的IMU数据
	Angle_DATA_TypeDef Angle_Data;//姿态解算欧拉角度数据
	Quaternion NumQ ;//四元数
	uint8_t is_init_success;
}AHRS_TypeDef;//AHRS解算数据

extern  AHRS_TypeDef my_ahrs;


/*函数声明*/
void IMU_Calibration(void);//零漂矫正
void IMU_DataUpdate(void);//数据更新
void IMU_GetAngle(float dt);//姿态解算

#ifdef USE_MMU
/*****使用磁力计解算航向角*****/
void mmu_angle_update(void);
#endif
/********************************End of File************************************/
#endif
