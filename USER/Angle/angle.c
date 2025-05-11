/********************************Copyright (c)**********************************`
** 文件名称: angle.c
** 创建人员: aBo
** 创建日期: 2025-5-04
** 文档描述: 陀螺仪姿态解算相关函数实现
**基于Mahony四元数的6/9轴IMU姿态解算
  创新性提出基于重力加速度模值变化的动态PI系数
  用于自适应调整在静、动态情景下陀螺仪与加速度计的相信程度
  降低了动态情景下，由于机体加速度影响，加速度计解算姿态的误差
  ******************************************************
  同时基于卡尔曼滤波思想，通过历史数据的斜率，预测当前数据，并与传感器测量值进行融合
  达到在静态情景下降低陀螺仪测量噪声，在动态情景下不损失响应速度的性能
  ******************************************************
  通过对加速度计进行六面校准，获得零偏和测量系数，矫正三轴加速度计轴与轴之间的测量误差
  ******************************************************
  通过额外引入三轴磁力计测量地磁向量，解算后得到绝对航向角度，并与陀螺仪积分得到的航向角
  进行一阶融合滤波，达到降低磁力计测量噪声同时避免了积分漂移的效果
********************************End of Head************************************/
#include "angle.h"
#include "delay.h"
#include "icm45686_task.h"
#include "filter.h"
#include "HANA_math.h"
#include "stdio.h"
#include "bmm350_task.h" 
#include "usart.h"
#define Gyro_NUM	500 // 角速度采样个数  视MCU内存大小修改(由于正态分布具有随机性，不建议少于50)
#define Acc_NUM   100 // 加速度采样个数	视MCU内存大小修改(由于正态分布具有随机性，不建议少于50)

AHRS_TypeDef my_ahrs;//姿态解算数据


/*******************************************************************************
** 函数名称: IMU_Calibration()
** 功能描述: 陀螺仪静态零偏校正以及静态重力加速度获取
** 参数说明: : 
** 返回说明: None
** 创建人员: JLB
** 创建日期: 2023-11-29
**------------------------------------------------------------------------------
** 修改人员:aBo
** 修改日期:2025-5-04
** 修改描述:增加重力加速度六面校准参数初始化，增加四元数初始化
**------------------------------------------------------------------------------
********************************************************************************/
void IMU_Calibration(void)
{
	uint16_t i=0;
	vector3float_t gyro[Gyro_NUM];//角速度计原始数据
	vector3float_t gyro_avg = {0.0f,0.0f,0.0f};//角速度计平均值
	vector3float_t gyro_var = {0.0f,0.0f,0.0f};//存放角速度计方差

	
	vector3float_t acc[Acc_NUM];//加速度计原始数据
	vector3float_t acc_avg = {0.0f,0.0f,0.0f};//加速度计平均值
	vector3float_t acc_var = {0.0f,0.0f,0.0f};//存放加速度计方差

	do
	{
		/****数据清零begin****/
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
		/****数据清零end****/
		
		for(i=0; i<Gyro_NUM; i++)//采集陀螺仪数据计算均值
		{
			/***获取IMU原始角速度数据***/
			icm45686_data_update();

			/***单位转换为°/s***/
			gyro[i].x = myIMU_data.gyro_x*Gyro_Gain;
			gyro[i].y = myIMU_data.gyro_y*Gyro_Gain;
			gyro[i].z = myIMU_data.gyro_z*Gyro_Gain;

			/***累加求和***/
			gyro_avg.x += gyro[i].x;
			gyro_avg.y += gyro[i].y;
			gyro_avg.z += gyro[i].z;

			delay_ms(1);//根据解算频率，设置采样率为1KHz
			
		}
		/***求平均,得到角速度零漂***/
		gyro_avg.x = gyro_avg.x/Gyro_NUM;
		gyro_avg.y = gyro_avg.y/Gyro_NUM;
		gyro_avg.z = gyro_avg.z/Gyro_NUM;

		
		for(i=0;i<Acc_NUM;i++)//采集加速度数据计算均值
		{
			
			/***获取IMU原始加速度数据***/
			icm45686_data_update();

			/***单位转换为 g(m/s^2)***/
			acc[i].x = myIMU_data.acc_x*Acc_Gain;
			acc[i].y = myIMU_data.acc_y*Acc_Gain;
			acc[i].z = myIMU_data.acc_z*Acc_Gain;

			/***累加求和***/
			acc_avg.x += acc[i].x;
			acc_avg.y += acc[i].y;
			acc_avg.z += acc[i].z;

			delay_ms(1);//根据解算频率，设置采样率为1KHz		
		}
		/***求平均,得到加速度静态向量，作为实际重力加速度***/
		acc_avg.x = acc_avg.x/Acc_NUM;
		acc_avg.y = acc_avg.y/Acc_NUM;
		acc_avg.z = acc_avg.z/Acc_NUM;

		
		/***计算方差 *确保校准的时候是静止状态的，同时获得传感器测量噪声水平***/
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
		

		/***判断并保存静止时的零偏***/
		if( gyro_var.x<VAR_GyX && gyro_var.y<VAR_GyY && gyro_var.z<VAR_GyZ&&acc_var.x<VAR_AcX && acc_var.y<VAR_AcY &&acc_var.z<VAR_AcZ)//方差足够小
		{
			/***零漂获取begin***/
			my_ahrs.IMU_Data.gyro_offset.x  = gyro_avg.x;
			my_ahrs.IMU_Data.gyro_offset.y  = gyro_avg.y;
			my_ahrs.IMU_Data.gyro_offset.z  = gyro_avg.z;
			my_ahrs.IMU_Data.acc_static_value		=		1.0f/invSqrt(acc_avg.x*acc_avg.x + acc_avg.y*acc_avg.y +acc_avg.z*acc_avg.z);

			/***零漂获取end***/
			
			printf("gyro_offset:x%f\n  y%f\n  z%f\r\n",my_ahrs.IMU_Data.gyro_offset.x,my_ahrs.IMU_Data.gyro_offset.y,my_ahrs.IMU_Data.gyro_offset.z);
			printf("acc_offset:%f\r\n",my_ahrs.IMU_Data.acc_static_value);
			my_ahrs.is_init_success=1;//零漂校准成功
			/***四元数初始化***/
			my_ahrs.NumQ.q0 = 1.0f;
			my_ahrs.NumQ.q1 = 0.0f;
			my_ahrs.NumQ.q2 = 0.0f;
			my_ahrs.NumQ.q3 = 0.0f;

#ifdef SIX_CAL
			/***加速度计六面校准系数初始化(通过六面校准获得，一般高性能IMU出厂已经校准过，故影响很小)***/
			my_ahrs.IMU_Data.acc_offset.x = -0.013347f;
			my_ahrs.IMU_Data.acc_offset.y = 0.009929f;
			my_ahrs.IMU_Data.acc_offset.z = 0.000814f;
			my_ahrs.IMU_Data.acc_Scale_K.x = 0.995170f;
			my_ahrs.IMU_Data.acc_Scale_K.y = 1.000649f;
			my_ahrs.IMU_Data.acc_Scale_K.z = 0.997839f;	
#endif
			return;
		}
		delay_ms(1000);//不是静止状态，延时等待稳定
	}while(1);

	
}


/*******************************************************************************
** 函数名称: IMU_DataUpdate(void)
** 功能描述: IMU原始数据更新，角速度一阶卡尔曼滤波，IMU加速度数据更新
** 参数说明: : 
** 返回说明: None
** 创建人员: JLB
** 创建日期: 2024-1-20
**------------------------------------------------------------------------------
** 修改人员:aBo
** 修改日期:2025-5-04
** 修改描述:增加角速度计两点外推一维卡尔曼滤波，增加加速度计六面校准接口
**------------------------------------------------------------------------------
********************************************************************************/
void IMU_DataUpdate(void)
{
	/***IMU原始数据更新***/
	icm45686_data_update();

	/***IMU角速度数据更新   单位为°/s***/
	my_ahrs.IMU_Data.gyro.x= myIMU_data.gyro_x*Gyro_Gain-my_ahrs.IMU_Data.gyro_offset.x;
	my_ahrs.IMU_Data.gyro.y= myIMU_data.gyro_y*Gyro_Gain-my_ahrs.IMU_Data.gyro_offset.y;
	my_ahrs.IMU_Data.gyro.z= myIMU_data.gyro_z*Gyro_Gain-my_ahrs.IMU_Data.gyro_offset.z;
	
	/***角速度一阶卡尔曼滤波begin***/
	static  kalman1_state ekf[3] = {{0.0f,0.0f,1.0f,1.0f,0.0005f,0.0f},{0.0f,0.0f,1.0f,1.0f,0.0005f,0.0f},{0.0f,0.0f,1.0f,1.0f,0.0005f,0.0f}};//第五个参数用于调整预测值方差，越大越信任传感器
	static float last_out[3],lastlast_out[3];
	/*****传感器方差更新*****/
	ekf[0].r = my_ahrs.IMU_Data.gyro_var.x; //陀螺仪测量方差
	ekf[1].r = my_ahrs.IMU_Data.gyro_var.y; //陀螺仪测量方差
	ekf[2].r = my_ahrs.IMU_Data.gyro_var.z; //陀螺仪测量方差
	/*****卡尔曼预测系数更新*****/
	ekf[0].A = last_out[0] - lastlast_out[0];//两点外推计算预测斜率
	ekf[1].A = last_out[1] - lastlast_out[1];//两点外推计算预测斜率
	ekf[2].A = last_out[2] - lastlast_out[2];//两点外推计算预测斜率
	/*****卡尔曼滤波输出*****/
	kalman1_filter(&ekf[0],my_ahrs.IMU_Data.gyro.x);  //对x轴角速度一维卡尔曼滤波
	my_ahrs.IMU_Data.gyro.x=ekf[0].x;
	kalman1_filter(&ekf[1],my_ahrs.IMU_Data.gyro.y);  //对Y轴角速度一维卡尔曼滤波
	my_ahrs.IMU_Data.gyro.y=ekf[1].x;
	kalman1_filter(&ekf[2],my_ahrs.IMU_Data.gyro.z);  //对Z轴角速度一维卡尔曼滤波	
	my_ahrs.IMU_Data.gyro.z=ekf[2].x;
	/*****历史数据更新*****/
	lastlast_out[0] = last_out[0];
	last_out[0] = my_ahrs.IMU_Data.gyro.x;
	lastlast_out[1] = last_out[1];
	last_out[1] = my_ahrs.IMU_Data.gyro.y;
	lastlast_out[2] = last_out[2];
	last_out[2] = my_ahrs.IMU_Data.gyro.z;	
	/***角速度一阶卡尔曼滤波end*****/	

  
	/***IMU加速度数据更新   单位为 g(m/s^2) ***/
#ifdef SIX_CAL
	my_ahrs.IMU_Data.acc.x=(myIMU_data.acc_x*Acc_Gain - my_ahrs.IMU_Data.acc_offset.x)*my_ahrs.IMU_Data.acc_Scale_K.x;//IMU加速度数据更新 六面校准(影响很小)      // 单位为 g(m/s^2)
	my_ahrs.IMU_Data.acc.y=(myIMU_data.acc_y*Acc_Gain - my_ahrs.IMU_Data.acc_offset.y)*my_ahrs.IMU_Data.acc_Scale_K.y ;
	my_ahrs.IMU_Data.acc.z=(myIMU_data.acc_z*Acc_Gain - my_ahrs.IMU_Data.acc_offset.z)*my_ahrs.IMU_Data.acc_Scale_K.z ;
#else
	my_ahrs.IMU_Data.acc.x=(myIMU_data.acc_x)*Acc_Gain;
	my_ahrs.IMU_Data.acc.y=(myIMU_data.acc_y)*Acc_Gain;
	my_ahrs.IMU_Data.acc.z=(myIMU_data.acc_z)*Acc_Gain;
#endif
	

	/***IMU温度数据更新   单位为 摄氏度 ***/
	my_ahrs.IMU_Data.temperature = (myIMU_data.temp/128.0f)+25.0f;
	
}
/*******************************************************************************
** 函数名称: GetAngle(float dt) 
** 功能描述: 基于Mahony四元数互补滤波计算角度信息
** 参数说明:  
					dt:采样时间 单位：s				(推荐采样解算频率：500Hz以上，解算频率一定要稳定,采样时间dt一定要准确)
** 返回说明: None
** 创建人员: JLB
** 创建日期: 2024-3-16
**------------------------------------------------------------------------------
** 修改人员:ABO
** 修改日期:2025-5-04
** 修改描述:优化了Mahony算法解算流程，添加了动态PI系数，增加磁力计航向角互补融合
**------------------------------------------------------------------------------
********************************************************************************/
void IMU_GetAngle(float dt) 
{		
	volatile struct V{
				float x;
				float y;
				float z;
				} halfGravity,//角速度通过四元数旋转矩阵获得的理论重力加速度向量
					Acc,//实际加速度计测量的重力加速度向量（静态时准确，动态时不准）
					Gyro,//经加速度修正后的角速度
					AccGravity;//重力加速度的实际值与理论值叉乘之后的模值，代表误差

	static struct V GyroIntegError = {0};//误差积分补偿项
	float q0_t,q1_t,q2_t,q3_t;//龙格库塔法暂存变量	
	float NormQuat; //归一化系数
	float HalfTime = dt * 0.5f;//半采样时间->减少乘法次数
	float INV_TAU = 0.15f;//Mahony滤波PI控制器时间常数的倒数
	float twoKpDef = 8.584f *INV_TAU ;//角速度和加速度融合滤波系数  控制对加速度计的相信程度，一般动态下加速度计不可信
	float twoKiDef = 9.210632f*INV_TAU*INV_TAU*dt;//误差积分融合滤波系数   一般做了零漂矫正就不需要积分了（或者一个极小量）


/*****************Mahony融合滤波迭代算法begin********************/

    /*** 只在加速度计数据有效时才进行误差运算 ***/     
	if(!((my_ahrs.IMU_Data.acc.x == 0.0f) && (my_ahrs.IMU_Data.acc.y == 0.0f) && (my_ahrs.IMU_Data.acc.z == 0.0f))) 
	{
		/***通过四元数得到理论重力加速度向量Gravity的一半----减少后续计算中的乘法次数***/ 
		halfGravity.x = (my_ahrs.NumQ.q1 * my_ahrs.NumQ.q3 - my_ahrs.NumQ.q0 * my_ahrs.NumQ.q2);								
		halfGravity.y = (my_ahrs.NumQ.q0 * my_ahrs.NumQ.q1 + my_ahrs.NumQ.q2 * my_ahrs.NumQ.q3);						  
		halfGravity.z = 0.5f-(my_ahrs.NumQ.q1 * my_ahrs.NumQ.q1 + my_ahrs.NumQ.q2 * my_ahrs.NumQ.q2);

		/***将加速度计得到的实际重力加速度向量Acc归一化***/
		NormQuat = invSqrt(sq(my_ahrs.IMU_Data.acc.x)+ sq(my_ahrs.IMU_Data.acc.y) +sq(my_ahrs.IMU_Data.acc.z));
		Acc.x = my_ahrs.IMU_Data.acc.x * NormQuat;
		Acc.y = my_ahrs.IMU_Data.acc.y * NormQuat;
		Acc.z = my_ahrs.IMU_Data.acc.z * NormQuat;	
		
#ifdef Dynamic_PI
		/***动态PI系数***/
		float now_G = 1.0f/NormQuat;
		INV_TAU *= expf(-fabs(now_G - my_ahrs.IMU_Data.acc_static_value)/my_ahrs.IMU_Data.acc_static_value);
		twoKiDef = 9.210632f*INV_TAU*INV_TAU*dt;
		twoKpDef = 8.584f *INV_TAU ;
#endif

		/***对实际重力加速度向量Acc与理论重力加速度向量Gravity做外积得到二者的误差***/
		AccGravity.x = (Acc.y * halfGravity.z - Acc.z * halfGravity.y);
		AccGravity.y = (Acc.z * halfGravity.x - Acc.x * halfGravity.z);
		AccGravity.z = (Acc.x * halfGravity.y - Acc.y * halfGravity.x);
		
		/***对误差进行积分***/
		GyroIntegError.x += AccGravity.x * twoKiDef;
		GyroIntegError.y += AccGravity.y * twoKiDef;
		GyroIntegError.z += AccGravity.z * twoKiDef;
		
		/***角速度融合加速度误差进行融合滤波***/
		Gyro.x = my_ahrs.IMU_Data.gyro.x * DegtoRad + twoKpDef * AccGravity.x  +  GyroIntegError.x;//弧度制
		Gyro.y = my_ahrs.IMU_Data.gyro.y * DegtoRad + twoKpDef * AccGravity.y  +  GyroIntegError.y;
		Gyro.z = my_ahrs.IMU_Data.gyro.z * DegtoRad + twoKpDef * AccGravity.z  +  GyroIntegError.z;		
	}

	/***一阶龙格库塔法求解微分方程, 更新四元数***/
	q0_t = (-my_ahrs.NumQ.q1*Gyro.x - my_ahrs.NumQ.q2*Gyro.y - my_ahrs.NumQ.q3*Gyro.z) * HalfTime;
	q1_t = ( my_ahrs.NumQ.q0*Gyro.x - my_ahrs.NumQ.q3*Gyro.y + my_ahrs.NumQ.q2*Gyro.z) * HalfTime;
	q2_t = ( my_ahrs.NumQ.q3*Gyro.x + my_ahrs.NumQ.q0*Gyro.y - my_ahrs.NumQ.q1*Gyro.z) * HalfTime;
	q3_t = (-my_ahrs.NumQ.q2*Gyro.x + my_ahrs.NumQ.q1*Gyro.y + my_ahrs.NumQ.q0*Gyro.z) * HalfTime;
	my_ahrs.NumQ.q0 += q0_t;
	my_ahrs.NumQ.q1 += q1_t;
	my_ahrs.NumQ.q2 += q2_t;
	my_ahrs.NumQ.q3 += q3_t;
	
	/***单位化四元数 保证四元数在迭代过程中保持单位性质***/
	NormQuat = invSqrt(sq(my_ahrs.NumQ.q0) + sq(my_ahrs.NumQ.q1) + sq(my_ahrs.NumQ.q2) + sq(my_ahrs.NumQ.q3));
	my_ahrs.NumQ.q0 *= NormQuat;
	my_ahrs.NumQ.q1 *= NormQuat;
	my_ahrs.NumQ.q2 *= NormQuat;
	my_ahrs.NumQ.q3 *= NormQuat;	

/*****************Mahony融合滤波迭代算法end********************/

/*****************姿态欧拉角获取begin*******************************/
	/***计算四元数旋转矩阵分量***/
	float R31 = 2.0f*(my_ahrs.NumQ.q1 * my_ahrs.NumQ.q3 - my_ahrs.NumQ.q0 *my_ahrs.NumQ.q2 ) ;/*矩阵(3,1)项*/
	float R32 = 2.0f*(my_ahrs.NumQ.q2 *my_ahrs.NumQ.q3 + my_ahrs.NumQ.q0 * my_ahrs.NumQ.q1);/*矩阵(3,2)项*/
	float R33 = 1.0f - 2.0f * ((my_ahrs.NumQ.q1 *my_ahrs.NumQ.q1) + (my_ahrs.NumQ.q2 * my_ahrs.NumQ.q2));	/*矩阵(3,3)项*/		
//	float R11 = 1.0f - 2.0f *((my_ahrs.NumQ.q2 *my_ahrs.NumQ.q2) + (my_ahrs.NumQ.q3 * my_ahrs.NumQ.q3));	/*矩阵(1,1)项*/		
//	float R21 = 2.0f*(my_ahrs.NumQ.q1 * my_ahrs.NumQ.q2 - my_ahrs.NumQ.q0 *my_ahrs.NumQ.q3);/*矩阵(2,1)项*/		

	my_ahrs.Angle_Data.pitch  =  atan2f(-R31,sqrtf(R32*R32+R33*R33))* RadtoDeg;	 //俯仰角					
	my_ahrs.Angle_Data.roll	= atan2f(R32,R33) * RadtoDeg;	//横滚角
	//Angle_Data.yaw = atan2f(R21,R11) * RadtoDeg;	//偏航角 
#ifdef USE_MMU
static uint8_t mmu_yaw_init_flag = 0; //磁力计初始化标志位
		/*****使用磁力计 *****/	
		if(my_at_cmd.mmu_mode)
		{
			if(my_MMU.is_init_ok==1&&my_MMU.angle.yaw!=0.0f&&mmu_yaw_init_flag==0)//磁力计初始化成功
			{
				my_ahrs.Angle_Data.yaw = my_MMU.angle.yaw; //使用磁力计的偏航角初始化航向
				mmu_yaw_init_flag = 1;
			}
			my_ahrs.Angle_Data.yaw  += Gyro.z*RadtoDeg* dt;//角速度积分成偏航角
			/***偏航角映射为0~360与磁力计兼容***/
			if(my_ahrs.Angle_Data.yaw>360.0f)
					my_ahrs.Angle_Data.yaw -= 360.0f;
			else if(my_ahrs.Angle_Data.yaw<0.0f)
					my_ahrs.Angle_Data.yaw += 360.0f;
		}
		else
		{
			if(fabs(Gyro.z) > 0.01f) //数据太小可以认为是干扰，不是偏航动作
			{
				/*****不使用磁力计*****/
				my_ahrs.Angle_Data.yaw  += Gyro.z*RadtoDeg* dt;//角速度积分成偏航角		
		
			} 			
		}

#else
	if(fabs(Gyro.z) > 0.01f) //数据太小可以认为是干扰，不是偏航动作
	{
		/*****不使用磁力计*****/
		my_ahrs.Angle_Data.yaw  += Gyro.z*RadtoDeg* dt;//角速度积分成偏航角		

	} 
#endif
/*****************姿态欧拉角获取end*******************************/	
}


#ifdef USE_MMU
/*****使用磁力计 *****/
void mmu_angle_update()
{
	float Mzx = 0.0f, Mzy = 0.0f;
	uint8_t rslt = 0;
	if(my_at_cmd.mmu_mode)
	{
		/*磁力计数据更新*/
		rslt = mmu_data_update();
		if(rslt==BMM350_OK)
		{
			/*磁力计水平偏转补偿*/
			Mzx = cosf(my_ahrs.Angle_Data.roll*DegtoRad)*my_MMU.Data.x 
						+ sinf(my_ahrs.Angle_Data.roll*DegtoRad)*sinf(my_ahrs.Angle_Data.pitch*DegtoRad)*my_MMU.Data.y
						+ sinf(my_ahrs.Angle_Data.roll*DegtoRad)*cosf(my_ahrs.Angle_Data.pitch*DegtoRad)*my_MMU.Data.z;
			
			Mzy = cosf(my_ahrs.Angle_Data.pitch*DegtoRad)*my_MMU.Data.y - sinf(my_ahrs.Angle_Data.pitch*DegtoRad)*my_MMU.Data.z;
			/*磁力计航向角计算 0~360*/
			my_MMU.angle.yaw = 180.0f+atan2f(Mzy,Mzx)*RadtoDeg;
			/*磁力计航向角与陀螺仪航向角融合互补滤波*/
			if(fabs(my_ahrs.Angle_Data.roll)<10.0f&&fabs(my_ahrs.Angle_Data.pitch)<10.0f)//水平状态下融合
			{
				if(fabs(my_MMU.angle.yaw-my_ahrs.Angle_Data.yaw)>350.0f)//如果磁力计航向角与陀螺仪航向角差值大于350度，说明磁力计数据在360和0之间跳变了
				{
					my_ahrs.Angle_Data.yaw = my_MMU.angle.yaw;//直接使用磁力计数据
				}
				else/*磁力计航向角与陀螺仪航向角差值小于350度，说明磁力计数据正常*/
				{
					my_ahrs.Angle_Data.yaw += (my_MMU.angle.yaw-my_ahrs.Angle_Data.yaw)*0.1f;
				}
			}
			else//倾斜状态下融合
			{
				if(fabs(my_MMU.angle.yaw-my_ahrs.Angle_Data.yaw)>350.0f)//如果磁力计航向角与陀螺仪航向角差值大于350度，说明磁力计数据在360和0之间跳变了
				{
					my_ahrs.Angle_Data.yaw = my_MMU.angle.yaw;//直接使用磁力计数据
				}
				else/*磁力计航向角与陀螺仪航向角差值小于350度，说明磁力计数据正常*/
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
