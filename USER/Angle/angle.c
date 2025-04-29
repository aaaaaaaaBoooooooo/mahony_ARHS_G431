/********************************Copyright (c)**********************************`
** 文件名称: angle.c
** 创建人员: aBo
** 创建日期: 2025-1-09
** 文档描述: 陀螺仪姿态解算
** 解算相关函数
********************************End of Head************************************/
#include "angle.h"
#include "delay.h"
#include "icm45686_task.h"
#include "filter.h"
#include "HANA_math.h"
#include "stdio.h"
#include "bmm350_task.h" 

#define Gyro_NUM	500 // 角速度采样个数  视MCU内存大小修改(由于正态分布具有随机性，不建议少于50)
#define Acc_NUM   100 // 加速度采样个数	视MCU内存大小修改(由于正态分布具有随机性，不建议少于50)

AHRS_TypeDef my_ahrs;//姿态解算数据

/*******************************************************************************
** 函数名称: IMU_Calibration()
** 功能描述: 零偏校正，取20组数据
** 参数说明: : 
** 返回说明: None
** 创建人员: JLB
** 创建日期: 2023-11-29
**------------------------------------------------------------------------------
** 修改人员:
** 修改日期:
** 修改描述:
**------------------------------------------------------------------------------
********************************************************************************/
void IMU_Calibration(void)
{
	uint16_t i=0;
	vector3float_t gyro[Gyro_NUM];//角速度计原始数据
	vector3float_t gyro_avg = {0.0f,0.0f,0.0f};//角速度计平均值
	vector3float_t gyro_var 		 = {0.0f,0.0f,0.0f};//存放角速度计方差

	
	vector3float_t acc[Acc_NUM];//加速度计原始数据
	vector3float_t acc_avg  = {0.0f,0.0f,0.0f};//加速度计平均值
	vector3float_t acc_var      = {0.0f,0.0f,0.0f};//存放加速度计方差

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

			icm45686_data_update();// 获取陀螺仪数据	  单位为°/s

			gyro[i].x = myIMU_data.gyro_x*Gyro_Gain;
			gyro[i].y = myIMU_data.gyro_y*Gyro_Gain;
			gyro[i].z = myIMU_data.gyro_z*Gyro_Gain;

			//累加求和
			gyro_avg.x += gyro[i].x;
			gyro_avg.y += gyro[i].y;
			gyro_avg.z += gyro[i].z;

			delay_ms(1);
			
		}
		//求平均
		gyro_avg.x = gyro_avg.x/Gyro_NUM;
		gyro_avg.y = gyro_avg.y/Gyro_NUM;
		gyro_avg.z = gyro_avg.z/Gyro_NUM;

		
		for(i=0;i<Acc_NUM;i++)//采集加速度数据计算均值
		{

			icm45686_data_update();// 获取加速度计数据	  // 单位为 g(m/s^2)

			acc[i].x = myIMU_data.acc_x*Acc_Gain;
			acc[i].y = myIMU_data.acc_y*Acc_Gain;
			acc[i].z = myIMU_data.acc_z*Acc_Gain;

			//累加求和
			acc_avg.x += acc[i].x;
			acc_avg.y += acc[i].y;
			acc_avg.z += acc[i].z;

			delay_ms(1);			
		}
		//求平均
		acc_avg.x = acc_avg.x/Acc_NUM;
		acc_avg.y = acc_avg.y/Acc_NUM;
		acc_avg.z = acc_avg.z/Acc_NUM;

		
		//计算方差 *确保校准的时候是静止状态的（零偏）
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

		printf("gyro_var\nx:%f\ny:%f\nz:%f\n",gyro_var.x,gyro_var.y,gyro_var.z);
		printf("acc_var\nx:%f\ny:%f\nz:%f\n",acc_var.x,acc_var.y,acc_var.z);
		

		//判断并保存静止时的零偏
		if( gyro_var.x<VAR_GyX && gyro_var.y<VAR_GyY && gyro_var.z<VAR_GyZ&&acc_var.x<VAR_AcX && acc_var.y<VAR_AcY &&acc_var.z<VAR_AcZ)//方差足够小
		{
			/***零漂获取begin***/
			my_ahrs.IMU_Data.gyro_offset.x  = gyro_avg.x;
			my_ahrs.IMU_Data.gyro_offset.y  = gyro_avg.y;
			my_ahrs.IMU_Data.gyro_offset.z  = gyro_avg.z;
			my_ahrs.IMU_Data.acc_static_value		=		1.0f/invSqrt(acc_avg.x*acc_avg.x + acc_avg.y*acc_avg.y +acc_avg.z*acc_avg.z);

			/***零漂获取end***/
			
			printf("gyro_offset:x%f\n  y%f\n  z%f\r\n",my_ahrs.IMU_Data.gyro_offset.x,my_ahrs.IMU_Data.gyro_offset.y,my_ahrs.IMU_Data.gyro_offset.z);
			printf("acc_static_value:%f\r\n",my_ahrs.IMU_Data.acc_static_value);

			/***加速度计六面校准系数初始化***/
			my_ahrs.IMU_Data.acc_offset.x = -0.013347f;
			my_ahrs.IMU_Data.acc_offset.y = 0.009929f;
			my_ahrs.IMU_Data.acc_offset.z = 0.000814f;
			my_ahrs.IMU_Data.acc_Scale_K.x = 0.995170f;
			my_ahrs.IMU_Data.acc_Scale_K.y = 1.000649f;
			my_ahrs.IMU_Data.acc_Scale_K.z = 0.997839f;	

			my_ahrs.is_init_success=1;//零漂校准成功
			return;
		}

	}while(1);

	
}

/*******************************************************************************
** 函数名称: icm42688_Update()
** 功能描述: 零偏校正，取20组数据
** 参数说明: : 
** 返回说明: None
** 创建人员: JLB
** 创建日期: 2024-1-20
**------------------------------------------------------------------------------
** 修改人员:
** 修改日期:
** 修改描述:
**------------------------------------------------------------------------------
********************************************************************************/

void IMU_DataUpdate(void)
{
//	volatile vector3float_t gyro_trans={0,0,0};
//	volatile vector3float_t gyro_temp = {0,0,0};
//	volatile vector3float_t acc_trans={0,0,0};
//	volatile vector3float_t acc_temp = {0,0,0};
//	int16_t acc_LPF_in[3],acc_LPF_out[3];

	
	icm45686_data_update();//获取IMU数据
	
//	static  kalman_1_struct ekf[3] = {{0.02,0,0,0,0.001,0.543},{0.02,0,0,0,0.001,0.543},{0.02,0,0,0,0.001,0.543}};
//	
//	kalman_1(&ekf[0],(float)icm42688_gyro_x);  //对x轴角速度一维卡尔曼滤波
//	gyro_temp.x=(short)ekf[0].out;
//	gyro_temp.x=(gyro_temp.x/10)*10;//降低角速度值的精确度来获得相对稳定的积分值
//  
//	kalman_1(&ekf[1],(float)icm42688_gyro_y);  //对y轴角速度一维卡尔曼滤波
//	gyro_temp.y=(short)ekf[1].out;
//	gyro_temp.y=(gyro_temp.y/10)*10;//降低角速度值的精确度来获得相对稳定的积分值
//  
//	kalman_1(&ekf[2],(float)icm42688_gyro_z);  //对Z轴角速度一维卡尔曼滤波
//	gyro_temp.z=(short)ekf[2].out;
//	gyro_temp.z=(gyro_temp.z/10)*10;//降低角速度值的精确度来获得相对稳定的积分值
  
		
		
	my_ahrs.IMU_Data.gyro.x= myIMU_data.gyro_x*Gyro_Gain-my_ahrs.IMU_Data.gyro_offset.x;//IMU角速度数据更新+去零偏   单位为°/s
	my_ahrs.IMU_Data.gyro.y= myIMU_data.gyro_y*Gyro_Gain-my_ahrs.IMU_Data.gyro_offset.y;
	my_ahrs.IMU_Data.gyro.z= myIMU_data.gyro_z*Gyro_Gain-my_ahrs.IMU_Data.gyro_offset.z;
	
	
//	static Filter_LPF_1 LPF1[3]={{100,0,10},{100,0,10},{8330,0,10}};//加速度收敛
//	acc_LPF_in[0] = icm42688_acc_x;
//	acc_LPF_in[1] = icm42688_acc_y;
//	acc_LPF_in[2] = icm42688_acc_z;
//	for(int i=0;i<3;i++)//低通滤波处理
//	{
//		LPF1[i].new_data = acc_LPF_in[i];
//		acc_LPF_out[i] = (int16_t)LPF_1_Filter_2(&LPF1[i],0.005f);
//		LPF1[i].old_data = acc_LPF_in[i];
//	}
	
	my_ahrs.IMU_Data.acc.x=(myIMU_data.acc_x*Acc_Gain - my_ahrs.IMU_Data.acc_offset.x)*my_ahrs.IMU_Data.acc_Scale_K.x;//IMU加速度数据更新 六面校准(影响很小)      // 单位为 g(m/s^2)
	my_ahrs.IMU_Data.acc.y=(myIMU_data.acc_y*Acc_Gain - my_ahrs.IMU_Data.acc_offset.y)*my_ahrs.IMU_Data.acc_Scale_K.y ;
	my_ahrs.IMU_Data.acc.z=(myIMU_data.acc_z*Acc_Gain - my_ahrs.IMU_Data.acc_offset.z)*my_ahrs.IMU_Data.acc_Scale_K.z ;

	my_ahrs.IMU_Data.temperature = myIMU_data.temp*1.0f;
	
}
/*******************************************************************************
** 函数名称: GetAngle(const _IMU_st *pImu,_Angle_st *pAngle, float dt) 
** 功能描述: 计算车身角度信息
** 参数说明:  
					pImu:陀螺仪原始数据
					pAngle:角度数据
					dt:采样时间 单位：s
** 返回说明: None
** 创建人员: JLB
** 创建日期: 2024-3-16
**------------------------------------------------------------------------------
** 修改人员:ABO
** 修改日期:2025-4-12
** 修改描述:优化了Mahony算法解算流程
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
	static Quaternion NumQ = {1, 0, 0, 0};//四元数
	float q0_t,q1_t,q2_t,q3_t;//龙格库塔法暂存变量	
	float NormQuat; //归一化系数
	float HalfTime = dt * 0.5f;//半采样时间->减少乘法次数
	float INV_TAU = 0.3f;//Mahony滤波PI控制器时间常数的倒数
	float twoKpDef = 8.584f *INV_TAU ;//角速度和加速度融合滤波系数  控制对加速度计的相信程度，一般动态下加速度计不可信
	float twoKiDef = 9.210632f*INV_TAU*INV_TAU*dt;//误差积分融合滤波系数   一般做了零漂矫正就不需要积分了（或者一个极小量）


/*****************Mahony融合滤波迭代算法begin********************/

    /*** 只在加速度计数据有效时才进行误差运算 ***/     
	if(!((my_ahrs.IMU_Data.acc.x == 0.0f) && (my_ahrs.IMU_Data.acc.y == 0.0f) && (my_ahrs.IMU_Data.acc.z == 0.0f))) 
	{
		/***通过四元数得到理论重力加速度向量Gravity的一半----减少后续计算中的乘法次数***/ 
		halfGravity.x = (NumQ.q1 * NumQ.q3 - NumQ.q0 * NumQ.q2);								
		halfGravity.y = (NumQ.q0 * NumQ.q1 + NumQ.q2 * NumQ.q3);						  
		halfGravity.z = 0.5f-(NumQ.q1 * NumQ.q1 + NumQ.q2 * NumQ.q2);

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
	q0_t = (-NumQ.q1*Gyro.x - NumQ.q2*Gyro.y - NumQ.q3*Gyro.z) * HalfTime;
	q1_t = ( NumQ.q0*Gyro.x - NumQ.q3*Gyro.y + NumQ.q2*Gyro.z) * HalfTime;
	q2_t = ( NumQ.q3*Gyro.x + NumQ.q0*Gyro.y - NumQ.q1*Gyro.z) * HalfTime;
	q3_t = (-NumQ.q2*Gyro.x + NumQ.q1*Gyro.y + NumQ.q0*Gyro.z) * HalfTime;
	NumQ.q0 += q0_t;
	NumQ.q1 += q1_t;
	NumQ.q2 += q2_t;
	NumQ.q3 += q3_t;
	
	/***单位化四元数 保证四元数在迭代过程中保持单位性质***/
	NormQuat = invSqrt(sq(NumQ.q0) + sq(NumQ.q1) + sq(NumQ.q2) + sq(NumQ.q3));
	NumQ.q0 *= NormQuat;
	NumQ.q1 *= NormQuat;
	NumQ.q2 *= NormQuat;
	NumQ.q3 *= NormQuat;	

/*****************Mahony融合滤波迭代算法end********************/

/*****************姿态欧拉角获取begin*******************************/
	/***计算四元数旋转矩阵分量***/
	float R31 = 2.0f*(NumQ.q1 * NumQ.q3 - NumQ.q0 *NumQ.q2 ) ;/*矩阵(3,1)项*/
	float R32 = 2.0f*(NumQ.q2 *NumQ.q3 + NumQ.q0 * NumQ.q1);/*矩阵(3,2)项*/
	float R33 = 1.0f - 2.0f * ((NumQ.q1 *NumQ.q1) + (NumQ.q2 * NumQ.q2));	/*矩阵(3,3)项*/		
	float R11 = 1.0f - 2.0f *((NumQ.q2 *NumQ.q2) + (NumQ.q3 * NumQ.q3));	/*矩阵(1,1)项*/		
	float R21 = 2.0f*(NumQ.q1 * NumQ.q2 - NumQ.q0 *NumQ.q3);/*矩阵(2,1)项*/		

	my_ahrs.Angle_Data.pitch  =  atan2f(-R31,sqrtf(R32*R32+R33*R33))* RadtoDeg;	 //俯仰角					
	my_ahrs.Angle_Data.roll	= atan2f(R32,R33) * RadtoDeg;	//横滚角
	//Angle_Data.yaw = atan2f(R21,R11) * RadtoDeg;	//偏航角 
#ifdef USE_MMU
static uint8_t mmu_yaw_init_flag = 0; //磁力计初始化标志位
		/*****使用磁力计 *****/	
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

#else
	if(fabs(Gyro.z) > 0.01f) //数据太小可以认为是干扰，不是偏航动作
	{
		/*****不使用磁力计*****/
		my_ahrs.Angle_Data.yaw  += Gyro.z*RadtoDeg* dt;//角速度积分成偏航角		
		/***偏航角映射为0~360与磁力计兼容***/
		if(my_ahrs.Angle_Data.yaw>360.0f)
				my_ahrs.Angle_Data.yaw -= 360.0f;
		else if(my_ahrs.Angle_Data.yaw<0.0f)
				my_ahrs.Angle_Data.yaw += 360.0f;


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

#endif
