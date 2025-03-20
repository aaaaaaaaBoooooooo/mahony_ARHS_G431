#ifndef _icm42688_h_
#define _icm42688_h_

#include "main.h"
#include "inv_imu_driver.h"
#include "inv_imu_transport.h"
//====================================================Ӳ�� SPI ����(STM32 HAL��Ӳ��SPI)====================================================                                          
#define IMU1_CS(x)              ((x) ? (HAL_GPIO_WritePin(IMU1_CS_GPIO_Port,IMU1_CS_Pin,GPIO_PIN_SET)) : (HAL_GPIO_WritePin(IMU1_CS_GPIO_Port,IMU1_CS_Pin,GPIO_PIN_RESET)))
#define IMU2_CS(x)              ((x) ? (HAL_GPIO_WritePin(IMU2_CS_GPIO_Port,IMU2_CS_Pin,GPIO_PIN_SET)) : (HAL_GPIO_WritePin(IMU2_CS_GPIO_Port,IMU2_CS_Pin,GPIO_PIN_RESET)))
//================================================���� 4��SPI R/W================================================
#define IMU_SPI_W              (0x00)
#define IMU_SPI_R              (0x80)

/*ICM45686�ṹ��*/
typedef struct 
{
	inv_imu_device_t dev;//����
	inv_imu_sensor_data_t data;//����
	uint8_t is_init_ok;
}ICM45686_TypeDef;
/*�������ݽṹ��*/
typedef struct 
{
	int16_t gyro_x;
	int16_t gyro_y;
	int16_t gyro_z;
	int16_t acc_x;
	int16_t acc_y;
	int16_t acc_z;
	int16_t temp;
}IMUDATA_TypeDef;

int imu_init(ICM45686_TypeDef *imu);
void icm45686_data_update(void);
extern ICM45686_TypeDef myIMU1;
extern ICM45686_TypeDef myIMU2;
extern IMUDATA_TypeDef myIMU_data;
#endif

