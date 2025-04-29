#include "bmm350_task.h"
#include "bmm350.h"
#include "bmm350_defs.h"
#include "i2c.h"
#include "string.h"
#include "stdio.h"
#include "delay.h"

MMU_TypeDef my_MMU;

void mmu_delay_us(uint32_t period, void *intf_ptr)
{
	delay_us(period);

}

BMM350_INTF_RET_TYPE mmu_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len,void *intf_ptr)
{

	/***此处实现IIC写寄存器***/
	uint8_t daddr; //device address (0x1e<<1)
	uint8_t data_to_send[len+1];
	daddr = BMM350_I2C_ADSEL_SET_LOW<<1;
	data_to_send[0] = reg_addr;
	memcpy(&data_to_send[1],reg_data,len);
	if(HAL_I2C_Master_Transmit(&hi2c2,daddr,data_to_send,len+1,0xFFFF)==HAL_OK)
	{
		return BMM350_INTF_RET_SUCCESS;
	}
	else
	{
		return BMM350_E_INVALID_CONFIG;
	}
}
BMM350_INTF_RET_TYPE mmu_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
	/***此处实现IIC读寄存器***/
	uint8_t daddr;
	daddr = BMM350_I2C_ADSEL_SET_LOW<<1;
	if(HAL_I2C_Master_Transmit(&hi2c2,daddr,&reg_addr,1,0xffff)==HAL_OK)
	{
		if(HAL_I2C_Master_Receive(&hi2c2,daddr+1,reg_data,len,0xffff)==HAL_OK)
		{

			return BMM350_INTF_RET_SUCCESS;
		}
	}
	return BMM350_E_INVALID_CONFIG;


}


/*磁力计初始化*/
int8_t mmu_init()
{
	int8_t rslt;
	uint8_t  int_ctrl, err_reg_data = 0;
//	uint8_t set_int_ctrl;

	struct bmm350_pmu_cmd_status_0 pmu_cmd_stat_0;

	my_MMU.dev.delay_us = mmu_delay_us;//微秒延时函数链接
	my_MMU.dev.write = mmu_i2c_write;//IIC写函数链接
	my_MMU.dev.read = mmu_i2c_read;//IIC读函数链接

	rslt = bmm350_init(&my_MMU.dev);
	if(rslt != BMM350_OK)
	{
		return rslt;
	}
	else
	{
    /* Check PMU busy */
    rslt = bmm350_get_pmu_cmd_status_0(&pmu_cmd_stat_0, &my_MMU.dev);

    printf("Expected : 0x07 : PMU cmd busy : 0x0\n");
    printf("Read : 0x07 : PMU cmd busy : 0x%X\n", pmu_cmd_stat_0.pmu_cmd_busy);

    /* Get error data */
    rslt = bmm350_get_regs(BMM350_REG_ERR_REG, &err_reg_data, 1, &my_MMU.dev);
    printf("Expected : 0x02 : Error Register : 0x0\n");
    printf("Read : 0x02 : Error Register : 0x%X\n", err_reg_data);

//    /* Configure interrupt settings */
//    rslt = bmm350_configure_interrupt(BMM350_PULSED, BMM350_ACTIVE_HIGH, BMM350_INTR_PUSH_PULL, BMM350_MAP_TO_PIN, &my_MMU.dev);

    /* Enable data ready interrupt */
    rslt = bmm350_enable_interrupt(BMM350_ENABLE_INTERRUPT, &my_MMU.dev);

//    /* Get interrupt settings */
//    rslt = bmm350_get_regs(BMM350_REG_INT_CTRL, &int_ctrl, 1, &my_MMU.dev);

//    set_int_ctrl = ((BMM350_INT_POL_ACTIVE_HIGH << 1) | (BMM350_INT_OD_PUSHPULL << 2) | (BMM350_ENABLE << 3) | BMM350_ENABLE << 7);

//    printf("Expected : 0x2E : Interrupt control : 0x%X\n", set_int_ctrl);
//    printf("Read : 0x2E : Interrupt control : 0x%X\n", int_ctrl);

    if (int_ctrl & BMM350_DRDY_DATA_REG_EN_MSK)
    {
        printf("Data ready enabled\r\n");
    }

    /* Set ODR and performance */
    rslt = bmm350_set_odr_performance(BMM350_DATA_RATE_25HZ, BMM350_AVERAGING_8, &my_MMU.dev);

    rslt = bmm350_delay_us(10000, &my_MMU.dev);

    /* Enable all axis */
    rslt = bmm350_enable_axes(BMM350_X_EN, BMM350_Y_EN, BMM350_Z_EN, &my_MMU.dev);
   if (rslt == BMM350_OK)
    {
        rslt = bmm350_set_pad_drive(BMM350_PAD_DRIVE_WEAKEST, &my_MMU.dev);


        rslt = bmm350_set_powermode(BMM350_NORMAL_MODE, &my_MMU.dev);

        printf("\nPower mode is set to normal mode\n");
        printf("Compensated Magnetometer and temperature data with delay\n");

        printf("Timestamp(ms), Mag_X(uT), Mag_Y(uT), Mag_Z(uT), Temperature(degC)\n");

		/***磁力计椭圆校准三轴参数***/
		my_MMU.comp_data.x_max = 49.0f;
		my_MMU.comp_data.x_min = -91.0f;
		my_MMU.comp_data.y_max = 54.0f;
		my_MMU.comp_data.y_min = -99.0f;
		my_MMU.comp_data.z_max = 85.0f;
		my_MMU.comp_data.z_min = -40.0f;

		my_MMU.is_init_ok =1;
		}
			return rslt;
	}



}


/*磁力计数据读取*/
int8_t mmu_data_update()
{
	int8_t rslt;
	uint8_t int_status;

	/* Get data ready interrupt status */
	rslt = bmm350_get_regs(BMM350_REG_INT_STATUS, &int_status, 1, &my_MMU.dev);

	/* Check if data ready interrupt occurred */
	if (int_status & BMM350_DRDY_DATA_REG_MSK)
	{
		rslt = bmm350_get_compensated_mag_xyz_temp_data(&my_MMU.Data, &my_MMU.dev);
		/***椭圆矫正归一化至+-1***/
		my_MMU.Data.x = (2.0f*my_MMU.Data.x - (my_MMU.comp_data.x_max+my_MMU.comp_data.x_min))/
				(my_MMU.comp_data.x_max-my_MMU.comp_data.x_min);
		my_MMU.Data.y = (2.0f*my_MMU.Data.y - (my_MMU.comp_data.y_max+my_MMU.comp_data.y_min))/
				(my_MMU.comp_data.y_max-my_MMU.comp_data.y_min);
		my_MMU.Data.z = (2.0f*my_MMU.Data.z - (my_MMU.comp_data.z_max+my_MMU.comp_data.z_min))/	
				(my_MMU.comp_data.z_max-my_MMU.comp_data.z_min);
		return rslt;
	}	
	else
	{
		return BMM350_E_INVALID_CONFIG;
	}
	

}






































