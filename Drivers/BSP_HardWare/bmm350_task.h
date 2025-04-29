#ifndef _BMM350_TASK_H_
#define _BMM350_TASK_H_
#include "main.h"
#include "bmm350_defs.h"

typedef struct
{
    float   roll;
    float   pitch;
    float   yaw;
}angle_Typedef;

typedef struct
{
    float   x_max;
    float   x_min;
    float   y_max;
    float   y_min;
    float   z_max;
    float   z_min;
}comp_Typedef;

typedef struct
{
    struct bmm350_dev dev;
    struct bmm350_mag_temp_data Data;
    comp_Typedef comp_data;
    angle_Typedef angle;

    uint8_t is_init_ok;
}MMU_TypeDef;

extern MMU_TypeDef my_MMU;
int8_t mmu_init(void);
int8_t mmu_data_update(void);
#endif

