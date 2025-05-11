//********************************************************************************************************
//*                                                                                   
//*  文件名：LL_flash.h                                                               
//*  文件说明：STM32G系列片上FLASH的相关操作                                                     
//*  作者：李佳                                                                       
//*  微信：LAOLIDESENLIN                                                              
//*  说明：本文档遵循GNU3.0开源许可证规范，即代码可开源并免费使用，引用、修改、衍生代码也需要开源、免费使用，
//*        但不允许修改后和衍生的代码做为闭源的商业软件发布和销售
//*
//********************************************************************************************************

#ifndef __STMFLASH_H__
#define __STMFLASH_H__

#include "main.h"


/**************************************************************************************/
/* G431芯片的128KFLASH容量的页地址分布如下，共有1个BANK，64页，每一页2kb大小 */
#define STM32_FLASH_BASE        0x08000000      /* STM32 FLASH 起始地址 */
#define STM32_FLASH_SIZE        0x20000         /* STM32 FLASH 总大小*/
#define STM32_FLASH_PAGE_SIZE   0x800           /* STM32 FLASH 页大小*/

#define FLASH_ADDR_BASE         0x0801f800


#define FLASH_NULL_U8           0xff
#define FLASH_NULL_U16          0xffff
#define FLASH_NULL_U32          0xffffffff
#define FLASH_NULL_U64          0xffffffffffffffff

extern uint64_t write_u64_temp;      //向FLASH写入1个变量时，需要将其临时转换为64位，才能写入

uint8_t stmflash_erase_page(uint16_t start_page_sn, uint8_t len, uint8_t bank);     //删除FLASH扇区
uint8_t stmflash_read(uint32_t addr, uint64_t* pdata64, uint32_t len_64);   //读片上FLASH
uint8_t stmflash_write(uint32_t addr, uint64_t* pdata64, uint32_t len_64);  //向片上FLASH写入数据


#endif
