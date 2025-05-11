//********************************************************************************************************
//*                                                                                   
//*  �ļ�����LL_flash.h                                                               
//*  �ļ�˵����STM32Gϵ��Ƭ��FLASH����ز���                                                     
//*  ���ߣ����                                                                       
//*  ΢�ţ�LAOLIDESENLIN                                                              
//*  ˵�������ĵ���ѭGNU3.0��Դ���֤�淶��������ɿ�Դ�����ʹ�ã����á��޸ġ���������Ҳ��Ҫ��Դ�����ʹ�ã�
//*        ���������޸ĺ�������Ĵ�����Ϊ��Դ����ҵ�������������
//*
//********************************************************************************************************

#ifndef __STMFLASH_H__
#define __STMFLASH_H__

#include "main.h"


/**************************************************************************************/
/* G431оƬ��128KFLASH������ҳ��ַ�ֲ����£�����1��BANK��64ҳ��ÿһҳ2kb��С */
#define STM32_FLASH_BASE        0x08000000      /* STM32 FLASH ��ʼ��ַ */
#define STM32_FLASH_SIZE        0x20000         /* STM32 FLASH �ܴ�С*/
#define STM32_FLASH_PAGE_SIZE   0x800           /* STM32 FLASH ҳ��С*/

#define FLASH_ADDR_BASE         0x0801f800


#define FLASH_NULL_U8           0xff
#define FLASH_NULL_U16          0xffff
#define FLASH_NULL_U32          0xffffffff
#define FLASH_NULL_U64          0xffffffffffffffff

extern uint64_t write_u64_temp;      //��FLASHд��1������ʱ����Ҫ������ʱת��Ϊ64λ������д��

uint8_t stmflash_erase_page(uint16_t start_page_sn, uint8_t len, uint8_t bank);     //ɾ��FLASH����
uint8_t stmflash_read(uint32_t addr, uint64_t* pdata64, uint32_t len_64);   //��Ƭ��FLASH
uint8_t stmflash_write(uint32_t addr, uint64_t* pdata64, uint32_t len_64);  //��Ƭ��FLASHд������


#endif
