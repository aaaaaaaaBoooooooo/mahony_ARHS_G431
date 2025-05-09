//********************************************************************************************************
//*                                                                                   
//*  文件名：LL_flash.c                                                               
//*  文件说明：STM32G系列片上FLASH的相关操作                                                     
//*  作者：李佳                                                                       
//*  微信：LAOLIDESENLIN                                                              
//*  说明：本文档遵循GNU3.0开源许可证规范，即代码可开源并免费使用，引用、修改、衍生代码也需要开源、免费使用，
//*        但不允许修改后和衍生的代码做为闭源的商业软件发布和销售
//*
//*
//*
//********************************************************************************************************


#include "stmflash.h"

static uint64_t page_buf[STM32_FLASH_PAGE_SIZE/8];      //2K字节的页缓存，以uint64_t方式定义
uint64_t write_uint64_t_temp=0;

//按页删除片上FLASH数据
//start: 起始页号
//len:  待删除的页的数量
//bank: bank编号
//返回值：错误类型，0表示无错误
uint8_t stmflash_erase_page(uint16_t start_page_sn, uint8_t len, uint8_t bank)
{
  uint32_t err;
  uint8_t ret;
  FLASH_EraseInitTypeDef EraseInitStruct;
  uint8_t flash_count=0;
  
  EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;    //删除方式
  EraseInitStruct.Page        = start_page_sn;            //超始页号
  EraseInitStruct.NbPages     = len;                      //页的数量
  EraseInitStruct.Banks       = bank;                     //bank号
  
  HAL_FLASH_Unlock();         //解锁，以准备进行FLASH操作
  
  for(uint8_t i=0; i<10; i++)      //最多重复10次，如果仍然失败，则返回
  {
    ret = HAL_FLASHEx_Erase(&EraseInitStruct, &err);      //擦除
    if (ret == HAL_OK)
      break;
  }
  
  HAL_FLASH_Lock();           //上锁，以结束FLASH操作
  return ret;
}


//读片上FLASH
//addr: 32位的地址值,该值应当是8的整数倍，因为是按照64位的方式写入数据的，读写应当保持一致
//pdata: 返回的数据首地址
//len: 待读取数据的长度， 长度是按uint64_t的数量
//返回值: 错误类型，0表示无错误
uint8_t stmflash_read(uint32_t addr, uint64_t* pdata64, uint32_t len_64)
{
  uint64_t data;
  
  for(uint32_t i=0; i<len_64; i++)
  {
    data = *(__IO uint64_t *)(addr+i*8);
    pdata64[i] = data;
  }
  return 0;
}


//直接写入FLASH，在写入前不检查FLASH内容是否为空
static uint8_t stmflash_write_without_check(uint32_t addr, uint64_t* pdata64, uint32_t len_64)
{
  uint16_t i,j;
  uint8_t ret;
  uint64_t read;
  
  HAL_FLASH_Unlock();
  
  for(j=0; j<len_64; j++)
  {
    for(i=0; i<10; i++)    //不超过10次的重复操作，以保证写入成功
    {
      ret = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr+j*8, *(pdata64+j));
      if(ret == HAL_OK)
      {
        read = *(__IO uint64_t *)(addr+j*8);  //在MCU认为写入正确以后，再次读取数据，并进行比对，如果比对不成功，说明写入出错
        if(read == *(pdata64+j))
          break;
        else
        {
          ret = 255;
          return ret;
        }
      }
    }
  }
  
  HAL_FLASH_Lock();
  return 0;
}

//向片上FLASH写入数据，每次写入8字节，不够8字节的，以0xFF补足
//addr: 32位的地址值,该值应当是8的整数倍，因为是按照64位的方式读取数据的
//pdata: 待写入的数据首地址
//len: 待写入数据的长度，长度是按uint64_t的数量
//返回值: 错误类型，0表示无错误
//
//注意事项：
//    写入字长小于64位的变量时，必须按如下方式进行，不能直接将uint8_t uint16_t uint32_t变量送到本函数
//    uint64_t write_uint64_t_temp;
//    write_uint64_t_temp = 40000;
//    LL_flash_write(0x0801f800+8*0, &write_uint64_t_temp, 1);
uint8_t stmflash_write(uint32_t addr, uint64_t* pdata64, uint32_t len_64)
{
  uint8_t ret, need_erase=0;
  uint8_t page_sn=0;                               //页号
  uint8_t is_across_page=0;                        //跨页标志
  uint32_t need_write_len;                         //需要写入的数据长度
  uint32_t i,j;
  
  
  if(addr%8 != 0)   //如果地址不是8的倍数，直接返回错误
    return 255;
  
  //待写区域全为空(0xFF)？
  //  Y 无需擦除
  //    need_erase = 0 无需擦除
  //  N 需要擦除
  //    need_erase = 1 需要擦除
  
  //待写区域分布在几个页？
  //  area=1 在1个页内，无跨页操作
  //  area>1 在多个页内，有跨页操作

  //写入流程：
  //若need_erase == 0，则直接写入
  //若need_erase != 0，需要 1 将该页的内容保存到缓冲区
  //                        2 更新缓冲区内的数据，即以待写数据填充缓冲区（2K字节，合256个uint64_t）
  //                        3 将缓冲区的数据写入页
  
  
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //
  //    准备阶段，计算各项准备数据
  //
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  for(i=0; i<len_64; i++)   //检查是否为空值，即0xFFFFFFFFFFFFFFFF
  {
    if(*(__IO uint64_t *)(addr+i*8) != FLASH_NULL_U64)
    {
      need_erase = 1;       //只要有1个字节不为空，则进行擦写操作
      break;
    }
  }
  
  if(need_erase == 0)   //如果无需擦除，直接写入，然后就可以退出了
  {
    ret = stmflash_write_without_check(addr, pdata64, len_64);
    return ret;
  }
  
  if((addr&0xfffff800) == ((addr+len_64*8-1)&0xfffff800))
    is_across_page=0;
  else
    is_across_page=1;
  
  ////////////////////////////////////////
  //
  //  起始页操作
  //    1 将起始页的数据全部读入缓冲区
  //    2 擦除起始页
  //    3 将起始页应该保留的原始数据写入FLASH原始位置
  //
  ////////////////////////////////////////
  if(need_erase)
  {
    //1 将起始页的数据全部读入缓冲区
    page_sn = (addr - STM32_FLASH_BASE) / STM32_FLASH_PAGE_SIZE;

    need_write_len = (addr - STM32_FLASH_BASE - STM32_FLASH_PAGE_SIZE * page_sn)/8;

    ret = stmflash_read(STM32_FLASH_BASE + STM32_FLASH_PAGE_SIZE * page_sn, \
                        page_buf, \
                        256);
    if(ret != HAL_OK)
      return ret;

    //2 擦除起始页
    if(is_across_page == 1)   //待写数据跨页时才擦除，否则不擦除
    {
      ret = stmflash_erase_page(page_sn, 1, 0);
      if(ret != HAL_OK)
        return ret;
    }

    //3 将应该保留的原始数据写回FLASH
    ret = stmflash_write_without_check(STM32_FLASH_BASE + STM32_FLASH_PAGE_SIZE * page_sn,\
                                       page_buf,\
                                       need_write_len);
    if(ret != HAL_OK)
      return ret;
  }

  /////////////////////////////////////////////////////////////////
  //
  //  终止页操作
  //    1 将终止页的数据全部读入缓冲区
  //    2 擦除终止页
  //    3 将终止页应该保留的原始数据写入FLASH原始位置
  /////////////////////////////////////////////////////////////////
  j = (2048-(addr+len_64*8-(STM32_FLASH_BASE+2048*63)))/8;
  if(need_erase == 1)//需要擦除
  {
    for(i=0; i<j; i++)   //检查是否为空值，即0xFFFFFFFFFFFFFFFF
    {
      if(*(__IO uint64_t *)(addr+len_64*8+i*8) != FLASH_NULL_U64)
      {
        need_erase = 1;       //只要有1个字节不为空，则进行擦写操作
        break;
      }
    }
  }


  if((addr+len_64*8-STM32_FLASH_BASE)%STM32_FLASH_PAGE_SIZE != 0)   //不为0，表示结束位置不是页尾，需要进行终止页的数据保存操作
  {
    i = (addr + len_64*8) & 0xfffff800; //终止页的首地址
    j = addr + len_64*8 - i;
    if(is_across_page == 1)   //跨页时的操作
    {
      ret = stmflash_read(i, page_buf, 256);
      if(ret != HAL_OK)
        return ret;
      
      ret = stmflash_erase_page((i-STM32_FLASH_BASE)/STM32_FLASH_PAGE_SIZE, 1, 0);
      if(ret != HAL_OK)
        return ret;

      if(need_erase == 1)
      ret = stmflash_write_without_check(addr+len_64*8, page_buf+j/8, 256-j/8);
    
    }
    else  //不跨页时的操作
    {
      for(i=0; i<256; i++)
        page_buf[i] = i<<8|i;
      if(need_erase == 1)
      {
        //待写区填充0xff
        
        //备份原始数据到页缓冲区
        i = addr & 0xfffff800;    //终止页的页首地址
        ret = stmflash_read(i, page_buf, STM32_FLASH_PAGE_SIZE/8);  //备份待写区全部数据
        
        //将待写数据放进页缓冲区
        j = ((addr - STM32_FLASH_BASE) % STM32_FLASH_PAGE_SIZE) / 8;
        for(i=0; i<len_64; i++)
          page_buf[j+i] = *(pdata64+i);

        
        //擦除页
        stmflash_erase_page(((addr&0xfffff800) - STM32_FLASH_BASE)/STM32_FLASH_PAGE_SIZE, 1, 0);
        
        //将页缓存全部数据写入终止页
        i = addr & 0xfffff800;    //终止页的页首地址
        stmflash_write_without_check(i,page_buf,STM32_FLASH_PAGE_SIZE/8);
      }
    }
    if(ret != HAL_OK)
      return ret;
  }


  /////////////////////////////////////////////////////////////////
  //
  //  至此，如果待写数据不跨页，全部操作完成，
  //  如果待写数据跨页，还未开始数据写入，只是将全部待写区域变为ff，以准备写入
  //
  /////////////////////////////////////////////////////////////////


  /////////////////////////////////////////////////////////////////
  //
  //  开始数据写入
  //    1 将连续的完整页擦除
  //    2 一次性写入全部待写数据
  //
  /////////////////////////////////////////////////////////////////
  
  //1 将连续的完整页擦除
  if((need_erase==1) && (is_across_page==1))    //需要擦除，且数据跨页时，才进行中间页的擦除
  {
    i = len_64 - (256-need_write_len);      //连续页加终止页的数据数量
    j = i - (i % 256);                      //连续页的数据数量
    ret = stmflash_erase_page(page_sn+1, j/256, 0);//擦除连续页的所有数据
    if(ret != HAL_OK)
      return ret;
  }
  
  //    2 一次性写入全部待写数据
  if(is_across_page==1) //如果不跨页，前面已经完成了数据写入
    ret = stmflash_write_without_check(addr, pdata64, len_64);


  return ret;
}

