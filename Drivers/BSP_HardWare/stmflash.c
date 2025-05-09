//********************************************************************************************************
//*                                                                                   
//*  �ļ�����LL_flash.c                                                               
//*  �ļ�˵����STM32Gϵ��Ƭ��FLASH����ز���                                                     
//*  ���ߣ����                                                                       
//*  ΢�ţ�LAOLIDESENLIN                                                              
//*  ˵�������ĵ���ѭGNU3.0��Դ���֤�淶��������ɿ�Դ�����ʹ�ã����á��޸ġ���������Ҳ��Ҫ��Դ�����ʹ�ã�
//*        ���������޸ĺ�������Ĵ�����Ϊ��Դ����ҵ�������������
//*
//*
//*
//********************************************************************************************************


#include "stmflash.h"

static uint64_t page_buf[STM32_FLASH_PAGE_SIZE/8];      //2K�ֽڵ�ҳ���棬��uint64_t��ʽ����
uint64_t write_uint64_t_temp=0;

//��ҳɾ��Ƭ��FLASH����
//start: ��ʼҳ��
//len:  ��ɾ����ҳ������
//bank: bank���
//����ֵ���������ͣ�0��ʾ�޴���
uint8_t stmflash_erase_page(uint16_t start_page_sn, uint8_t len, uint8_t bank)
{
  uint32_t err;
  uint8_t ret;
  FLASH_EraseInitTypeDef EraseInitStruct;
  uint8_t flash_count=0;
  
  EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;    //ɾ����ʽ
  EraseInitStruct.Page        = start_page_sn;            //��ʼҳ��
  EraseInitStruct.NbPages     = len;                      //ҳ������
  EraseInitStruct.Banks       = bank;                     //bank��
  
  HAL_FLASH_Unlock();         //��������׼������FLASH����
  
  for(uint8_t i=0; i<10; i++)      //����ظ�10�Σ������Ȼʧ�ܣ��򷵻�
  {
    ret = HAL_FLASHEx_Erase(&EraseInitStruct, &err);      //����
    if (ret == HAL_OK)
      break;
  }
  
  HAL_FLASH_Lock();           //�������Խ���FLASH����
  return ret;
}


//��Ƭ��FLASH
//addr: 32λ�ĵ�ֵַ,��ֵӦ����8������������Ϊ�ǰ���64λ�ķ�ʽд�����ݵģ���дӦ������һ��
//pdata: ���ص������׵�ַ
//len: ����ȡ���ݵĳ��ȣ� �����ǰ�uint64_t������
//����ֵ: �������ͣ�0��ʾ�޴���
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


//ֱ��д��FLASH����д��ǰ�����FLASH�����Ƿ�Ϊ��
static uint8_t stmflash_write_without_check(uint32_t addr, uint64_t* pdata64, uint32_t len_64)
{
  uint16_t i,j;
  uint8_t ret;
  uint64_t read;
  
  HAL_FLASH_Unlock();
  
  for(j=0; j<len_64; j++)
  {
    for(i=0; i<10; i++)    //������10�ε��ظ��������Ա�֤д��ɹ�
    {
      ret = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr+j*8, *(pdata64+j));
      if(ret == HAL_OK)
      {
        read = *(__IO uint64_t *)(addr+j*8);  //��MCU��Ϊд����ȷ�Ժ��ٴζ�ȡ���ݣ������бȶԣ�����ȶԲ��ɹ���˵��д�����
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

//��Ƭ��FLASHд�����ݣ�ÿ��д��8�ֽڣ�����8�ֽڵģ���0xFF����
//addr: 32λ�ĵ�ֵַ,��ֵӦ����8������������Ϊ�ǰ���64λ�ķ�ʽ��ȡ���ݵ�
//pdata: ��д��������׵�ַ
//len: ��д�����ݵĳ��ȣ������ǰ�uint64_t������
//����ֵ: �������ͣ�0��ʾ�޴���
//
//ע�����
//    д���ֳ�С��64λ�ı���ʱ�����밴���·�ʽ���У�����ֱ�ӽ�uint8_t uint16_t uint32_t�����͵�������
//    uint64_t write_uint64_t_temp;
//    write_uint64_t_temp = 40000;
//    LL_flash_write(0x0801f800+8*0, &write_uint64_t_temp, 1);
uint8_t stmflash_write(uint32_t addr, uint64_t* pdata64, uint32_t len_64)
{
  uint8_t ret, need_erase=0;
  uint8_t page_sn=0;                               //ҳ��
  uint8_t is_across_page=0;                        //��ҳ��־
  uint32_t need_write_len;                         //��Ҫд������ݳ���
  uint32_t i,j;
  
  
  if(addr%8 != 0)   //�����ַ����8�ı�����ֱ�ӷ��ش���
    return 255;
  
  //��д����ȫΪ��(0xFF)��
  //  Y �������
  //    need_erase = 0 �������
  //  N ��Ҫ����
  //    need_erase = 1 ��Ҫ����
  
  //��д����ֲ��ڼ���ҳ��
  //  area=1 ��1��ҳ�ڣ��޿�ҳ����
  //  area>1 �ڶ��ҳ�ڣ��п�ҳ����

  //д�����̣�
  //��need_erase == 0����ֱ��д��
  //��need_erase != 0����Ҫ 1 ����ҳ�����ݱ��浽������
  //                        2 ���»������ڵ����ݣ����Դ�д������仺������2K�ֽڣ���256��uint64_t��
  //                        3 ��������������д��ҳ
  
  
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //
  //    ׼���׶Σ��������׼������
  //
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  for(i=0; i<len_64; i++)   //����Ƿ�Ϊ��ֵ����0xFFFFFFFFFFFFFFFF
  {
    if(*(__IO uint64_t *)(addr+i*8) != FLASH_NULL_U64)
    {
      need_erase = 1;       //ֻҪ��1���ֽڲ�Ϊ�գ�����в�д����
      break;
    }
  }
  
  if(need_erase == 0)   //������������ֱ��д�룬Ȼ��Ϳ����˳���
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
  //  ��ʼҳ����
  //    1 ����ʼҳ������ȫ�����뻺����
  //    2 ������ʼҳ
  //    3 ����ʼҳӦ�ñ�����ԭʼ����д��FLASHԭʼλ��
  //
  ////////////////////////////////////////
  if(need_erase)
  {
    //1 ����ʼҳ������ȫ�����뻺����
    page_sn = (addr - STM32_FLASH_BASE) / STM32_FLASH_PAGE_SIZE;

    need_write_len = (addr - STM32_FLASH_BASE - STM32_FLASH_PAGE_SIZE * page_sn)/8;

    ret = stmflash_read(STM32_FLASH_BASE + STM32_FLASH_PAGE_SIZE * page_sn, \
                        page_buf, \
                        256);
    if(ret != HAL_OK)
      return ret;

    //2 ������ʼҳ
    if(is_across_page == 1)   //��д���ݿ�ҳʱ�Ų��������򲻲���
    {
      ret = stmflash_erase_page(page_sn, 1, 0);
      if(ret != HAL_OK)
        return ret;
    }

    //3 ��Ӧ�ñ�����ԭʼ����д��FLASH
    ret = stmflash_write_without_check(STM32_FLASH_BASE + STM32_FLASH_PAGE_SIZE * page_sn,\
                                       page_buf,\
                                       need_write_len);
    if(ret != HAL_OK)
      return ret;
  }

  /////////////////////////////////////////////////////////////////
  //
  //  ��ֹҳ����
  //    1 ����ֹҳ������ȫ�����뻺����
  //    2 ������ֹҳ
  //    3 ����ֹҳӦ�ñ�����ԭʼ����д��FLASHԭʼλ��
  /////////////////////////////////////////////////////////////////
  j = (2048-(addr+len_64*8-(STM32_FLASH_BASE+2048*63)))/8;
  if(need_erase == 1)//��Ҫ����
  {
    for(i=0; i<j; i++)   //����Ƿ�Ϊ��ֵ����0xFFFFFFFFFFFFFFFF
    {
      if(*(__IO uint64_t *)(addr+len_64*8+i*8) != FLASH_NULL_U64)
      {
        need_erase = 1;       //ֻҪ��1���ֽڲ�Ϊ�գ�����в�д����
        break;
      }
    }
  }


  if((addr+len_64*8-STM32_FLASH_BASE)%STM32_FLASH_PAGE_SIZE != 0)   //��Ϊ0����ʾ����λ�ò���ҳβ����Ҫ������ֹҳ�����ݱ������
  {
    i = (addr + len_64*8) & 0xfffff800; //��ֹҳ���׵�ַ
    j = addr + len_64*8 - i;
    if(is_across_page == 1)   //��ҳʱ�Ĳ���
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
    else  //����ҳʱ�Ĳ���
    {
      for(i=0; i<256; i++)
        page_buf[i] = i<<8|i;
      if(need_erase == 1)
      {
        //��д�����0xff
        
        //����ԭʼ���ݵ�ҳ������
        i = addr & 0xfffff800;    //��ֹҳ��ҳ�׵�ַ
        ret = stmflash_read(i, page_buf, STM32_FLASH_PAGE_SIZE/8);  //���ݴ�д��ȫ������
        
        //����д���ݷŽ�ҳ������
        j = ((addr - STM32_FLASH_BASE) % STM32_FLASH_PAGE_SIZE) / 8;
        for(i=0; i<len_64; i++)
          page_buf[j+i] = *(pdata64+i);

        
        //����ҳ
        stmflash_erase_page(((addr&0xfffff800) - STM32_FLASH_BASE)/STM32_FLASH_PAGE_SIZE, 1, 0);
        
        //��ҳ����ȫ������д����ֹҳ
        i = addr & 0xfffff800;    //��ֹҳ��ҳ�׵�ַ
        stmflash_write_without_check(i,page_buf,STM32_FLASH_PAGE_SIZE/8);
      }
    }
    if(ret != HAL_OK)
      return ret;
  }


  /////////////////////////////////////////////////////////////////
  //
  //  ���ˣ������д���ݲ���ҳ��ȫ��������ɣ�
  //  �����д���ݿ�ҳ����δ��ʼ����д�룬ֻ�ǽ�ȫ����д�����Ϊff����׼��д��
  //
  /////////////////////////////////////////////////////////////////


  /////////////////////////////////////////////////////////////////
  //
  //  ��ʼ����д��
  //    1 ������������ҳ����
  //    2 һ����д��ȫ����д����
  //
  /////////////////////////////////////////////////////////////////
  
  //1 ������������ҳ����
  if((need_erase==1) && (is_across_page==1))    //��Ҫ�����������ݿ�ҳʱ���Ž����м�ҳ�Ĳ���
  {
    i = len_64 - (256-need_write_len);      //����ҳ����ֹҳ����������
    j = i - (i % 256);                      //����ҳ����������
    ret = stmflash_erase_page(page_sn+1, j/256, 0);//��������ҳ����������
    if(ret != HAL_OK)
      return ret;
  }
  
  //    2 һ����д��ȫ����д����
  if(is_across_page==1) //�������ҳ��ǰ���Ѿ����������д��
    ret = stmflash_write_without_check(addr, pdata64, len_64);


  return ret;
}

