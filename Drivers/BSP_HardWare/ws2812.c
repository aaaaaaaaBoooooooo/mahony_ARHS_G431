#include "ws2812.h"
 
//ʹ��PWM+DMA����ws2812ʱҪע�ⶨʱ���Ƕ���λ�ģ������16λ��������DMAΪ���֣������32λ��������DMAΪȫ�� 
 
//�Դ����飬����Ϊ �Ƶ�����*24+��λ����
uint32_t WS2812_RGB_Buff[LED_NUM*DATA_LEN+WS2812_RST_NUM] = {0}; 
 
/**
 * ������WS2812�������ú���
 * ������num:�Ƶ�λ�ã�R��G��B�ֱ�Ϊ������ɫͨ�������ȣ����ֵΪ255
 * ���ã���������ÿһ��WS2812����ɫ
***/
void WS2812_Set(uint16_t num,uint8_t R,uint8_t G,uint8_t B)
{
  uint32_t indexx=(num*(3*8));
  for (uint8_t i = 0;i < 8;i++)
  {
	//�������
	WS2812_RGB_Buff[indexx+i]      = (G << i) & (0x80)?WS_H:WS_L;
	WS2812_RGB_Buff[indexx+i + 8]  = (R << i) & (0x80)?WS_H:WS_L;
	WS2812_RGB_Buff[indexx+i + 16] = (B << i) & (0x80)?WS_H:WS_L;
  }
}
 
//WS2812��ʼ������
void WS2812_Init()
{
	//���ùر����е�
  for(int i=0;i<LED_NUM;i++)
  {
			WS2812_Set(i,5,5,5);
  }
  //���ã�����DMA���Դ��е�����ʵʱ��������ʱ���ıȽϼĴ���
  HAL_TIM_PWM_Start_DMA(&htim2,TIM_CHANNEL_1,(uint32_t *)WS2812_RGB_Buff,sizeof(WS2812_RGB_Buff)/sizeof(uint32_t)); 
	
}
