#include "main.h"
#include "tim.h"
 
#define WS_H           80-1   // 1 ����Լ���ֵ
#define WS_L           25-1   // 0 ����Լ���ֵ
#define WS_REST        40   // ��λ�ź���������
#define LED_NUM         1   // WS2812�Ƹ���
#define DATA_LEN       24   // WS2812���ݳ��ȣ�������Ҫ24���ֽ�
#define WS2812_RST_NUM 50   // �ٷ���λʱ��Ϊ50us��40�����ڣ����������ʹ��50������
 
void WS2812_Init(void);
void WS2812_Set(uint16_t num,uint8_t R,uint8_t G,uint8_t B);
