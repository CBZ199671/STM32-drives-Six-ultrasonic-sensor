#include "led.h"
#include "delay.h"
#include "key.h"
#include "sys.h"
#include "usart.h"
#include "exti.h"
#include "beep.h"
#include "timer.h"
#include "wave.h"
#include "sys.h"


 int main(void)
 {		
	delay_init();	    	 //��ʱ������ʼ��	  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //����NVIC�жϷ���2:2λ��ռ���ȼ���2λ��Ӧ���ȼ�
	uart_init(115200);	 //���ڳ�ʼ��Ϊ115200
 	LED_Init();		  		//��ʼ����LED���ӵ�Ӳ���ӿ�
	BEEP_Init();         	//��ʼ���������˿�
	KEY_Init();         	//��ʼ���밴�����ӵ�Ӳ���ӿ�
	//EXTIX_Init();		 	//�ⲿ�жϳ�ʼ��
	//LED0=0;					//����LED0
	Timer_SRD_Init(5000,7199);
	UltrasonicWave_Configuration();

	while(1)
	{	  
    ultrasonic_startMeasure();
    printf("a= %.2f\r\n", distance_ultrasonic[0]);//��λΪcm
    printf("b= %.2f\r\n", distance_ultrasonic[1]);
		printf("c= %.2f\r\n", distance_ultrasonic[2]);
    printf("d= %.2f\r\n", distance_ultrasonic[3]);
    printf("e= %.2f\r\n", distance_ultrasonic[4]);
		printf("f= %.2f\r\n", distance_ultrasonic[5]);
	}
 }

