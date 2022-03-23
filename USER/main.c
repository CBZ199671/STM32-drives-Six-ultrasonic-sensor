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
	delay_init();	    	 //延时函数初始化	  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //设置NVIC中断分组2:2位抢占优先级，2位响应优先级
	uart_init(115200);	 //串口初始化为115200
 	LED_Init();		  		//初始化与LED连接的硬件接口
	BEEP_Init();         	//初始化蜂鸣器端口
	KEY_Init();         	//初始化与按键连接的硬件接口
	//EXTIX_Init();		 	//外部中断初始化
	//LED0=0;					//点亮LED0
	Timer_SRD_Init(5000,7199);
	UltrasonicWave_Configuration();

	while(1)
	{	  
    ultrasonic_startMeasure();
    printf("a= %.2f\r\n", distance_ultrasonic[0]);//单位为cm
    printf("b= %.2f\r\n", distance_ultrasonic[1]);
		printf("c= %.2f\r\n", distance_ultrasonic[2]);
    printf("d= %.2f\r\n", distance_ultrasonic[3]);
    printf("e= %.2f\r\n", distance_ultrasonic[4]);
		printf("f= %.2f\r\n", distance_ultrasonic[5]);
	}
 }

