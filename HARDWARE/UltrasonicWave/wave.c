#include "wave.h"
#include "sys.h"
#include "delay.h"
#include "usart.h"

#define TRIG_PORT      GPIOC            
#define ECHO_PORT      GPIOD 

#define TRIG_0_PIN       GPIO_Pin_4   
#define ECHO_0_PIN       GPIO_Pin_10  

#define TRIG_1_PIN       GPIO_Pin_5  
#define ECHO_1_PIN       GPIO_Pin_11

#define TRIG_2_PIN       GPIO_Pin_6
#define ECHO_2_PIN       GPIO_Pin_12

#define TRIG_3_PIN       GPIO_Pin_7
#define ECHO_3_PIN       GPIO_Pin_13

#define TRIG_4_PIN       GPIO_Pin_8
#define ECHO_4_PIN       GPIO_Pin_14

#define TRIG_5_PIN       GPIO_Pin_9
#define ECHO_5_PIN       GPIO_Pin_15



#define UL_NUM 6
#define MAX_MEASURE_DISTANCE 255      //单位：cm
#define UL_TIM_MAX_COUNT 150 //最大测量距离对应的定时器计数  //cnt = (255cm * 2 /(340 * 100))* 10000
/* 该时间设置非常关键，当前设定时间要确保上个超声波已经进入了中断（因为进入了中断就会自动等到测量完成），
   待上个测量完成后才能开启下个超声波的测量。否则两个测量信号都发送出去，外部中断会依次触发先执行先收到
   触发信号的中断服务函数，另一个中断会等到当前中断执行完毕之后才会进入，这时后面信号的高电平时间可能很
   小或没有了，因而收到的数据会显示0或较小的数值。 */
#define MEASURE_INTERVAL 10000  //单位：us  //由实验测量大于等于5000us较为合适	 


float distance_ultrasonic[UL_NUM];
 

void UltrasonicWave_Configuration(void)    //超声波引脚、中断及优先级配置
{
  GPIO_InitTypeDef GPIO_InitStructure;  
  EXTI_InitTypeDef EXTI_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
     
  GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable , ENABLE);   
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD | RCC_APB2Periph_AFIO, ENABLE);      
	
  GPIO_InitStructure.GPIO_Pin = TRIG_0_PIN | TRIG_1_PIN | TRIG_2_PIN | TRIG_3_PIN | TRIG_4_PIN | TRIG_5_PIN;                  
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;           
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;          
  GPIO_Init(TRIG_PORT, &GPIO_InitStructure);                     
 
  GPIO_InitStructure.GPIO_Pin = ECHO_0_PIN | ECHO_1_PIN | ECHO_2_PIN | ECHO_3_PIN | ECHO_4_PIN | ECHO_5_PIN;                  
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;      
  GPIO_Init(ECHO_PORT,&GPIO_InitStructure);                     
     
     
  //中断配置   
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOD,GPIO_PinSource10);  
 
  EXTI_InitStructure.EXTI_Line=EXTI_Line10;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt; 
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);    
         
         
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOD,GPIO_PinSource11);
 
  EXTI_InitStructure.EXTI_Line=EXTI_Line11;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt; 
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
	
	
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOD,GPIO_PinSource12);
 
  EXTI_InitStructure.EXTI_Line=EXTI_Line12;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt; 
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

   
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOD,GPIO_PinSource13);
 
  EXTI_InitStructure.EXTI_Line=EXTI_Line13;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt; 
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);


  GPIO_EXTILineConfig(GPIO_PortSourceGPIOD,GPIO_PinSource14);
 
  EXTI_InitStructure.EXTI_Line=EXTI_Line14;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt; 
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
	
	
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOD,GPIO_PinSource15);
 
  EXTI_InitStructure.EXTI_Line=EXTI_Line15;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt; 
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
	
	
  NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;          
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;   
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;                  
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;                            
  NVIC_Init(&NVIC_InitStructure);                
		}
 
 
 
 
void EXTI15_10_IRQHandler(void)
{
	  /*Ultrasonic 1*/	
	if(EXTI_GetITStatus(EXTI_Line10) != RESET)
	{
    EXTI_ClearITPendingBit(EXTI_Line10);  
		TIM_SetCounter(TIM3, 0);
		TIM_Cmd(TIM3, ENABLE);
		while(GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_10))  //上升沿触发中断后等待变为低电平
    {
      if(TIM_GetCounter(TIM3) == UL_TIM_MAX_COUNT) // cnt = (255cm * 2 /(340 * 100))* 10000
      {
        break;  //超出设定的测量最大距离，跳出等待
      }
    }
		TIM_Cmd(TIM3,DISABLE);
    /* 计算超声波测量到的距离，如超出最大距离按最大距离显示 */
		distance_ultrasonic[0] = TIM_GetCounter(TIM3) * 340 / 200.0;  //cnt * (1/10000) * (340 / 2) *100(单位：cm)
	}
  
  /* Ultrosonic 2 */
	if(EXTI_GetITStatus(EXTI_Line11) != RESET)
	{
    EXTI_ClearITPendingBit(EXTI_Line11); 
		TIM_SetCounter(TIM3, 0);
		TIM_Cmd(TIM3, ENABLE);
		while(GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_11))  //上升沿触发中断后等待变为低电平
    {
      if(TIM_GetCounter(TIM3) == UL_TIM_MAX_COUNT) // cnt = (255cm * 2 /(340 * 100))* 10000
      {
        break;  //超出设定的测量最大距离，跳出等待
      }
    }
		TIM_Cmd(TIM3,DISABLE);
    /* 计算超声波测量到的距离，如超出最大距离按最大距离显示 */
		distance_ultrasonic[1] = TIM_GetCounter(TIM3) * 340 / 200.0;  //cnt * (1/10000) * (340 / 2) *100(单位：cm)
  } 
	
	 /* Ultrosonic 3 */
	if(EXTI_GetITStatus(EXTI_Line12) != RESET)
	{
    EXTI_ClearITPendingBit(EXTI_Line12); 
		TIM_SetCounter(TIM3, 0);
		TIM_Cmd(TIM3, ENABLE);
		while(GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_12))  //上升沿触发中断后等待变为低电平
    {
      if(TIM_GetCounter(TIM3) == UL_TIM_MAX_COUNT) // cnt = (255cm * 2 /(340 * 100))* 10000
      {
        break;  //超出设定的测量最大距离，跳出等待
      }
    }
		TIM_Cmd(TIM3,DISABLE);
    /* 计算超声波测量到的距离，如超出最大距离按最大距离显示 */
		distance_ultrasonic[2] = TIM_GetCounter(TIM3) * 340 / 200.0;  //cnt * (1/10000) * (340 / 2) *100(单位：cm)
  } 
	
	
	/*Ultrasonic 4*/	
	if(EXTI_GetITStatus(EXTI_Line13) != RESET)
	{
    EXTI_ClearITPendingBit(EXTI_Line13);  
		TIM_SetCounter(TIM3, 0);
		TIM_Cmd(TIM3, ENABLE);
		while(GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_13))  //上升沿触发中断后等待变为低电平
    {
      if(TIM_GetCounter(TIM3) == UL_TIM_MAX_COUNT) // cnt = 255cm * 2 /(340 * 100) * 100000
      {
        break;  //超出设定的测量最大距离，跳出等待
      }
    }
		TIM_Cmd(TIM3,DISABLE);
    /* 计算超声波测量到的距离，如超出最大距离按最大距离显示 */
		distance_ultrasonic[3] = TIM_GetCounter(TIM3) * 340 / 200.0;  //cnt * (1/10000) * (340 / 2) *100(单位：cm)
	}
  
  /* Ultrosonic 5 */
	if(EXTI_GetITStatus(EXTI_Line14) != RESET)
	{
    EXTI_ClearITPendingBit(EXTI_Line14); 
		TIM_SetCounter(TIM3, 0);
		TIM_Cmd(TIM3, ENABLE);
		while(GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_14))  //上升沿触发中断后等待变为低电平
    {
      if(TIM_GetCounter(TIM3) == UL_TIM_MAX_COUNT) // cnt = 255cm * 2 /(340 * 100) * 100000
      {
        break;  //超出设定的测量最大距离，跳出等待
      }
    }
		TIM_Cmd(TIM3,DISABLE);
    /* 计算超声波测量到的距离，如超出最大距离按最大距离显示 */
		distance_ultrasonic[4] = TIM_GetCounter(TIM3) * 340 / 200.0;  //cnt * (1/10000) * (340 / 2) *100(单位：cm)
  } 
	
	 /* Ultrosonic 6 */
	if(EXTI_GetITStatus(EXTI_Line15) != RESET)
	{
    EXTI_ClearITPendingBit(EXTI_Line15); 
		TIM_SetCounter(TIM3, 0);
		TIM_Cmd(TIM3, ENABLE);
		while(GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_15))  //上升沿触发中断后等待变为低电平
    {
      if(TIM_GetCounter(TIM3) == UL_TIM_MAX_COUNT) // cnt = 255cm * 2 /(340 * 100) * 100000
      {
        break;  //超出设定的测量最大距离，跳出等待
      }
    }
		TIM_Cmd(TIM3,DISABLE);
    /* 计算超声波测量到的距离，如超出最大距离按最大距离显示 */
		distance_ultrasonic[5] = TIM_GetCounter(TIM3) * 340 / 200.0;  //cnt * (1/10000) * (340 / 2) *100(单位：cm)
  } 	
	
}
 


//依次发送触发信号，轮询测量距离值。根据测量机制，该函数执行完毕，整个测量过程已经执行完成。
void ultrasonic_startMeasure(void)
{
	GPIO_SetBits(GPIOC, GPIO_Pin_4);
	delay_us(10);
	GPIO_ResetBits(GPIOC, GPIO_Pin_4);
  delay_us(MEASURE_INTERVAL); //延时保证UL1已经进入中断

  GPIO_SetBits(GPIOC, GPIO_Pin_5);
  delay_us(10);
  GPIO_ResetBits(GPIOC, GPIO_Pin_5);
  delay_us(MEASURE_INTERVAL); //延时保证UL2已经进入中断
	
	GPIO_SetBits(GPIOC, GPIO_Pin_6);
	delay_us(10);
	GPIO_ResetBits(GPIOC, GPIO_Pin_6);
	delay_us(MEASURE_INTERVAL); //延时保证UL3已经进入中断
	
	GPIO_SetBits(GPIOC, GPIO_Pin_7);
	delay_us(10);
	GPIO_ResetBits(GPIOC, GPIO_Pin_7);
  delay_us(MEASURE_INTERVAL); //延时保证UL4已经进入中断

  GPIO_SetBits(GPIOC, GPIO_Pin_8);
  delay_us(10);
  GPIO_ResetBits(GPIOC, GPIO_Pin_8);
  delay_us(MEASURE_INTERVAL); //延时保证UL5已经进入中断
	
	GPIO_SetBits(GPIOC, GPIO_Pin_9);
	delay_us(10);
	GPIO_ResetBits(GPIOC, GPIO_Pin_9);
	delay_us(MEASURE_INTERVAL); //延时保证UL6已经进入中断	
}













