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
#define MAX_MEASURE_DISTANCE 255      //��λ��cm
#define UL_TIM_MAX_COUNT 150 //�����������Ӧ�Ķ�ʱ������  //cnt = (255cm * 2 /(340 * 100))* 10000
/* ��ʱ�����÷ǳ��ؼ�����ǰ�趨ʱ��Ҫȷ���ϸ��������Ѿ��������жϣ���Ϊ�������жϾͻ��Զ��ȵ�������ɣ���
   ���ϸ�������ɺ���ܿ����¸��������Ĳ������������������źŶ����ͳ�ȥ���ⲿ�жϻ����δ�����ִ�����յ�
   �����źŵ��жϷ���������һ���жϻ�ȵ���ǰ�ж�ִ�����֮��Ż���룬��ʱ�����źŵĸߵ�ƽʱ����ܺ�
   С��û���ˣ�����յ������ݻ���ʾ0���С����ֵ�� */
#define MEASURE_INTERVAL 10000  //��λ��us  //��ʵ��������ڵ���5000us��Ϊ����	 


float distance_ultrasonic[UL_NUM];
 

void UltrasonicWave_Configuration(void)    //���������š��жϼ����ȼ�����
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
     
     
  //�ж�����   
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
		while(GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_10))  //�����ش����жϺ�ȴ���Ϊ�͵�ƽ
    {
      if(TIM_GetCounter(TIM3) == UL_TIM_MAX_COUNT) // cnt = (255cm * 2 /(340 * 100))* 10000
      {
        break;  //�����趨�Ĳ��������룬�����ȴ�
      }
    }
		TIM_Cmd(TIM3,DISABLE);
    /* ���㳬�����������ľ��룬�糬�������밴��������ʾ */
		distance_ultrasonic[0] = TIM_GetCounter(TIM3) * 340 / 200.0;  //cnt * (1/10000) * (340 / 2) *100(��λ��cm)
	}
  
  /* Ultrosonic 2 */
	if(EXTI_GetITStatus(EXTI_Line11) != RESET)
	{
    EXTI_ClearITPendingBit(EXTI_Line11); 
		TIM_SetCounter(TIM3, 0);
		TIM_Cmd(TIM3, ENABLE);
		while(GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_11))  //�����ش����жϺ�ȴ���Ϊ�͵�ƽ
    {
      if(TIM_GetCounter(TIM3) == UL_TIM_MAX_COUNT) // cnt = (255cm * 2 /(340 * 100))* 10000
      {
        break;  //�����趨�Ĳ��������룬�����ȴ�
      }
    }
		TIM_Cmd(TIM3,DISABLE);
    /* ���㳬�����������ľ��룬�糬�������밴��������ʾ */
		distance_ultrasonic[1] = TIM_GetCounter(TIM3) * 340 / 200.0;  //cnt * (1/10000) * (340 / 2) *100(��λ��cm)
  } 
	
	 /* Ultrosonic 3 */
	if(EXTI_GetITStatus(EXTI_Line12) != RESET)
	{
    EXTI_ClearITPendingBit(EXTI_Line12); 
		TIM_SetCounter(TIM3, 0);
		TIM_Cmd(TIM3, ENABLE);
		while(GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_12))  //�����ش����жϺ�ȴ���Ϊ�͵�ƽ
    {
      if(TIM_GetCounter(TIM3) == UL_TIM_MAX_COUNT) // cnt = (255cm * 2 /(340 * 100))* 10000
      {
        break;  //�����趨�Ĳ��������룬�����ȴ�
      }
    }
		TIM_Cmd(TIM3,DISABLE);
    /* ���㳬�����������ľ��룬�糬�������밴��������ʾ */
		distance_ultrasonic[2] = TIM_GetCounter(TIM3) * 340 / 200.0;  //cnt * (1/10000) * (340 / 2) *100(��λ��cm)
  } 
	
	
	/*Ultrasonic 4*/	
	if(EXTI_GetITStatus(EXTI_Line13) != RESET)
	{
    EXTI_ClearITPendingBit(EXTI_Line13);  
		TIM_SetCounter(TIM3, 0);
		TIM_Cmd(TIM3, ENABLE);
		while(GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_13))  //�����ش����жϺ�ȴ���Ϊ�͵�ƽ
    {
      if(TIM_GetCounter(TIM3) == UL_TIM_MAX_COUNT) // cnt = 255cm * 2 /(340 * 100) * 100000
      {
        break;  //�����趨�Ĳ��������룬�����ȴ�
      }
    }
		TIM_Cmd(TIM3,DISABLE);
    /* ���㳬�����������ľ��룬�糬�������밴��������ʾ */
		distance_ultrasonic[3] = TIM_GetCounter(TIM3) * 340 / 200.0;  //cnt * (1/10000) * (340 / 2) *100(��λ��cm)
	}
  
  /* Ultrosonic 5 */
	if(EXTI_GetITStatus(EXTI_Line14) != RESET)
	{
    EXTI_ClearITPendingBit(EXTI_Line14); 
		TIM_SetCounter(TIM3, 0);
		TIM_Cmd(TIM3, ENABLE);
		while(GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_14))  //�����ش����жϺ�ȴ���Ϊ�͵�ƽ
    {
      if(TIM_GetCounter(TIM3) == UL_TIM_MAX_COUNT) // cnt = 255cm * 2 /(340 * 100) * 100000
      {
        break;  //�����趨�Ĳ��������룬�����ȴ�
      }
    }
		TIM_Cmd(TIM3,DISABLE);
    /* ���㳬�����������ľ��룬�糬�������밴��������ʾ */
		distance_ultrasonic[4] = TIM_GetCounter(TIM3) * 340 / 200.0;  //cnt * (1/10000) * (340 / 2) *100(��λ��cm)
  } 
	
	 /* Ultrosonic 6 */
	if(EXTI_GetITStatus(EXTI_Line15) != RESET)
	{
    EXTI_ClearITPendingBit(EXTI_Line15); 
		TIM_SetCounter(TIM3, 0);
		TIM_Cmd(TIM3, ENABLE);
		while(GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_15))  //�����ش����жϺ�ȴ���Ϊ�͵�ƽ
    {
      if(TIM_GetCounter(TIM3) == UL_TIM_MAX_COUNT) // cnt = 255cm * 2 /(340 * 100) * 100000
      {
        break;  //�����趨�Ĳ��������룬�����ȴ�
      }
    }
		TIM_Cmd(TIM3,DISABLE);
    /* ���㳬�����������ľ��룬�糬�������밴��������ʾ */
		distance_ultrasonic[5] = TIM_GetCounter(TIM3) * 340 / 200.0;  //cnt * (1/10000) * (340 / 2) *100(��λ��cm)
  } 	
	
}
 


//���η��ʹ����źţ���ѯ��������ֵ�����ݲ������ƣ��ú���ִ����ϣ��������������Ѿ�ִ����ɡ�
void ultrasonic_startMeasure(void)
{
	GPIO_SetBits(GPIOC, GPIO_Pin_4);
	delay_us(10);
	GPIO_ResetBits(GPIOC, GPIO_Pin_4);
  delay_us(MEASURE_INTERVAL); //��ʱ��֤UL1�Ѿ������ж�

  GPIO_SetBits(GPIOC, GPIO_Pin_5);
  delay_us(10);
  GPIO_ResetBits(GPIOC, GPIO_Pin_5);
  delay_us(MEASURE_INTERVAL); //��ʱ��֤UL2�Ѿ������ж�
	
	GPIO_SetBits(GPIOC, GPIO_Pin_6);
	delay_us(10);
	GPIO_ResetBits(GPIOC, GPIO_Pin_6);
	delay_us(MEASURE_INTERVAL); //��ʱ��֤UL3�Ѿ������ж�
	
	GPIO_SetBits(GPIOC, GPIO_Pin_7);
	delay_us(10);
	GPIO_ResetBits(GPIOC, GPIO_Pin_7);
  delay_us(MEASURE_INTERVAL); //��ʱ��֤UL4�Ѿ������ж�

  GPIO_SetBits(GPIOC, GPIO_Pin_8);
  delay_us(10);
  GPIO_ResetBits(GPIOC, GPIO_Pin_8);
  delay_us(MEASURE_INTERVAL); //��ʱ��֤UL5�Ѿ������ж�
	
	GPIO_SetBits(GPIOC, GPIO_Pin_9);
	delay_us(10);
	GPIO_ResetBits(GPIOC, GPIO_Pin_9);
	delay_us(MEASURE_INTERVAL); //��ʱ��֤UL6�Ѿ������ж�	
}













