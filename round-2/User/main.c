#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"
#include "Key.h"
#include "Timer.h"
#include "Motor.h"
#include "Encoder.h"
#include "Serial.h"


/*电机测试*/

uint8_t KeyNum;
uint8_t found=0;

float Target_s1=0,Actual_s1=0,Out_s1=0;
float kp_s1=0.3,ki_s1=0.1,kd_s1=0.01;
float Error0_s1=0,Error1_s1=0,Error2_s1=0;

float Target_s2=0,Actual_s2=0,Out_s2=0;
float kp_s2=0.3,ki_s2=0.1,kd_s2=0.01;
float Error0_s2=0,Error1_s2=0,Error2_s2=0;



float Target_pos1=0,Actual_pos1=0,Out_pos1=0;
float kp_pos1=0.3,ki_pos1=0.1,kd_pos1=0.01;
float Error0_pos1=0,Error1_pos1=0,Error2_pos1=0;

float Target_pos2=0,Actual_pos2=0,Out_pos2=0;
float kp_pos2=0.3,ki_pos2=0.1,kd_pos2=0.01;
float Error0_pos2=0,Error1_pos2=0,Error2_pos2=0;

float PID(float kp,float ki,float kd,float target,float actual,float *error0,float *error1,float *error2,float *output)
{
			*error2=*error1;
			*error1=*error0;
			*error0=target-actual;
			
			*output+=kp*(*error0-*error1)+ki*(*error0)+kd*(*error0-2*(*error1)+*error2);
		return *output;
}

int main(void)
{
	OLED_Init();
	Key_Init();
	Motor_Init();
	Encoder1_Init();
	Encoder2_Init();
	
	Timer_Init();
	
//	OLED_Printf(0,0,OLED_8X16,"Speed Control");//location control
//	OLED_Update();
	
	
	while(1)
	{
		KeyNum=Key_GetNum();
		
		if(KeyNum==1)  
		{
			found=!found;		
			if(found)
			{
                Error0_pos1 = Error1_pos1 = Error2_pos1 = 0;
                Error0_pos2 = Error1_pos2 = Error2_pos2 = 0;
                Out_pos1 = Out_pos2 = 0;
				
				OLED_Printf(0,0,OLED_8X16,"position Control");
				OLED_Update();
            }
            else
            {
                Error0_s1 = Error1_s1 = Error2_s1 = 0;
                Out_s1 = 0;
				
				OLED_Printf(0,0,OLED_8X16,"Speed Control");
				OLED_Update();
            }
//		OLED_Update();
//		Serial_Printf("%f,%f,%f\r\n",Target,Actual,Out); 
		}
	}
}


void TIM1_UP_IRQHandler(void)
{
	static uint16_t Count=0;
	if(TIM_GetITStatus(TIM1,TIM_IT_Update)==SET)
	{
		
		
		Key_Tick();
		Count++;
		if(Count>=10)
		{
			Count=0;
			
			Actual_s1=Encoder1_Get();
			Actual_s2=Encoder2_Get();
			Actual_pos1+=Actual_s1;
			Actual_pos2+=Actual_s2;
			
//			Actual_pos1+=Encoder1_Get();//+= 位置式
//			Actual_pos2+=Encoder2_Get();//+= 位置式
			
			if(found==1)
			{
				Target_pos2=Actual_pos1;
				Target_pos1=Actual_pos1;//阻力
				
				PID(kp_pos1,ki_pos1,kd_pos1,Target_pos1,Actual_pos1,&Error0_pos1,&Error1_pos1,&Error2_pos1,&Out_pos1);
				PID(kp_pos2,ki_pos2,kd_pos2,Target_pos2,Actual_pos2,&Error0_pos2,&Error1_pos2,&Error2_pos2,&Out_pos2);
				
			if(Out_pos1>100){Out_pos1=100;}
			if(Out_pos1<-100){Out_pos1=-100;};
			if(Out_pos2>100){Out_pos2=100;}
			if(Out_pos2<-100){Out_pos2=-100;};
			
			Motor_SetPWM(Out_pos1);
			Motor_SetPWM(Out_pos2);
			}
			else
			{
				PID(kp_s1,ki_s1,kd_s1,Target_s1,Actual_s1,&Error0_s1,&Error1_s1,&Error2_s1,&Out_s1);
				if(Out_s1>100){Out_s1=100;}
				if(Out_s1<-100){Out_s1=-100;};
			
				Motor_SetPWM(Out_s1);
				Motor_SetPWM(Out_s2);
				
				Serial_Printf("%.0f,%.0f,%.0f,%.1f\r\n", Target_s1, Actual_s1, Actual_s2, Out_s1);
			}
		}
		TIM_ClearITPendingBit(TIM1,TIM_IT_Update);
	}
}


/*编码器测试*/
//int16_t Speed;
//int16_t Location;

//int main(void)
//{
//	OLED_Init();
//	Encoder_Init();
//	Timer_Init();
//	while(1)
//	{
//		
//		OLED_Printf(0,0,OLED_8X16,"Speed:%+05d",Speed);
//		OLED_Printf(0,16,OLED_8X16,"Location:%+05d",Location);
//		OLED_Update();
//	}
//}


//void TIM1_UP_IRQHandler(void)
//{
//	static uint16_t Count;
//	
//	if(TIM_GetITStatus(TIM1,TIM_IT_Update)==SET)
//	{
//		Count++;
//		if(Count>=40)
//		{
//			Count=0;
//			
//			Speed=Encoder_Get();
//			Location+=Speed;
//		}
//		TIM_ClearITPendingBit(TIM1,TIM_IT_Update);
//	}
//}

