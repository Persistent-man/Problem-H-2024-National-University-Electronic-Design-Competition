#include "stm32f10x.h"                  // Device header
#include "headfile.h"

//以下为定时器中断服务函数
void TIM2_IRQHandler(void)
{
	if(TIM2->SR&1)
	{
		//此处编写中断代码
		TIM2->SR &= ~1; 
	}
}

void TIM3_IRQHandler(void)
{
	if(TIM3->SR&1)
	{
		//此处编写中断代码
//		speed_now = Encoder_count;
//		Encoder_count = 0;
		if(g_start_pending)
		{
			if(g_start_delay_tick > 0)
			{
				g_start_delay_tick--;
			}
			else
			{
				g_start_pending = 0;
				Route_Start();
			}
		}
		pid_control();
		
		TIM3->SR &= ~1; 
	}
}

void TIM4_IRQHandler(void)
{
	if(TIM4->SR&1)
	{
		//此处编写中断代码
		TIM4->SR &= ~1; 
	}
}


//以下为串口中断服务函数
void USART1_IRQHandler(void)
{
	if (USART1->SR&0x20)
	{
		//此处编写中断代码

		USART1->SR &= ~0x20;   //清除标志位
	}
}


void USART2_IRQHandler(void)
{
	if (USART2->SR&0x20)
	{
		//此处编写中断代码

		USART2->SR &= ~0x20;   //清除标志位
	}
}

void USART3_IRQHandler(void)
{
	if (USART3->SR&0x20)
	{
		//此处编写中断代码

		USART3->SR &= ~0x20;   //清除标志位
	}
}


//以下为外部中断服务函数
void EXTI0_IRQHandler(void) // PA0/PB0/PC0
{
	if(EXTI->PR&(1<<0))
	{
		//此处编写中断代码
		
		EXTI->PR = 1<<0; //清除标志位
	}
}

void EXTI1_IRQHandler(void) // PA1/PB1/PC1
{
	if(EXTI->PR&(1<<1))
	{
		//此处编写中断代码
		
		EXTI->PR = 1<<1; //清除标志位
	}
}
void EXTI2_IRQHandler(void) // PA2/PB2/PC2
{
	if(EXTI->PR&(1<<2))
	{
		//此处编写中断代码
		if(gpio_get(GPIO_A, Pin_3))
			Encoder_count1 ++;
		else
			Encoder_count1 --;
		
		EXTI->PR = 1<<2; //清除标志位
	}
}
void EXTI3_IRQHandler(void) // PA3/PB3/PC3
{
	if(EXTI->PR&(1<<3))
	{
		//此处编写中断代码
		// 获取 MPU6050 原始数据；没有磁力计时不要再读 HMC5883L，避免 I2C 等待拖慢中断。
		MPU6050_GetData();
		
		// 陀螺仪角度。yaw 只能靠 gz 积分，gz_offset 是上电静止时标定出的零漂。
		roll_gyro += (float)gx / 16.4f * 0.005f;
		pitch_gyro += (float)gy / 16.4f * 0.005f;
		yaw_gyro += ((float)gz - gz_offset) / 16.4f * 0.005f;
		
		// 加速度计只能修正 roll/pitch，不能修正 yaw；yaw_acc 保留为调试参考，不参与角度环。
		roll_acc = atan((float)ay/az) * 57.296f;
		pitch_acc = - atan((float)ax/az) * 57.296f;	
		yaw_acc = atan((float)ay/ax) * 57.296f;
		
		// 没有磁力计时，yaw_Kalman 不能再融合 yaw_hmc；这里直接使用短时间相对角 yaw_gyro。
		roll_Kalman = Kalman_Filter(&KF_Roll, roll_acc, (float)gx / 16.4f);
		pitch_Kalman = Kalman_Filter(&KF_Pitch, pitch_acc, (float)gy / 16.4f);
		yaw_Kalman = yaw_gyro;
		EXTI->PR = 1<<3; //清除标志位
	}
}
void EXTI4_IRQHandler(void) // PA4/PB4/PC4
{
	if(EXTI->PR&(1<<4))
	{
		//此处编写中断代码
		if(gpio_get(GPIO_A, Pin_5))
			Encoder_count2 ++;
		else
			Encoder_count2 --;
		EXTI->PR = 1<<4; //清除标志位
	}
}

void EXTI9_5_IRQHandler(void)
{
	if(EXTI->PR&(1<<5))   //EXTI5  PA5/PB5/PC5
	{
		//此处编写中断代码

		EXTI->PR = 1<<5; //清除标志位
	}
	
	if(EXTI->PR&(1<<6))   //EXTI6  PA6/PB6/PC6
	{
		//此处编写中断代码
		
		EXTI->PR = 1<<6; //清除标志位
	}
	
	if(EXTI->PR&(1<<7))   //EXTI7  PA7/PB7/PC7
	{
		//此处编写中断代码
		
		// 获取 MPU6050 原始数据；没有磁力计时不要再读 HMC5883L，避免 I2C 等待拖慢中断。
		MPU6050_GetData();
		
		// 陀螺仪角度。yaw 只能靠 gz 积分，gz_offset 是上电静止时标定出的零漂。
		roll_gyro += (float)gx / 16.4f * 0.005f;
		pitch_gyro += (float)gy / 16.4f * 0.005f;
		yaw_gyro += ((float)gz - gz_offset) / 16.4f * 0.005f;
		
		// 加速度计只能修正 roll/pitch，不能修正 yaw；yaw_acc 保留为调试参考，不参与角度环。
		roll_acc = atan((float)ay/az) * 57.296f;
		pitch_acc = - atan((float)ax/az) * 57.296f;	
		yaw_acc = atan((float)ay/ax) * 57.296f;
		
		// 没有磁力计时，yaw_Kalman 不能再融合 yaw_hmc；这里直接使用短时间相对角 yaw_gyro。
		roll_Kalman = Kalman_Filter(&KF_Roll, roll_acc, (float)gx / 16.4f);
		pitch_Kalman = Kalman_Filter(&KF_Pitch, pitch_acc, (float)gy / 16.4f);
		yaw_Kalman = yaw_gyro;
		
		EXTI->PR = 1<<7; //清除标志位
	}
	
	if(EXTI->PR&(1<<8))   //EXTI8
	{
		//此处编写中断代码
		
		EXTI->PR = 1<<8; //清除标志位
	}
	
	if(EXTI->PR&(1<<9))   //EXTI9
	{
		//此处编写中断代码
	
		EXTI->PR = 1<<9; //清除标志位
	}
}
