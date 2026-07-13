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
		
		EXTI->PR = 1<<2; //清除标志位
	}
}
void EXTI3_IRQHandler(void) // PA3/PB3/PC3
{
	if(EXTI->PR&(1<<3))
	{
		//此处编写中断代码
		
		EXTI->PR = 1<<3; //清除标志位
	}
}
void EXTI4_IRQHandler(void) // PA4/PB4/PC4
{
	if(EXTI->PR&(1<<4))
	{
		//此处编写中断代码
		
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
