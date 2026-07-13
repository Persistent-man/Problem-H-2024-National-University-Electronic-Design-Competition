#include "headfile.h"
uint8_t key_last = 0;

static uint8_t g_select_mode =1;    //默认题号
static uint8_t g_select_last =0;    //上一次选题状态
static uint8_t g_start_last=0;      //上一次开关
volatile uint8_t g_start_pending=0;   //是否按下开关等2s
volatile uint8_t g_start_delay_tick=0;//2s倒计时，放到10ms定时中断里减

int main(void)
{
	OLED_Init();
	Route_Init();
	motor_init();
	encoder_init();

	//uart_init(UART_1,115200,0);
	
	pid_init(&motorA, POSITION_PID, 50, 2.0f, 0);
  pid_init(&motorB, POSITION_PID, 50, 2.0f, 0);
	pid_init(&angle, POSITION_PID, 2, 0, 0.5f);
	
	gray_init();
	
	I2C_Init();
	MPU6050_Init();                 // 使用陀螺仪做短时间相对角度，必须先初始化 MPU6050。
	delay_ms(100);                  // 上电后等待数据稳定，标定时小车要保持静止。
	MPU6050_CalibrateGyroZ(200);    // 没有磁力计时，yaw 只靠 gz 积分，先标定 Z 轴零漂。
	MPU6050_YawReset();             // 把当前车头方向作为 0 度，后面角度环以它为相对角。
	//HMC5883L_Init();
	//exti_init(EXTI_PB7,RISING,0);
	gpio_init(GPIO_A, Pin_11, ID);
	gpio_init(GPIO_A, Pin_10, ID);
	
	tim_interrupt_ms_init(TIM_3,10,0);
	
	
	while (1)
	{
		uint8_t select_now=gpio_get(GPIO_A,Pin_11);
		uint8_t start_now=gpio_get(GPIO_A,Pin_10);

    if(g_select_last==0&&select_now==1)
    {
        delay_ms(20);  // 简单消抖
        if(gpio_get(GPIO_A, Pin_11) == 1)
        {
						g_select_mode++;
						if(g_select_mode>4)
						{
							g_select_mode=1;
						}
        }
    }
    g_select_last=select_now;
		
		if(g_start_last==0 && start_now==1)
		{
			delay_ms(20);
			if(gpio_get(GPIO_A,Pin_10)==1)
			{
				Route_SetMode(g_select_mode);
				g_start_delay_tick=200;   //10ms * 200 = 2s
				g_start_pending=1;
			}
		}
		g_start_last=start_now;
		
    //		printf("ax:%d, ay:%d az:%d gx:%d gy:%d gz:%d\r\n", ax, ay, az, gx, gy, gz);
  	//printf("yaw:%.2f  pitch:%.2f roll:%.2f\r\n", yaw_Kalman, pitch_Kalman, roll_Kalman);
		OLED_ShowSignedNum(1, 1,g_select_mode, 1);
		OLED_ShowSignedNum(1, 3, route_get_state(), 2);
		OLED_ShowFloat(2, 1, yaw_Kalman,4, 1);       //调角度环时先看yaw是否稳定
		OLED_ShowSignedNum(3, 1, Encoder_count1, 5);
		OLED_ShowSignedNum(4, 1, Encoder_count2, 5);
		//OLED_ShowFloat(2, 1, yaw_Kalman, 4, 1);
    //OLED_ShowFloat(3, 1, angle_loop_cal(0), 4, 1);
		//OLED_ShowHexNum(2, 1, MPU6050_Read(WHO_AM_I), 2);
    //OLED_ShowSignedNum(2, 1, gz, 6);

   delay_ms(20);
	} 
}
