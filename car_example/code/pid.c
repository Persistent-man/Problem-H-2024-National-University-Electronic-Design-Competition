#include "headfile.h"
float AngleTarget=0;
pid_t motorA;
pid_t motorB;
pid_t angle;

void datavision_send()  // 上位机波形发送函数
{
	// 数据包头
	uart_sendbyte(UART_1, 0x03);
	uart_sendbyte(UART_1, 0xfc);

	// 发送数据
	uart_sendbyte(UART_1, (uint8_t)motorA.target);  
	uart_sendbyte(UART_1, (uint8_t)motorA.now);
//	uart_sendbyte(UART_1, (uint8_t)motorB.target);  
//	uart_sendbyte(UART_1, (uint8_t)motorB.now);
	// 数据包尾
	uart_sendbyte(UART_1, 0xfc);
	uart_sendbyte(UART_1, 0x03);
}


void pid_init(pid_t *pid, uint32_t mode, float p, float i, float d)
{
	pid->pid_mode = mode;
	pid->p = p;
	pid->i = i;
	pid->d = d;
}

void motor_target_set(int spe1, int spe2)
{
	if(spe1 >= 0)
	{
		motorA_dir = 1;
		motorA.target = spe1;
	}
	else
	{
		motorA_dir = 0;
		motorA.target = -spe1;
	}
	
	if(spe2 >= 0)
	{
		motorB_dir = 1;
		motorB.target = spe2;
	}
	else
	{
		motorB_dir = 0;
		motorB.target = -spe2;
	}

     //目标速度不为 0 但 PWM 还太小时，给一个起步最低 PWM，帮助电机克服静摩擦。
    if(motorA.target > 0 && motorA.out < 2600)
        motorA.out = 2600;
    if(motorB.target > 0 && motorB.out < 2600)
        motorB.out = 2600;

    // 低目标速度时限制历史 PWM，避免从高速状态切到慢速/停转时继续冲出去。
	  if(motorA.target < 80 && motorA.out > 8000)
    motorA.out = 8000;
    if(motorB.target < 80 && motorB.out > 8000)
    motorB.out = 8000;

    // 目标速度为0时，必须立刻清掉PWM和PID历史量，否则位置式PID可能还会拖着车往前走。
    if(motorA.target == 0)
    {
        motorA.out = 0;
        motorA.iout = 0;
        motorA.error[0] = 0;
        motorA.error[1] = 0;
        motorA.error[2] = 0;
    }
    if(motorB.target == 0)
    {
        motorB.out = 0;
        motorB.iout = 0;
        motorB.error[0] = 0;
        motorB.error[1] = 0;
        motorB.error[2] = 0;
    }
}


void angle_loop_reset(void)
{
	// 没有磁力计时 yaw 是相对角度；调用这里表示把当前车头方向当作 0 度。
	MPU6050_YawReset();
	angle.target = 0;
	angle.now = 0;
	angle.error[0] = 0;
	angle.error[1] = 0;
	angle.error[2] = 0;
	angle.pout = 0;
	angle.iout = 0;
	angle.dout = 0;
	angle.out = 0;
}

float angle_loop_cal(float target_angle)
{
	// 以后需要按角度转弯时调用这个函数，返回值就是角度环给左右轮的差速修正量。
	angle.target = target_angle;
	angle.now = yaw_Kalman;
	pid_cal(&angle);
	return angle.out;
}

uint8_t angle_loop_reached(float tolerance)
{
	float error;

	error = angle.target - angle.now;
	if(error < 0)
		error = -error;

	return error <= tolerance;
}
void Update_Yaw_10ms(void)
{
    MPU6050_GetData();

    yaw_gyro += ((float)gz - gz_offset) / 16.4f * 0.01f;
    yaw_Kalman = yaw_gyro;
}

void pid_control(void)
{
	/* Keil老C标准要求变量定义放在语句前面，所以先定义变量，再更新yaw */
	int left_delta=Encoder_count1;
	int right_delta=Encoder_count2;
	
	/*
		只有路线真正启动后才积分yaw。
		否则上电等待、选题、2秒倒计时时，陀螺仪零漂会让yaw自己慢慢加。
	*/
	if(route_get_state()!=Route_Wait_Start)
	{
		Update_Yaw_10ms();
	}
	
	/*1读取编码量，并转化为当前方向下的正向速度*/
	if(motorA_dir)
		left_delta=Encoder_count1;
	else
		left_delta=-Encoder_count1;
	if(motorB_dir)
		right_delta=Encoder_count2;
	else
		right_delta=-Encoder_count2;
	
	/*2累计距离，判断是否到达终点*/
  Route_Distance_Updata(left_delta,right_delta);
	/*3当前速度给PID*/
	motorA.now=left_delta;
	motorB.now=right_delta;
	/*4清除本周期编码器计数*/
	Encoder_count1=0;
	Encoder_count2=0;
	/*5让状态机根据距离决定motor_target_set()*/
	Route_Task_10ms();

    // 如果状态机已经要求停车，就不要再让PID计算新的PWM。
    // 否则车到点后，编码器惯性脉冲会让位置式PID又算出输出，车就会继续往前蹭。
    if(motorA.target == 0 && motorB.target == 0)
    {
        pid_reset();
        motorA_duty(0);
        motorB_duty(0);
        return;
    }

	/*再跑速度PID*/
	//输入PID控制器进行计算
	pid_cal(&motorA);
	pid_cal(&motorB);
	//电机输出限幅
	pidout_limit(&motorA);
	pidout_limit(&motorB);
	//PID的输出值 输入给电机
	motorA_duty(motorA.out);
	motorB_duty(motorB.out);
	
  /*
	// 暂时不启用角度环，只刷新当前相对 yaw，后面需要时调用 angle_loop_cal()。
	angle.now = yaw_Kalman;
	// 速度环
	// 1.根据灰度传感器信息 设定目标速度
	//track();
	// 1.角度环PID输出 设定为速度环的目标值
	//motor_target_set(0,0); //-angle.out, angle.out
	// 2.获取当前速度
	if(motorA_dir){motorA.now = Encoder_count1;}else{motorA.now = -Encoder_count1;}
	if(motorB_dir){motorB.now = Encoder_count2;}else{motorB.now = -Encoder_count2;}
	Encoder_count1 = 0;
	Encoder_count2 = 0;
	
//	datavision_send();
   */
}
void pid_cal(pid_t *pid)
{
	// 计算当前偏差
	pid->error[0] = pid->target - pid->now;

	// 计算输出
	if(pid->pid_mode == DELTA_PID)  // 增量式
	{
		pid->pout = pid->p * (pid->error[0] - pid->error[1]);
		
	if(pid->out < 11000)
    pid->iout = pid->i * pid->error[0];
	else
    pid->iout = 0;
	
		pid->dout = pid->d * (pid->error[0] - 2 * pid->error[1] + pid->error[2]);
		pid->out += pid->pout + pid->iout + pid->dout;
	}
	else if(pid->pid_mode == POSITION_PID)
 {
    pid->pout = pid->p * pid->error[0];

    pid->iout += pid->i * pid->error[0];

    // 积分限幅，防止积分越积越大
    if(pid->iout > 4000) pid->iout = 4000;
    if(pid->iout < -4000) pid->iout = -4000;

    pid->dout = pid->d * (pid->error[0] - pid->error[1]);

    pid->out = pid->pout + pid->iout + pid->dout;
 }

	// 记录前两次偏差
	pid->error[2] = pid->error[1];
	pid->error[1] = pid->error[0];

	// 输出限幅
//	if(pid->out>=MAX_DUTY)	
//		pid->out=MAX_DUTY;
//	if(pid->out<=0)	
//		pid->out=0;
	
}

void pidout_limit(pid_t *pid)
{
	// 输出限幅
	if(pid->out>=25000)	
		pid->out=25000;
	if(pid->out<=0)	
		pid->out=0;
}
//MAX_DUTY

void pid_reset(void)
{
    motorA.out = 0;
    motorA.now = 0;
    motorA.target = 0;
    motorA.error[0] = motorA.error[1] = motorA.error[2] = 0;

    motorB.out = 0;
    motorB.now = 0;
    motorB.target = 0;
    motorB.error[0] = motorB.error[1] = motorB.error[2] = 0;
}
