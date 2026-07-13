#include "route.h"

#define Pulse_Per_cm 12.5f    		//每一厘米的脉冲
#define Distance_AB_cm 90.0f 			//A到B的距离 
#define Distance_BC_ARC_cm 170.0f //B到C圆弧的距离
#define Distance_CD_cm 90.0f 			//C到D的距离
#define Distance_DA_ARC_cm 196.0f //D到A圆弧的距离
#define Distance_AC_cm 133.0f     //A到C的距离
#define Distance_BD_cm 134.0f     //B到D的距离
/*目标角度*/
#define Angle_AB 0.0f             //第一问AB方向：A点上电清零后，AB就是0度
#define Angle_AC 50.0f            //目标AC角度
#define Angle_BD 135.0f           //目标BD角度
/*直线角度修正参数*/
#define Angle_Line_Kp  0.5f       
#define Angle_Line_Max_Correction 25
/*速度*/
#define Base_Speed 100         		//正常速度
/* 调整角度*/
#define C_Target_Angle 180.0f     //C点调车角度
#define C_Angle_Min  150.0f     //下阈值
#define C_Angle_Max  210.0f     //上阈值

#define Adjust_Angle_Tolerance  3.0f //180上下3都可以
#define Adjust_Max_Speed  35   //角度调整最大速度
#define Adjust_Min_Speed  15   //角度调整最小速度
#define Adjust_Stable_Tick 20  //连续稳定20个10ms 才行
#define Adjust_Timeout 250     //最多调2.5S

/*直线修正参数 左右轮计差多少*/
#define Straight_Kp 0.02f         
#define Max_Correction 10        
/*到点后停车保持的 10ms tick数 50tick代表500ms*/
#define Stop_Ticks 100 

#define Mode4_Total_Laps 4      //第4问要求跑4圈

static int g_Adjust_Stable_Tick=0;//稳定时间
static int g_Adjust_Timeout_Tick=0;//调整时间限制
static float g_Adjust_Target_Angle=0.0f;//调整角度参数
static Route_State_t g_Adjust_Next_State=Route_Finish;


static uint8_t g_mode=1;

static Route_State_t g_state=Route_Wait_Start;
static long g_Left_Total=0;
static long g_Right_Total=0;
static int g_Stop_Tick=0;
static uint8_t g_Lap_Count=0;   //第4问已经完成的圈数

static void Route_Task_Mode1(void);
static void Route_Task_Mode2(void);
static void Route_Task_Mode3(void);
static void Route_Task_Mode4(void);
static void Route_Task_OneLap_ACBDA(uint8_t repeat_enable);
static void track_right_arc(void);
static void track_left_arc(void);
static void Run_Left_ARC(float target_cm,Route_State_t next_stop_state);
static void Run_ARC(float target_cm,Route_State_t next_stop_state);
static void Run_Straight(float target_cm);
static void Run_Straight_With_Angle(float target_cm,float target_angle);
static void Start_Adjust_Angle(float target_angle,Route_State_t next_state);
static uint8_t Adjust_To_Angle_Task(void);
static void Finish_Adjust_To_Next(void);
static void Stop_Keep(void);

void Route_SetMode(uint8_t mode)//选择你想建立的模式
{
	if(mode<1)mode=1;
	if(mode>4)mode=1;
	g_mode=mode;
}

uint8_t Route_GetMode(void)    //获取当前模式
{
    return g_mode;
}

static int Limit_int(int value,int min_value,int max_value)//速度限制函数
{
	if(value<min_value)return min_value;
	if(value>max_value)return max_value;
	
	return value;
}
static long abs_long(long value)//绝对值函数1
{
	return (value<0)?-value:value;
}
static float abs_float(float value)//绝对值函数2
{
	return (value<0.0f)?-value:value;
}
void Route_Distance_reset()    //距离清零
{
	g_Left_Total=0;
	g_Right_Total=0;
}
void Route_Distance_Updata(int left_delta,int right_delta)//delta 变化差
{
	g_Left_Total+=left_delta;
	g_Right_Total+=right_delta;
}
float Route_Distance_cm(void)    //走了多少距离
{
	long left=abs_long(g_Left_Total);
	long right=abs_long(g_Right_Total);
	long avarage_distance=(left+right)/2;
	
	return (float)avarage_distance/Pulse_Per_cm;
}

static float Angle_Error(float target,float now)//角度差控制在-180到180
{
		float error=target-now;
		while(error>180.0f)error-=360.0f;
		while(error<-180.0f)error+=360.0f;
		return error;
}

void Route_Init(void)    //状态机初始化
{
	Route_Distance_reset();
	g_Stop_Tick=0;
	g_state = Route_Wait_Start;
}
void Route_Start(void)  //状态机开始
{
    Route_Distance_reset();
    g_Stop_Tick = 0;
    angle_loop_reset();    //按下开始键并等2秒后，把当前车头方向重新定为0度

	if(g_mode==3)
	{
		/* 第3问：如果A点车头按AB方向清零，就先在A点调到AC方向 */
		Start_Adjust_Angle(Angle_AC,Route_Run_AC);
		g_state=Route_Adjust_At_A;
	}
	else if(g_mode==4)
	{
		/*
			第4问：和第3问同路线，但要跑4圈。
			每次回到A点后，g_Lap_Count加1；不足4圈就重新从A点调到AC方向。
		*/
		g_Lap_Count=0;
		Start_Adjust_Angle(Angle_AC,Route_Run_AC);
		g_state=Route_Adjust_At_A;
	}
	else
	{
    g_state = Route_Run_AB;
	}
}
Route_State_t route_get_state(void)  //获取g_state
{
	return g_state;
}
static void Route_Beep_or_Led(void)  //声光提示
{
	/*放声光提示 例如：gpio_set(GPIO_B,Pin_5,1)*/
}
static void Run_Straight_With_Angle(float target_cm,float target_angle)
{
	float distance_cm;
	float angle_error;
	int correction;
	int left_speed;
	int right_speed;
	
	distance_cm=Route_Distance_cm();
	if(distance_cm>=target_cm)
	{
		motor_target_set(0,0);
		pid_reset();
		Route_Beep_or_Led();
		return ;
	}
	/*
	target_angle是这段希望保持的车头角度
	Angle_Error()会把误差限制在-180到180，避免181和-179被当成差很远
	*/
	angle_error=Angle_Error(target_angle,yaw_Kalman);
	correction=(int)(angle_error*Angle_Line_Kp);

	/*
		角度误差不小但 correction 被 int 截断成0时，车看起来就像没有角度环。
		所以误差超过2度时给一个最小修正量，帮助车头拉回来。
	*/
	if(correction == 0)
	{
		if(angle_error > 4.0f) correction = 5;
		else if(angle_error < -4.0f) correction = -5;
	}

	correction=Limit_int(correction,-Angle_Line_Max_Correction,Angle_Line_Max_Correction);
	/*
	正负号注意
	*/
	left_speed=Base_Speed+correction;
	right_speed=Base_Speed-correction;
	
	left_speed=Limit_int(left_speed,0,120);   //角度环直线需要留修正余量，不能卡在Base_Speed=100
	right_speed=Limit_int(right_speed,0,120);
	
	motor_target_set(left_speed+8,right_speed);
}
static void Run_Straight(float target_cm) //直线
{ /*用变量接值*/
	int base_speed;
	int correction;
	int left_speed;
	int right_speed;
	float distance_cm;
	float remain_cm;        /*目标与实际的差值*/
	long diff;
	
	base_speed=Base_Speed;
	
	distance_cm=Route_Distance_cm();
	
	remain_cm=target_cm-distance_cm;
	
	if(remain_cm<=0.0f)     /*到达位置后*/
	{
		motor_target_set(0,0);
		pid_reset();
		Route_Beep_or_Led();
		return;
	}
		
	/*左右轮累计修正直线*/
	diff=g_Left_Total-g_Right_Total;
	correction=(int)((float)diff*Straight_Kp);
	correction=Limit_int(correction,-Max_Correction,Max_Correction);
	
	left_speed=base_speed+correction+8;
	right_speed=base_speed-correction;
	
	left_speed=Limit_int(left_speed,0,180);
	right_speed=Limit_int(right_speed,0,180);
	
	motor_target_set(left_speed,right_speed);
}

static void Run_ARC(float target_cm,Route_State_t next_stop_state) //圆弧运行
{
	if(Route_Distance_cm()>=target_cm)
	{
		g_state=next_stop_state;
		g_Stop_Tick=Stop_Ticks;
		motor_target_set(0,0);
		Route_Beep_or_Led();
		return ;
	}
	track_right_arc();
}	

static void Start_Adjust_Angle(float target_angle,Route_State_t next_state)
{
	g_Adjust_Target_Angle=target_angle;
	g_Adjust_Next_State=next_state;
	g_Adjust_Stable_Tick=0;
	g_Adjust_Timeout_Tick=Adjust_Timeout;
}

static uint8_t Adjust_To_Angle_Task(void)    //调节角度
{
	float error;
	int turn_speed;
	if(g_Adjust_Timeout_Tick>0)g_Adjust_Timeout_Tick--;
	else return 1;//超时也放行 避免卡死
	
	error=Angle_Error(g_Adjust_Target_Angle,yaw_Kalman);//目标与现在的角度
  
	if(abs_float(error)<=Adjust_Angle_Tolerance)/*进入180容差后，还要稳定时间*/
	{
		motor_target_set(0,0);
		pid_reset();
		g_Adjust_Stable_Tick++;
		if(g_Adjust_Stable_Tick>=Adjust_Stable_Tick)return 1;
		
		return 0;
	}
	g_Adjust_Stable_Tick=0;
	
	turn_speed=(int)(abs_float(error)*0.8f);
	if(turn_speed>Adjust_Max_Speed)turn_speed=Adjust_Max_Speed;
	if(turn_speed<Adjust_Min_Speed)turn_speed=Adjust_Min_Speed;
	/*如果调节反就换位置（motor_target_set）*/
	if(error>0)
	{
		motor_target_set(turn_speed,-turn_speed);
	}
	else
	{
		motor_target_set(-turn_speed,+turn_speed);
	}
	
	return 0;
}
static void Finish_Adjust_To_Next(void)
{
	/* 调角完成后统一停车、清距离，再进入之前设好的下一状态 */
	motor_target_set(0,0);
	pid_reset();
	Route_Distance_reset();
	g_state=g_Adjust_Next_State;
}

static void Stop_Keep(void) //停止保持
{
	motor_target_set(0,0);
	
	if(g_Stop_Tick>0)
	{
		g_Stop_Tick--;
		return;
	}
}
void Route_Task_10ms(void) //轮询
{
	switch(g_mode)
	{
		case 1:
			Route_Task_Mode1();
		break;
		case 2:
			Route_Task_Mode2();
		break;
		case 3:
			Route_Task_Mode3();
		break;
		case 4:
			Route_Task_Mode4();
		break;
		
		default:
			motor_target_set(0,0);
			pid_reset();
		break;
	}
}
static void Route_Task_Mode1(void)
{
   switch(g_state)
   {
       case Route_Wait_Start:
            motor_target_set(0, 0);
            pid_reset();
       break;

       case Route_Run_AB:
            /* Mode1: A->B straight with angle loop */
            Run_Straight_With_Angle(Distance_AB_cm,Angle_AB);
            if(Route_Distance_cm() >= Distance_AB_cm)
            {
                g_state = Route_Stop_At_B;
                g_Stop_Tick = Stop_Ticks;
                motor_target_set(0, 0);
                pid_reset();
            }
       break;

       case Route_Stop_At_B:
            Stop_Keep();
            pid_reset();
            if(g_Stop_Tick<=0)
            {
                g_state = Route_Finish;
            }
       break;

       case Route_Finish:
            motor_target_set(0, 0);
            pid_reset();
       break;

       default:
            g_state = Route_Wait_Start;
            motor_target_set(0, 0);
            pid_reset();
       break;
   }
}

static void Route_Task_Mode2(void)
{
   switch(g_state)
   {
       case Route_Wait_Start:
            motor_target_set(0,0);
            pid_reset();
       break;

       case Route_Run_AB:
            Run_Straight(Distance_AB_cm);
            if(Route_Distance_cm() >= Distance_AB_cm)
            {
                g_state = Route_Stop_At_B;
                g_Stop_Tick = Stop_Ticks;
                motor_target_set(0, 0);
                pid_reset();
            }
       break;

       case Route_Stop_At_B:
            Stop_Keep();
            pid_reset();
            if(g_Stop_Tick<=0)
            {
                Route_Distance_reset();
                g_state=Route_Run_BC_ARC;
            }
       break;

       case Route_Run_BC_ARC:
            Run_ARC(Distance_BC_ARC_cm,Route_Stop_At_C);
       break;

       case Route_Stop_At_C:
            Stop_Keep();
            pid_reset();
            if(g_Stop_Tick<=0)
            {
                Start_Adjust_Angle(C_Target_Angle,Route_Run_CD);
                g_state=Route_Adjust_At_C;
            }
       break;

       case Route_Adjust_At_C:
            if(Adjust_To_Angle_Task())
            {
                Finish_Adjust_To_Next();
            }
       break;

       case Route_Run_CD:
            Run_Straight(Distance_CD_cm);
            if(Route_Distance_cm()>=Distance_CD_cm)
            {
                g_state=Route_Stop_At_D;
                g_Stop_Tick=Stop_Ticks;
                motor_target_set(0,0);
                pid_reset();
            }
       break;

       case Route_Stop_At_D:
            Stop_Keep();
            pid_reset();
            if(g_Stop_Tick<=0)
            {
                Route_Distance_reset();
                g_state=Route_Run_DA_ARC;
            }
       break;

       case Route_Run_DA_ARC:
            Run_ARC(Distance_DA_ARC_cm,Route_Finish);
       break;

       case Route_Finish:
            motor_target_set(0,0);
            pid_reset();
       break;

       default:
            g_state=Route_Wait_Start;
            motor_target_set(0,0);
            pid_reset();
       break;
   }
}

static void Route_Task_OneLap_ACBDA(uint8_t repeat_enable)
{
   switch(g_state)
   {
       case Route_Wait_Start:
            motor_target_set(0,0);
            pid_reset();
       break;

       case Route_Adjust_At_A:
            if(Adjust_To_Angle_Task())
            {
                Finish_Adjust_To_Next();
            }
       break;

       case Route_Run_AC:
            Run_Straight_With_Angle(Distance_AC_cm,Angle_AC);
            if(Route_Distance_cm()>=Distance_AC_cm)
            {
                g_state=Route_Stop_At_C;
                g_Stop_Tick=Stop_Ticks;
                motor_target_set(0,0);
                pid_reset();
            }
       break;

       case Route_Stop_At_C:
            Stop_Keep();
            pid_reset();
            if(g_Stop_Tick<=0)
            {
                Route_Distance_reset();
                g_state=Route_Run_CB_ARC;
            }
       break;

       case Route_Run_CB_ARC:
            Run_Left_ARC(Distance_BC_ARC_cm,Route_Stop_At_B);
       break;

       case Route_Stop_At_B:
            Stop_Keep();
            pid_reset();
            if(g_Stop_Tick<=0)
            {
                /* Reverted: at B, adjust directly to BD angle, no 180-degree calibration. */
                Start_Adjust_Angle(Angle_BD,Route_Run_BD);
                g_state=Route_Adjust_At_B;
            }
       break;

       case Route_Adjust_At_B:
            if(Adjust_To_Angle_Task())
            {
                Finish_Adjust_To_Next();
            }
       break;

       case Route_Run_BD:
            Run_Straight_With_Angle(Distance_BD_cm,Angle_BD);
            if(Route_Distance_cm()>=Distance_BD_cm)
            {
                g_state=Route_Stop_At_D;
                g_Stop_Tick=Stop_Ticks;
                motor_target_set(0,0);
                pid_reset();
            }
       break;

       case Route_Stop_At_D:
            Stop_Keep();
            pid_reset();
            if(g_Stop_Tick<=0)
            {
                Route_Distance_reset();
                g_state=Route_Run_DA_ARC;
            }
       break;

       case Route_Run_DA_ARC:
            Run_ARC(Distance_DA_ARC_cm,Route_Finish);
       break;

       case Route_Finish:
            motor_target_set(0,0);
            pid_reset();
            if(repeat_enable)
            {
                g_Lap_Count++;
                if(g_Lap_Count<Mode4_Total_Laps)
                {
                    Route_Distance_reset();
                    Start_Adjust_Angle(Angle_AC,Route_Run_AC);
                    g_state=Route_Adjust_At_A;
                }
                else
                {
                    g_state=Route_Mode4_Done;
                }
            }
       break;

       case Route_Mode4_Done:
            motor_target_set(0,0);
            pid_reset();
       break;

       default:
            g_state=Route_Wait_Start;
            motor_target_set(0,0);
            pid_reset();
       break;
   }
}

static void Route_Task_Mode3(void)
{
    Route_Task_OneLap_ACBDA(0);
}

static void Route_Task_Mode4(void)
{
    Route_Task_OneLap_ACBDA(1);
}
static void track_right_arc(void)
{
    // 右侧严重压线：强右转
    if(D6 == 0 && D7 == 0 && D8 == 0)
    {
        motor_target_set(60, 10);
        return;
    }

    if(D7 == 0 && D8 == 0)
    {
        motor_target_set(60, 12);
        return;
    }

    if(D6 == 0 && D7 == 0)
    {
        motor_target_set(60, 15);
        return;
    }

    if(D8 == 0)
    {
        motor_target_set(60, 12);
        return;
    }

    if(D7 == 0)
    {
        motor_target_set(56, 15);
        return;
    }

    if(D6 == 0)
    {
        motor_target_set(54, 18);
        return;
    }

    // 中间：正常右弯，不要直走
    if(D4 == 0 && D5 == 0 && D6 == 0)
    {
        motor_target_set(55, 22);
        return;
    }

    if(D4 == 0 && D5 == 0)
    {
        motor_target_set(52, 25);
        return;
    }

    if(D5 == 0)
    {
        motor_target_set(55, 22);
        return;
    }

    if(D4 == 0)
    {
        motor_target_set(50, 28);
        return;
    }

    // 左侧：右转太多，减弱右转或略向左修
    if(D1 == 0 && D2 == 0 && D3 == 0)
    {
        motor_target_set(35, 50);
        return;
    }

    if(D1 == 0 && D2 == 0)
    {
        motor_target_set(38, 48);
        return;
    }

    if(D2 == 0 && D3 == 0)
    {
        motor_target_set(42, 45);
        return;
    }

    if(D3 == 0 && D4 == 0)
    {
        motor_target_set(45, 40);
        return;
    }

    if(D1 == 0)
    {
        motor_target_set(40, 48);
        return;
    }

    if(D2 == 0)
    {
        motor_target_set(42, 45);
        return;
    }

    if(D3 == 0)
    {
        motor_target_set(45, 40);
        return;
    }

    // 全白丢线：不要太猛
    motor_target_set(60, 10);
}
static void track_left_arc(void)
{
	if(D1==0&&D2==0&&D3==0)
	{
		motor_target_set(10,60);
		return;
	}
	if(D1==0&&D2==0)
	{
		motor_target_set(12,58);
		return;
	}
	if(D2==0&&D3==0)
	{
		motor_target_set(15,56);
		return;
	}
	if(D1==0)
	{
		motor_target_set(12,58);
		return;
	}
	if(D2==0)
	{
		motor_target_set(15,56);
		return;
	}
	if(D3==0)
	{
		motor_target_set(18,54);
		return;
	}
	if(D4==0&&D5==0)
	{
		motor_target_set(25,52);
		return;
	}
	if(D4==0)
	{
		motor_target_set(22,56);
		return;
	}
	if(D5==0)
	{
		motor_target_set(28,50);
		return;
	}
	if(D6==0||D7==0||D8==0)
	{
		motor_target_set(48,38);
		return;
	}
	
	motor_target_set(10,60);
}
static void Run_Left_ARC(float target_cm,Route_State_t next_stop_state)
{
	if(Route_Distance_cm()>=target_cm)
	{
		g_state=next_stop_state;
		g_Stop_Tick=Stop_Ticks;
		motor_target_set(0,0);
		Route_Beep_or_Led();
		return;
	}
	track_left_arc();
}
