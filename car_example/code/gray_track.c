#include "headfile.h"

void gray_init()
{
	gpio_init(GPIO_B, Pin_12, IU);   // D1
	gpio_init(GPIO_B, Pin_13, IU);   // D2
	gpio_init(GPIO_B, Pin_14, IU);   // D3
	gpio_init(GPIO_B, Pin_15, IU);   // D4
	gpio_init(GPIO_A, Pin_8, IU);    // D5
	gpio_init(GPIO_C, Pin_13, IU);   // D6
	gpio_init(GPIO_C, Pin_14, IU);   // D7
	gpio_init(GPIO_C, Pin_15, IU);   // D8
}

#define Base_speed 35
#define Turn_in_speed 0
#define Turn_out_speed 75
#define Search_in_speed 0
#define Search_out_speed 70
#define Max_turn 65

void track(void)
{
    static int last_error = 0;
    static int last_state = 0;     // -1 表示最近向左修正，1 表示最近向右修正；丢线时靠它决定找线方向。
    int d1;
    int d2;
    int d3;
    int d4;
    int d5;
    int d6;
    int d7;
    int d8;
    int error = 0;
    int count = 0;
    int black_count = 0;
    int left_edge = 0;
    int right_edge = 0;
    int turn;
    int left;
    int right;

    // 先统一采样一次，后面所有判断都用这一次结果，避免同一轮判断里读数变化造成抽动。
    d1 = D1;
    d2 = D2;
    d3 = D3;
    d4 = D4;
    d5 = D5;
    d6 = D6;
    d7 = D7;
    d8 = D8;

    // D1-D3 属于左侧，D6-D8 属于右侧；D4/D5 是中间循迹区域。
    if(d1 == 0) { black_count++; left_edge++; }
    if(d2 == 0) { black_count++; left_edge++; }
    if(d3 == 0) { black_count++; left_edge++; }
    if(d4 == 0) { black_count++; }
    if(d5 == 0) { black_count++; }
    if(d6 == 0) { black_count++; right_edge++; }
    if(d7 == 0) { black_count++; right_edge++; }
    if(d8 == 0) { black_count++; right_edge++; }
		
    if( d1 == 0&&d2 == 0&&d3 == 0&&d4 == 0 && right_edge < 2)
    {
        // 左侧外边缘压线，认为正在进入左直角；不加锁，只要当前状态满足就强转。
        last_state = -1;
        last_error = -35;
        motor_target_set(Turn_in_speed, Turn_out_speed);
        return;
    }

    if(d5 == 0&&d6 == 0&&d7 == 0 &&d8 == 0 && left_edge < 2)
    {
        // 右侧外边缘压线，认为正在进入右直角；不加锁，只要当前状态满足就强转。
        last_state = 1;
        last_error = 35;
        motor_target_set(Turn_out_speed, Turn_in_speed);
        return;
    }
    if(black_count == 0)
    {
        // 全白表示丢线，不使用锁定计时，只按上一次偏移方向继续慢速找线。
        if(last_state < 0)
            motor_target_set(Search_in_speed, Search_out_speed);
        else if(last_state > 0)
            motor_target_set(Search_out_speed, Search_in_speed);
        else
            motor_target_set(Base_speed, Base_speed);
        return;
    }

    if(black_count >= 7)
    {
        // 全黑或接近全黑时，普通 PD 会很乱；这里按最近方向找线，不加锁。
        if(last_state < 0)
            motor_target_set(Search_in_speed, Search_out_speed);
        else if(last_state > 0)
            motor_target_set(Search_out_speed, Search_in_speed);
        else
            motor_target_set(Base_speed, Base_speed);
        return;
    }

    if(d1 == 0) { error += -40; count++; }
    if(d2 == 0) { error += -25; count++; }
    if(d3 == 0) { error += -18; count++; }
    if(d4 == 0) { error += -8;  count++; }
    if(d5 == 0) { error += 8;   count++; }
    if(d6 == 0) { error += 18;  count++; }
    if(d7 == 0) { error += 25;  count++; }
    if(d8 == 0) { error += 40;  count++; }

    if(count > 0)
        error /= count;
    else
        error = last_error;
		
		
		
    // 普通循迹 PD。D 项取小一点，并限制最大转向量，减少左右摆动和抽搐。
    turn = 5 * error + 1 * (error - last_error);
    if(turn > Max_turn) turn = Max_turn;
    if(turn < -Max_turn) turn = -Max_turn;

    if(error < -5)
        last_state = -1;
    else if(error > 5)
        last_state = 1;

    last_error = error;

    left = Base_speed + turn;
    right = Base_speed - turn;

    if(left < 0) left = 0;
    if(right < 0) right = 0;
    if(left > 800) left = 800;
    if(right > 800) right = 800;

    motor_target_set(left, right);
}
unsigned char digtal(unsigned char channel)//1-8	  获取X通道数字值
{
	u8 value = 0;
	switch(channel) 
	{
		case 1:  
			if(gpio_get(GPIO_B, Pin_12) == 1) value = 1;
			else value = 0;  
			break;  
		case 2: 
			if(gpio_get(GPIO_B, Pin_13) == 1) value = 1;
			else value = 0;  
			break;  
		case 3: 
			if(gpio_get(GPIO_B, Pin_14) == 1) value = 1;
			else value = 0;  
			break;   
		case 4:  
			if(gpio_get(GPIO_B, Pin_15) == 1) value = 1;
			else value = 0;  
			break;   
		case 5:
			if(gpio_get(GPIO_A, Pin_8) == 1) value = 1;
			else value = 0;  
			break;
		case 6:  
			if(gpio_get(GPIO_C, Pin_13) == 1) value = 1;
			else value = 0;  
			break;  
		case 7: 
			if(gpio_get(GPIO_C, Pin_14) == 1) value = 1;
			else value = 0;  
			break;  
 		case 8: 
 			if(gpio_get(GPIO_C, Pin_15) == 1) value = 1;
 			else value = 0;  
 			break;   
	}
	return value; 
}

