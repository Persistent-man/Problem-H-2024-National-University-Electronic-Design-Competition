#ifndef __ROUTE_H_
#define __ROUTE_H_

#include "headfile.h"

typedef enum
{
	Route_Wait_Start =0,
	Route_Adjust_At_A,
	Route_Run_AB,
	Route_Run_AC,
	Route_Stop_At_B,
	Route_Adjust_At_B,
	Route_Run_BC_ARC,
	Route_Run_CB_ARC,
	Route_Stop_At_C,
	Route_Adjust_At_C,
	Route_Run_CD,
	Route_Run_BD,
	Route_Stop_At_D,
	Route_Run_DA_ARC,
	Route_Mode4_Done,    //第4问跑满4圈后的最终停车状态
	Route_Finish
}Route_State_t;

void Route_SetMode(uint8_t mode);
uint8_t Route_GetMode(void);

void Route_Init(void);
void Route_Start(void);
void Route_Distance_reset(void);
void Route_Distance_Updata(int left_delta,int right_delta);//delta 变化差
float Route_Distance_cm(void);
Route_State_t route_get_state(void);
void Route_Task_10ms(void);


#endif
