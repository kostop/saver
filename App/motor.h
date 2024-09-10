#ifndef _MOTOR_H
#define _MOTOR_H

#include "main.h"
#include "cmsis_os.h"
#include "math.h"
#include "tim.h"
#include "pid.h"
#include "chassis.h"
#include "encoder.h"

#define MOTOR_TASK_TIMEOUT		2


#define Motor1_Cor()			Motor1_corotation()
#define Motor1_Rev()			Motor1_reversal()
#define Motor1_Stop()			Motor1_stop()
#define Motor2_Cor()			Motor2_corotation()
#define Motor2_Rev()			Motor2_reversal()
#define Motor2_Stop()			Motor2_stop()
#define Motor_Start()			Motor_start()

typedef struct
{
	pid_type_def motor_speed_pid;
	float rate;
	float set_rate;
	float angle;
	
	uint16_t set_compare;		//占空比
	uint8_t dirt;				//方向
}motor_para_t;

typedef struct
{
	chassis_para_t *motor_chassis_para;
	all_encoder_para_t *motor_encoder_para;
	motor_para_t motor_para[2];
}all_motor_para_t;

void motor_controller(motor_para_t *motor_para_speed_controller, uint8_t motor_num);
void Motor_task(void const * argument);
all_motor_para_t *get_all_motor_para_point(void);

#endif 
