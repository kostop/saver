#ifndef _CHASSIS_H
#define _CHASSIS_H

#include "main.h"
#include "cmsis_os.h"
#include "math.h"
#include "encoder.h"
#include "pid.h"

#define CHASSIS_TASK_TIMEOUT		2
#define CHASSIS_RADIUS				124		//mm

#define CHASSIS_GET_TARGET_BUFFER	0.02f	//底盘到达目的地阈值m

typedef struct
{
	all_encoder_para_t *chassis_encoder_para;
	pid_type_def	chassis_pos_pid;
	pid_type_def	chassis_angle_pid;
	
	//绝对坐标
	float x;
	float y;
	float z;
	//目标位置
	float target_x;
	float target_y;
	float target_z;
	//相对位置
	float err_x;
	float err_y;
	float err_z;
	float target_err_angle;
	
	float real_pos_len;
	float target_pos_len;
	uint8_t get_pos_state;
	uint8_t get_angle_state;
	
	//实际底盘速度
	float vx;
	float wz;
	
	//设定底盘速度
	float vx_set;
	float wz_set;
	
	float m1_set_rate;
	float m2_set_rate;
}chassis_para_t;

void Chassis_task(void const * argument);
chassis_para_t *get_chassis_para_point(void);

#endif
