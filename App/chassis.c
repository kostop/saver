#include "chassis.h"

#define CHASSIS_POS_KP				10.0f//10
#define CHASSIS_POS_KI				0.0f
#define CHASSIS_POS_KD				0.0f
#define CHASSIS_POS_MAX_OUT			0.8f
#define CHASSIS_POS_MAX_IOUT		0.0f

#define CHASSIS_ANGLE_KP			6.0f
#define CHASSIS_ANGLE_KI			0.0f
#define CHASSIS_ANGLE_KD			0.0f
#define CHASSIS_ANGLE_MAX_OUT		2.0f
#define CHASSIS_ANGLE_MAX_IOUT		0.0f

chassis_para_t chassis_para;

void chassis_init(chassis_para_t *chassis_para_init)
{
	chassis_para_init->chassis_encoder_para = get_encoder_para_point();
	
	float chassis_pos_pid[] = {CHASSIS_POS_KP, CHASSIS_POS_KI, CHASSIS_POS_KD};
	float chassis_angle_pid[] = {CHASSIS_ANGLE_KP, CHASSIS_ANGLE_KI, CHASSIS_ANGLE_KD};
	
	PID_init(&chassis_para_init->chassis_pos_pid, PID_POSITION, chassis_pos_pid, CHASSIS_POS_MAX_OUT, CHASSIS_POS_MAX_IOUT);
	PID_init(&chassis_para_init->chassis_angle_pid, PID_POSITION, chassis_angle_pid, CHASSIS_ANGLE_MAX_OUT, CHASSIS_ANGLE_MAX_IOUT);
	
	chassis_para_init->target_x = -0.6f;
	chassis_para_init->target_y = -0.6f;
	chassis_para_init->target_z = PI;
	
}

//运动正解算
void chassis_move_forward_decomposition(float *m1_set_rate, float *m2_set_rate, float vx_set, float wz_set)
{
	*m1_set_rate = vx_set - wz_set;
	*m2_set_rate = vx_set + wz_set;
}

//运动逆解算
void chassis_move_back_decomposition(float *x, float *y, float *z, float m1_len, float m2_len, float m1_time_len, float m2_time_len, uint16_t task_time_out)
{
	if(m1_time_len||m2_time_len != 0)
	{
		*z = ((m2_len - m1_len)/(CHASSIS_RADIUS*2.0f))*1000.0f;
		
		float sin_angle = sin(*z);
		float cos_angle = cos(*z);
		float move_len = ((m1_time_len + m2_time_len)/2.0f)*(task_time_out/1000.0f);
		
		*x += move_len * cos_angle;
		*y += move_len * sin_angle;
	}
}

//优劣弧运算
void chassis_arc_count(float *target_z, float *err_z)
{
	while(*err_z>PI)
	{
		*target_z -= 2.0f*PI;
		*err_z -=2.0f*PI;
	}
	while(*err_z<-PI)
	{
		*target_z += 2.0f*PI;
		*err_z +=2.0f*PI;
	}
}

//位置环计算
void chassis_position_count(chassis_para_t *chassis_para_position_count)
{
	chassis_para_position_count->err_x = chassis_para_position_count->target_x - chassis_para_position_count->x;
	chassis_para_position_count->err_y = chassis_para_position_count->target_y - chassis_para_position_count->y;
	chassis_para_position_count->err_z = chassis_para_position_count->target_z - chassis_para_position_count->z;
	chassis_para_position_count->target_err_angle = atan2(chassis_para_position_count->err_y, chassis_para_position_count->err_x);
	chassis_para_position_count->real_pos_len = sqrt(pow(chassis_para_position_count->x,2)+pow(chassis_para_position_count->y,2));
	chassis_para_position_count->target_pos_len = sqrt(pow(chassis_para_position_count->target_x,2)+pow(chassis_para_position_count->target_y,2));
	//转动标志位
	if(fabs(chassis_para_position_count->real_pos_len - chassis_para_position_count->target_pos_len)<CHASSIS_GET_TARGET_BUFFER)
		chassis_para_position_count->get_pos_state = 1;
	
	
	//未转到目标角度
	if(chassis_para_position_count->get_pos_state == 0)
	{
		chassis_para_position_count->wz_set = PID_calc(&chassis_para_position_count->chassis_angle_pid, chassis_para_position_count->z, chassis_para_position_count->target_err_angle);
		//移动到目标点
		chassis_para_position_count->vx_set = PID_calc(&chassis_para_position_count->chassis_pos_pid, chassis_para_position_count->real_pos_len, chassis_para_position_count->target_pos_len);
	}
	//转到目标角度
	else
	{
		chassis_arc_count(&chassis_para_position_count->target_z, &chassis_para_position_count->err_z);
		chassis_para_position_count->wz_set = PID_calc(&chassis_para_position_count->chassis_angle_pid, chassis_para_position_count->z, chassis_para_position_count->target_z);
		chassis_para_position_count->vx_set = 0.0f;
	}
}


void chassis_control_loop(chassis_para_t *chassis_control_loop_para)
{
	chassis_move_forward_decomposition(&chassis_control_loop_para->m1_set_rate, &chassis_control_loop_para->m2_set_rate, chassis_control_loop_para->vx_set, chassis_control_loop_para->wz_set);
	chassis_move_back_decomposition(&chassis_control_loop_para->x, &chassis_control_loop_para->y, &chassis_control_loop_para->z,
									chassis_control_loop_para->chassis_encoder_para->encoder_para[0].len, chassis_control_loop_para->chassis_encoder_para->encoder_para[1].len,
									chassis_control_loop_para->chassis_encoder_para->encoder_para[0].time_len, chassis_control_loop_para->chassis_encoder_para->encoder_para[1].time_len,
									CHASSIS_TASK_TIMEOUT);
	chassis_position_count(chassis_control_loop_para);
}



void Chassis_task(void const * argument)
{
	chassis_init(&chassis_para);
	while(1)
	{
		chassis_control_loop(&chassis_para);
		vTaskDelay(CHASSIS_TASK_TIMEOUT);
	}
}

chassis_para_t *get_chassis_para_point()
{
	return &chassis_para;
}

