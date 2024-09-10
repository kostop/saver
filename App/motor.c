#include "motor.h"

#define MOTOR1_SPEED_KP			800
#define MOTOR1_SPEED_KI			0
#define MOTOR1_SPEED_KD			0
#define MOTOR1_SPEED_MAX_OUT	4000
#define MOTOR1_SPEED_MAX_IOUT	50

#define MOTOR2_SPEED_KP			800
#define MOTOR2_SPEED_KI			0
#define MOTOR2_SPEED_KD			0
#define MOTOR2_SPEED_MAX_OUT	4000
#define MOTOR2_SPEED_MAX_IOUT	50

all_motor_para_t all_motor_para;

void Motor1_corotation(){
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
}
void Motor1_reversal(){
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
}
void Motor1_stop(){
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
}
void Motor2_corotation(){
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
}
void Motor2_reversal(){
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
}
void Motor2_stop(){
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
}
void motor_start()
{
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
}


void motor_init(all_motor_para_t *motor_para_init)
{
	motor_start();
	
	motor_para_init->motor_chassis_para = get_chassis_para_point();
	motor_para_init->motor_encoder_para = get_encoder_para_point();
	
	float motor1_speed_pid[] = {MOTOR1_SPEED_KP, MOTOR1_SPEED_KI, MOTOR1_SPEED_KD};
	float motor2_speed_pid[] = {MOTOR2_SPEED_KP, MOTOR2_SPEED_KI, MOTOR2_SPEED_KD};
	
	PID_init(&motor_para_init->motor_para[0].motor_speed_pid, PID_POSITION, motor1_speed_pid, MOTOR1_SPEED_MAX_OUT, MOTOR1_SPEED_MAX_IOUT);
	PID_init(&motor_para_init->motor_para[1].motor_speed_pid, PID_POSITION, motor2_speed_pid, MOTOR1_SPEED_MAX_OUT, MOTOR2_SPEED_MAX_IOUT);
}


void motor_feedback(all_motor_para_t *all_motor_para_feedback)
{
	all_motor_para_feedback->motor_para[0].set_rate = all_motor_para_feedback->motor_chassis_para->m1_set_rate;
	all_motor_para_feedback->motor_para[1].set_rate = all_motor_para_feedback->motor_chassis_para->m2_set_rate;
	
	all_motor_para_feedback->motor_para[0].rate = all_motor_para_feedback->motor_encoder_para->encoder_para[0].rate;
	all_motor_para_feedback->motor_para[1].rate = all_motor_para_feedback->motor_encoder_para->encoder_para[1].rate;
}


void Motor_task(void const * argument)
{
	motor_init(&all_motor_para);
	vTaskDelay(500);
	while(1)
	{
		motor_feedback(&all_motor_para);
		motor_controller(&all_motor_para.motor_para[0], 0);
		motor_controller(&all_motor_para.motor_para[1], 1);
		
		vTaskDelay(MOTOR_TASK_TIMEOUT);
	}
}



//面对车的前进方向
//左轮	0
//右轮	1
void motor_controller(motor_para_t *motor_para_controller, uint8_t motor_num)
{
	float compare_result = PID_calc(&motor_para_controller->motor_speed_pid, motor_para_controller->rate, motor_para_controller->set_rate);
	motor_para_controller->set_compare = (uint16_t)(fabs(compare_result));
	switch(motor_num)
	{
		case 0:
			if(compare_result>0)
				Motor1_Cor();
			else if(compare_result<0)
				Motor1_Rev();
			else
				Motor1_Stop();
			__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1,motor_para_controller->set_compare);
			break;
		case 1:
			if(compare_result>0)
				Motor2_Cor();
			else if(compare_result<0)
				Motor2_Rev();
			else
				Motor2_Stop();
			__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, motor_para_controller->set_compare);
			break;
	}
	
}


all_motor_para_t *get_all_motor_para_point(void)
{
	return &all_motor_para;
}



