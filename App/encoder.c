#include "encoder.h"

all_encoder_para_t all_encoder_para;

void encoder_init(all_encoder_para_t *encoder_para_init)
{
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_1|TIM_CHANNEL_2);
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_1|TIM_CHANNEL_2);
	
	encoder_para_init->encoder_time_out = ENCODER_TASK_TIMEOUT;
	
}

void encoder_feedback(all_encoder_para_t *encoder_para_feedback)
{
	encoder_para_feedback->encoder_para[0].step = (short)__HAL_TIM_GET_COUNTER(&htim3)/4;
	encoder_para_feedback->encoder_para[1].step = -(short)__HAL_TIM_GET_COUNTER(&htim4)/4;
}

void encoder_rate_count(encoder_para_t *encoder_para_rate_count, uint16_t task_time_out)
{
	encoder_para_rate_count->err_step = encoder_para_rate_count->step - encoder_para_rate_count->last_step;
	encoder_para_rate_count->rate = ((float)encoder_para_rate_count->err_step/ENCODER_STEP)/(task_time_out/1000.0f)/MOTOR_REDUCTION_RETIO;
	encoder_para_rate_count->last_step = encoder_para_rate_count->step;
}


void encoder_len_count(encoder_para_t *encoder_para_len_count, uint16_t task_time_out)
{
	encoder_para_len_count->len = encoder_para_len_count->step/(ENCODER_STEP*MOTOR_REDUCTION_RETIO)*WHEEL_RAD*2.0f*PI/1000.0f;
	encoder_para_len_count->time_len = ((encoder_para_len_count->len - encoder_para_len_count->last_len)/(task_time_out/1000.0f));
	encoder_para_len_count->last_len = encoder_para_len_count->len;
}

void encoder_control_loop(all_encoder_para_t *encoder_para_control_loop)
{
	for(int i=0; i<2; i++)
	{
		encoder_rate_count(&encoder_para_control_loop->encoder_para[i], ENCODER_TASK_TIMEOUT);
		encoder_len_count(&encoder_para_control_loop->encoder_para[i], ENCODER_TASK_TIMEOUT);
	}
}



void Encoder_task(void const * argument)
{
	encoder_init(&all_encoder_para);
	while(1)
	{
		encoder_feedback(&all_encoder_para);
		encoder_control_loop(&all_encoder_para);
		
		vTaskDelay(ENCODER_TASK_TIMEOUT);
	}
}

all_encoder_para_t *get_encoder_para_point(void)
{
	return &all_encoder_para;
}
