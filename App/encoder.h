#ifndef _ENCODER_H
#define _ENCODER_H

#include "main.h"
#include "cmsis_os.h"
#include "tim.h"

#define ENCODER_TASK_TIMEOUT		1
#define WHEEL_RAD					34			//mm
#define ENCODER_STEP				13			//µ¥È¦±àÂëÆ÷Âö³å
#define MOTOR_REDUCTION_RETIO		30.0f		//¼õËÙ±È

#define PI							3.1415926535897932384626433832795f


typedef struct
{
	short step;
	short last_step;
	short err_step;
	
	float rate;
	float len;
	float last_len;
	float time_len;
}encoder_para_t;

typedef struct
{
	encoder_para_t encoder_para[2];
	uint16_t encoder_time_out;
}all_encoder_para_t;

void Encoder_task(void const * argument);
all_encoder_para_t *get_encoder_para_point(void);

#endif
