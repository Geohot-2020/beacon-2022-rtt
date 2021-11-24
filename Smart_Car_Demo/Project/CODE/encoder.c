#include "encoder.h"

#define ENCODER1_TIM		TIM_3
#define ENCODER1_A			TIM_3_ENC1_B04
#define ENCODER1_B			TIM_3_ENC2_B05

#define ENCODER2_TIM		TIM_4
#define ENCODER2_A			TIM_4_ENC1_B06
#define ENCODER2_B			TIM_4_ENC2_B07

uint16 speed_l, speed_r;

void encoder_init(void)
{
    tim_encoder_init(ENCODER1_TIM, ENCODER1_A, ENCODER1_B);
	tim_encoder_init(ENCODER2_TIM, ENCODER2_A, ENCODER2_B);
}


void encoder_get(void)
{
    speed_l = tim_encoder_get_count(ENCODER1_TIM);
    tim_encoder_rst(ENCODER1_TIM);
    speed_r = tim_encoder_get_count(ENCODER2_TIM);
    tim_encoder_rst(ENCODER2_TIM);
}