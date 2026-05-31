/*
 * File: DC_MOTOR_cfg.c
 * Driver Name: [ DC MOTOR ]
 * SW Layer:   ECUAL
 * Created on: 15/2/2025
 * Author:     Embedded Team
 *
 */

#include "../Inc/DC_MOTOR.h"

const DC_MOTOR_CfgType DC_MOTOR_CfgParam[DC_MOTOR_UNITS] =
{
	// DC MOTOR 1 Configurations
    {
	    GPIOA,
		GPIOA,
		GPIO_PIN_2, //L298_IN1
		GPIO_PIN_3, //L298_IN2
		TIM3,
		TIM_CHANNEL_2, //L298_ENA
		72,
		DC_MOTOR_F_PWM,
		DC_MOTOR_PWM_RES
	}
};
