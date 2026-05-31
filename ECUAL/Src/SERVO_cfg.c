/*
 * File: SERVO_cfg.c
 * Driver Name: [[ SERVO Motor ]]
 * SW Layer:   ECUAL
 * Created on: Feb 28, 2025
 * Author:     Embedded Team
 * -------------------------------------------
 * For More Information, Tutorials, etc.
 * Visit Website: www.DeepBlueMbedded.com
 *
 */

#include "../Inc/SERVO.h"

const SERVO_CfgType SERVO_CfgParam[SERVO_NUM] =
{
	// Servo Motor 1 Configurations
    {
	    GPIOB,
		GPIO_PIN_4,
		TIM3,
		&TIM3->CCR1,
		TIM_CHANNEL_1,
		72000000,
		0.65,
		2.3
	}
};


