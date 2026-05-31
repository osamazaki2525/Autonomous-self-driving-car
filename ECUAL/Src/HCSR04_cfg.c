/*
 * File: HCSR04_cfg.c
 * Driver Name: [ HC-SR04 Ultrasonic Sensor ]
 * SW Layer:   ECUAL
 * Created on: 16/2/2025
 * Author:     Embedded Team
 *
 */

#include "../Inc/HCSR04.h"

const HCSR04_CfgType HCSR04_CfgParam[HCSR04_UNITS] =
{
	// HC-SR04 Sensor Unit 1 Configurations
    {
		GPIOB,         //Trigger pin1 -->> B15
		GPIO_PIN_15,
		TIM4,
		TIM_CHANNEL_4, //Echo pin1 -->> B6
		72
	},
	// HC-SR04 Sensor Unit 2 Configurations
	{
		GPIOB,        //Trigger pin2 -->> B13
		GPIO_PIN_13,
		TIM4,
		TIM_CHANNEL_2, //Echo pin2 -->> B7
		72
	},
	// HC-SR04 Sensor Unit 3 Configurations
    {
		GPIOB,         //Trigger pin3 -->> B14
		GPIO_PIN_14,
		TIM4,
		TIM_CHANNEL_3, //Echo pin3 -->> B8
		72
	},
	// HC-SR04 Sensor Unit 4 Configurations
	{
		GPIOB,         //Trigger pin4 -->> B11
		GPIO_PIN_11,
		TIM2,
		TIM_CHANNEL_1, //Echo pin4 -->> A0
		72
	},
	// HC-SR04 Sensor Unit 5 Configurations
	{
		GPIOB,         //Trigger pin5 -->> B12
		GPIO_PIN_1,
		TIM4,
		TIM_CHANNEL_1, //Echo pin5 -->> A1
		72
	},
	// HC-SR04 Sensor Unit 6 Configurations
	{
		GPIOB,         //Trigger pin6 -->> B0
		GPIO_PIN_0,
		TIM2,
		TIM_CHANNEL_3, //Echo pin6 -->> B10
		72
	},

};
