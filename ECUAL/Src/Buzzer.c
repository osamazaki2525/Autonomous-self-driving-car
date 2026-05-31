/*
 * Buzzer.c
 *
 *  Created on: Feb 27, 2025
 *      Author: Embedded Team
 */

#include "../Inc/Buzzer.h"

void Buzzer_Init(void){
	HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);
}

void Buzzer_ON(void){
	HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_SET);
}

void Buzzer_OFF(void){
	HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);
}




