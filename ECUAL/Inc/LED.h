/*
 * LED.h
 *
 *  Created on: Feb 5, 2025
 *      Author: Embedded Team
 */

#ifndef INC_LED_H_
#define INC_LED_H_

#include "../../Core/Inc/main.h"

void LED_Init(void);
void LED_ON(uint8_t ledInstance);
void LED_OFF(uint8_t ledInstance);

#endif /* INC_LED_H_ */
