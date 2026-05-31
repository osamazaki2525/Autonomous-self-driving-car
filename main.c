/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DC_MOTOR          0
#define SERVO_MOTOR       0

#define HCSR04_SENSOR1  0   //Left
#define HCSR04_SENSOR2  1   //Forward
#define HCSR04_SENSOR3  2   //Right
#define HCSR04_SENSOR4  3   //Right Blind Spot
#define HCSR04_SENSOR5  4   //Left Blind Spot
#define HCSR04_SENSOR6  5   //Backward

#define GREEN_LED       0
#define YELLOW_LED      1
#define RED_LED         2

#define THRESHOLD_DISTANCE   20.0  //in centimeters
#define SAFE_DISTANCE        30.0
#define ADAPTIVE_DISTANCE    40.0

#define ALARM_OFF            0
#define ALARM_LOW_LEVEL      1
#define ALARM_MEDIUM_LEVEL   2
#define ALARM_HIGH_LEVEL     3

#define FORWARD    			  'F'
#define BACKWARD   			  'B'
#define RIGHT      			  'R'
#define LEFT       			  'L'
#define STOP       			  'S'
#define OBSTACLE_AVOIDANCE    'O'
#define BLIND_SPOT            'D'
#define AUTO_PARKING		  'P'
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint8_t current_state = 0;
uint16_t TRIG_Ticks = 0;
uint8_t RX_Data = 0;
uint32_t Current_Time = 0;
float Speed = 0.0;
uint16_t current_speed = 0;
uint8_t received_char = 0;
uint8_t send_flag = 0;
volatile uint8_t obstacleAvoidanceActive = 1;
volatile uint8_t shouldStop = 0;  // Flag to indicate stop command received
typedef enum {
    IDLE,
    CHECK_LEFT_SPACE,
    CHECK_RIGHT_SPACE,
    ALIGNING_LEFT,
    ALIGNING_RIGHT,
    BACKWARD_LEFT,
    BACKWARD_RIGHT,
    FINAL_FORWARD,
    PARKED
} ParkingState;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
static void Sys_Init(void);
void SysTick_CallBack(void);
void move_forward(void);
void move_backward(void);
void move_backward_right(void);
void move_backward_left(void);
void move_right(void);
void move_left(void);
void stop(void);
void Alarm_Subsystem(uint8_t alarmLevel);
float Calculate_Speed(float current_distance, uint32_t current_time);
void Obstacle_Avoidance(float Front_Distance, float Right_Distance, float Left_Distance);
void BlindSpot_Detection(float BlindSpot_Right, float BlindSpot_Left);
void My_Auto_Parking(float Front_Distance, float Right_Distance, float Left_Distance, float Backward_Distance, float BlindSpot_Right, float BlindSpot_Left);
void My_Auto_Parking_FSM();
void UART_SendString(char *string);
void UART_SendFloat(float num);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  Sys_Init();
  float Front_Distance = 0.0, Right_Distance = 0.0, Left_Distance = 0.0, BlindSpot_Right = 0.0, BlindSpot_Left = 0.0, Backward_Distance = 0.0;
  UART_SendString("Welcome To ADAS!!\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  SysTick_CallBack();
	  if (HAL_UART_Receive(&huart1, &received_char, sizeof(received_char), 10) == HAL_OK)
	          {
	              // Check the received character
	              if (
	            		  (received_char == 'F') ||
						  (received_char == 'B') ||
						  (received_char == 'G') ||
						  (received_char == 'H') ||
						  (received_char == 'R') ||
						  (received_char == 'L') ||
					  	  (received_char == 'O') ||
						  (received_char == 'P') ||
						  (received_char == 'D') ||
						   received_char == 'S'
					 )
	              {
	            	  current_state = received_char;
	                  // Start sending the character repeatedly
	                  send_flag = 1;
	              }
	              else if (received_char == 'S')
	              {
	            	  current_state = received_char;
	                  // Stop sending the character
	                  send_flag = 0;
	              }
	              HAL_UART_Transmit(&huart1, &received_char, sizeof(received_char), 10);
	          }

	          // If the send flag is set, send the character repeatedly
	          if (send_flag)
	          {
	              uint8_t data_to_send = current_state;  // Character to send
	              switch(data_to_send)
	              	  {
	              	  	  case 'F':
	              	  	  {
	              	  		Front_Distance = HCSR04_Read(HCSR04_SENSOR2);
	              	  		if (Front_Distance > SAFE_DISTANCE)
	              	  		{
	              	  			Alarm_Subsystem(ALARM_LOW_LEVEL);
								UART_SendString("Moving Forward \r\n");
								UART_SendString("Distance = ");
								UART_SendFloat(Front_Distance+1);
								UART_SendString("\r\n");
	              	  			move_forward();
	              	  		}
	              	  		else if ((Front_Distance > THRESHOLD_DISTANCE) && (Front_Distance <= SAFE_DISTANCE))
	              	  		{
	              	  			Alarm_Subsystem(ALARM_MEDIUM_LEVEL);
								UART_SendString("Moving Forward \r\n");
								UART_SendString("Distance = ");
								UART_SendFloat(Front_Distance+1);
								UART_SendString("\r\n");
	              	  			move_forward();
	              	  		}
	              	  		else if (Front_Distance < THRESHOLD_DISTANCE)
	              	  		{
	              	  			Alarm_Subsystem(ALARM_HIGH_LEVEL);
								UART_SendString("Stopped \r\n");
								UART_SendString("Distance = ");
								UART_SendFloat(Front_Distance+1);
								UART_SendString("\r\n");
	              	  			stop();
	              	  		}
	              	  		break;
	              	  	  }

	              	  	  case 'B':
	              	  	  {
		              	  		Backward_Distance = HCSR04_Read(HCSR04_SENSOR6);
		              	  		if (Backward_Distance > SAFE_DISTANCE)
		              	  		{
		              	  			Alarm_Subsystem(ALARM_LOW_LEVEL);
									UART_SendString("Moving Forward \r\n");
									UART_SendString("Distance = ");
									UART_SendFloat(Backward_Distance+1);
									UART_SendString("\r\n");
		              	  			move_backward();
		              	  		}
		              	  		else if ((Backward_Distance > THRESHOLD_DISTANCE) && (Backward_Distance <= SAFE_DISTANCE))
		              	  		{
		              	  			Alarm_Subsystem(ALARM_MEDIUM_LEVEL);
									UART_SendString("Moving Forward \r\n");
									UART_SendString("Distance = ");
									UART_SendFloat(Backward_Distance+1);
									UART_SendString("\r\n");
		              	  			move_backward();
		              	  		}
		              	  		else if (Backward_Distance < THRESHOLD_DISTANCE)
		              	  		{
		              	  			Alarm_Subsystem(ALARM_HIGH_LEVEL);
									UART_SendString("Stopped \r\n");
									UART_SendString("Distance = ");
									UART_SendFloat(Backward_Distance+1);
									UART_SendString("\r\n");
		              	  			stop();
		              	  		}
		              	  		break;
	              	  	  }
	              	  	  case 'G':
	              	  	  {
	              	  		  move_backward_right();
	              	  		  UART_SendString("Moving backward in right direction\r\n");
	              	  		  break;
	              	  	  }
	              	  	  case 'H':
	              	  	  {
	              	  		  move_backward_left();
	              	  		  UART_SendString("Moving backward in left direction\r\n");
	              	  		  break;
	              	  	  }

	              	  	  case 'R':
	              	  	  {

	              	  		Right_Distance = HCSR04_Read(HCSR04_SENSOR1);
	              	  		if (Right_Distance > ADAPTIVE_DISTANCE)
	              	  		{
	              	  			Alarm_Subsystem(ALARM_LOW_LEVEL);
	              	  			UART_SendString("Moving Right\r\n");
	              	  			UART_SendString("Distance = ");
	              	  			UART_SendFloat(Right_Distance+1);
	              	  			UART_SendString("\r\n");
	              	  			move_right();
	              	  		}
	              	  		else if ((Right_Distance > SAFE_DISTANCE) && (Right_Distance <= ADAPTIVE_DISTANCE))
	              	  		{
	              	  			Alarm_Subsystem(ALARM_MEDIUM_LEVEL);
								UART_SendString("Moving Right\r\n");
								UART_SendString("Distance = ");
								UART_SendFloat(Right_Distance+1);
								UART_SendString("\r\n");
	              	  			move_right();
	              	  		}
	              	  		else if (Right_Distance < SAFE_DISTANCE)
	              	  		{
	              	  			Alarm_Subsystem(ALARM_HIGH_LEVEL);
								UART_SendString("Moving Right\r\n");
								UART_SendString("Distance = ");
								UART_SendFloat(Right_Distance+1);
								UART_SendString("\r\n");
	              	  			stop();
	              	  		}
	              			break;
	              	  	  }
	              	  	  case 'L':
	              	  	  {

	              	  		Left_Distance = HCSR04_Read(HCSR04_SENSOR3);
	              	  		if (Left_Distance > ADAPTIVE_DISTANCE)
	              	  		{
	              	  			Alarm_Subsystem(ALARM_LOW_LEVEL);
								UART_SendString("Moving Left\r\n");
								UART_SendString("Distance = ");
								UART_SendFloat(Left_Distance+1);
								UART_SendString("\r\n");
	              	  			move_left();
	              	  		}
	              	  		else if ((Left_Distance > SAFE_DISTANCE) && (Left_Distance <= ADAPTIVE_DISTANCE))
	              	  		{
	              	  			Alarm_Subsystem(ALARM_MEDIUM_LEVEL);
								UART_SendString("Moving Left\r\n");
								UART_SendString("Distance = ");
								UART_SendFloat(Left_Distance+1);
								UART_SendString("\r\n");
	              	  			move_left();
	              	  		}
	              	  		else if (Left_Distance < SAFE_DISTANCE)
	              	  		{
	              	  			Alarm_Subsystem(ALARM_HIGH_LEVEL);
								UART_SendString("Moving Left\r\n");
								UART_SendString("Distance = ");
								UART_SendFloat(Left_Distance+1);
								UART_SendString("\r\n");
	              	  			stop();
	              	  		}
	              			break;
	              	  	  }
	              	  	  case 'S':
	              	  	  {

	              	  		UART_SendString("Stopped\r\n");
	              			stop();
	              			Alarm_Subsystem(ALARM_OFF);
	              			break;
	              	  	  }

	              	  	case 'O':
							Right_Distance = HCSR04_Read(HCSR04_SENSOR1);
							Left_Distance = HCSR04_Read(HCSR04_SENSOR3);
							Front_Distance = HCSR04_Read(HCSR04_SENSOR2);
							Obstacle_Avoidance(Front_Distance, Right_Distance, Left_Distance);
							HAL_Delay(50);

							break;

	              	  	case 'D':
	              	  		BlindSpot_Right = HCSR04_Read(HCSR04_SENSOR4);
	              	  		BlindSpot_Left = HCSR04_Read(HCSR04_SENSOR5);
	              	  		BlindSpot_Detection(BlindSpot_Right, BlindSpot_Left);
	              	  		HAL_Delay(50);
	              	  		break;

	              	  	case 'P':
	              	  		My_Auto_Parking_FSM();
	              	  		HAL_Delay(50);
	              	  		break;

	              	  	default:
	              	  		break;

	              	  }
	              // Small delay to avoid flooding the UART
	              HAL_Delay(100);  // Adjust delay as needed
	          }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, L298_IN1_Pin|L298_IN2_Pin|LED_GREEN_Pin|LED_YELLOW_Pin
                          |LED_RED_Pin|Buzzer_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Trigger6_Pin|Trigger5_Pin|Trigger4_Pin|Trigger2_Pin
                          |Trigger3_Pin|Trigger1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : L298_IN1_Pin L298_IN2_Pin LED_GREEN_Pin LED_YELLOW_Pin
                           LED_RED_Pin Buzzer_Pin */
  GPIO_InitStruct.Pin = L298_IN1_Pin|L298_IN2_Pin|LED_GREEN_Pin|LED_YELLOW_Pin
                          |LED_RED_Pin|Buzzer_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : Trigger6_Pin Trigger5_Pin Trigger4_Pin Trigger2_Pin
                           Trigger3_Pin Trigger1_Pin */
  GPIO_InitStruct.Pin = Trigger6_Pin|Trigger5_Pin|Trigger4_Pin|Trigger2_Pin
                          |Trigger3_Pin|Trigger1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/*------------------------- (Initialization) ---------------------------------*/
static void Sys_Init(void)
{
    DC_MOTOR_Init(DC_MOTOR);
    SERVO_Init(SERVO_MOTOR);
	HCSR04_Init(HCSR04_SENSOR1, &htim4);
	HCSR04_Init(HCSR04_SENSOR2, &htim4);
	HCSR04_Init(HCSR04_SENSOR3, &htim4);
	HCSR04_Init(HCSR04_SENSOR4, &htim2);
	HCSR04_Init(HCSR04_SENSOR5, &htim4);
	HCSR04_Init(HCSR04_SENSOR6, &htim2);
    Buzzer_Init();
    LED_Init();
}
/*------------------------- (Initialization end) ---------------------------------*/


/*------------------------- (Interrupt handling start) ---------------------------------*/


void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	HCSR04_TMR_IC_ISR(htim);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
	HCSR04_TMR_OVF_ISR(htim);
}

void SysTick_CallBack(void)
{
	TRIG_Ticks++;
    if(TRIG_Ticks >= 10) // Each 10msec
    {
    	HCSR04_Trigger(HCSR04_SENSOR1);
    	HCSR04_Trigger(HCSR04_SENSOR2);
    	HCSR04_Trigger(HCSR04_SENSOR3);
    	HCSR04_Trigger(HCSR04_SENSOR4);
    	HCSR04_Trigger(HCSR04_SENSOR5);
    	HCSR04_Trigger(HCSR04_SENSOR6);
    	TRIG_Ticks = 0;
    }
}
/*------------------------- (Interrupt handling end) ---------------------------------*/

void move_forward(void)
{
	DC_MOTOR_Start(DC_MOTOR, DIR_CCW, 0XFFFF);
	SERVO_MoveTo(SERVO_MOTOR, 62);
}

void move_backward(void)
{
	DC_MOTOR_Start(DC_MOTOR, DIR_CW, 0XFFFF);
	SERVO_MoveTo(SERVO_MOTOR, 62);
}

void move_backward_right(void)
{
	DC_MOTOR_Start(DC_MOTOR, DIR_CW, 0XFFFF);
	SERVO_MoveTo(SERVO_MOTOR, 115);
}

void move_backward_left(void)
{
	DC_MOTOR_Start(DC_MOTOR, DIR_CW, 0XFFFF);
	SERVO_MoveTo(SERVO_MOTOR, 10);
}

void move_right(void)
{
	DC_MOTOR_Start(DC_MOTOR, DIR_CCW, 0XFFFF);
	SERVO_MoveTo(SERVO_MOTOR, 115);
}

void move_left(void)
{
	DC_MOTOR_Start(DC_MOTOR, DIR_CCW, 0XFFFF);
	SERVO_MoveTo(SERVO_MOTOR, 10);
}

void stop(void)
{
	DC_MOTOR_Start(DC_MOTOR, DIR_CW, 0);
	SERVO_MoveTo(SERVO_MOTOR, 62);
}

void Alarm_Subsystem(uint8_t alarmLevel) {
	if (ALARM_OFF == alarmLevel) {
		Buzzer_OFF();
		LED_OFF(GREEN_LED);
		LED_OFF(YELLOW_LED);
		LED_OFF(RED_LED);
	} else if (ALARM_LOW_LEVEL == alarmLevel) {
		Buzzer_OFF();
		LED_ON(GREEN_LED);
		LED_OFF(YELLOW_LED);
		LED_OFF(RED_LED);
	} else if (ALARM_MEDIUM_LEVEL == alarmLevel) {
		Buzzer_OFF();
		LED_OFF(GREEN_LED);
		LED_ON(YELLOW_LED);
		LED_OFF(RED_LED);
	} else if(ALARM_HIGH_LEVEL == alarmLevel){
		Buzzer_ON();
		LED_OFF(GREEN_LED);
		LED_OFF(YELLOW_LED);
		LED_ON(RED_LED);
	} else{
		/* Nothing */
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if(huart->Instance == USART1) {  // e.g., USART1, USART2, etc.
        char receivedChar = current_state;  // Where you store received bytes

        if(receivedChar == 'S') {
            shouldStop = 1;  // Set the stop flag
        }

        // Restart UART reception
        HAL_UART_Receive_IT(&huart1, &current_state, 1);
    }
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (current_state == 'S') {

        obstacleAvoidanceActive = 0;
        stop();
    }
}
void Obstacle_Avoidance(float Front_Distance, float Right_Distance, float Left_Distance) {
	if(shouldStop) {
	        shouldStop = 0;  // Reset the flag
	        stop();
	        return;
	    }

	    static uint8_t avoidanceState = 0; // 0=normal, 1=backing up, 2=turning
	    static uint32_t maneuverEndTime = 0;

	    // Check if we're in the middle of a maneuver
	    if(avoidanceState > 0) {
	        if(HAL_GetTick() < maneuverEndTime) {
	            // Check for stop command during maneuver too
	            if(shouldStop) {
	                shouldStop = 0;
	                stop();
	                return;
	            }
	            return; // Still executing maneuver
	        } else {
	            // Maneuver complete, return to normal state
	            avoidanceState = 0;
	            stop();
	            HAL_Delay(100); // Small delay to get fresh sensor readings
	            return;
	        }
	    }

    // Perform obstacle avoidance logic
    if (Front_Distance < SAFE_DISTANCE) {
        // Front obstacle detected - start avoidance maneuver
        UART_SendString("Front obstacle - moving backward\r\n");
        stop();
        move_backward();
        avoidanceState = 1;
        maneuverEndTime = HAL_GetTick() + 3000; // Back up for 3 seconds
    }
    else if (Right_Distance < SAFE_DISTANCE || Left_Distance < SAFE_DISTANCE) {
        // Side obstacle detected - choose best direction
        if (Right_Distance < Left_Distance) {
            UART_SendString("Right closer - moving left\r\n");
            move_left();
        } else {
            UART_SendString("Left closer - moving right\r\n");
            move_right();
        }
        avoidanceState = 2;
        maneuverEndTime = HAL_GetTick() + 3000; // Turn for 3 seconds
    }
    else {
        // No obstacle detected - move forward
        UART_SendString("Moving forward\r\n");
        UART_SendString("Distance = ");
        UART_SendFloat(Front_Distance);
        UART_SendString("\r\n");
        move_forward();
    }
}


void BlindSpot_Detection(float BlindSpot_Right, float BlindSpot_Left){
	//For blind spot detection
	if ((BlindSpot_Right > THRESHOLD_DISTANCE) && (BlindSpot_Right < SAFE_DISTANCE)){
		Alarm_Subsystem(ALARM_MEDIUM_LEVEL);
		UART_SendString("Take care of your right!! \r\n");
		UART_SendString("Your blind spot is not safe\r\n");
	} else if (BlindSpot_Right < THRESHOLD_DISTANCE){
		Alarm_Subsystem(ALARM_HIGH_LEVEL);
		UART_SendString("Danger on your right!!!\r\n");
	} else if ((BlindSpot_Left > THRESHOLD_DISTANCE) && (BlindSpot_Left < SAFE_DISTANCE)){
		Alarm_Subsystem(ALARM_MEDIUM_LEVEL);
		UART_SendString("Take care of your left!! \r\n");
		UART_SendString("Your blind spot is not safe\r\n");
	}  else if (BlindSpot_Left < THRESHOLD_DISTANCE){
		Alarm_Subsystem(ALARM_HIGH_LEVEL);
		UART_SendString("Danger on your left!!!\r\n");
	} else{
		UART_SendString("Your blind spots are safe\r\n");
		Alarm_Subsystem(ALARM_LOW_LEVEL);
	}
}

void My_Auto_Parking(float Front_Distance, float Right_Distance, float Left_Distance, float Backward_Distance, float BlindSpot_Right, float BlindSpot_Left)
{
	if ((BlindSpot_Left < THRESHOLD_DISTANCE) && (Left_Distance < THRESHOLD_DISTANCE))
	{
		UART_SendString("Moving Forward\r\n");
		move_forward();
		HAL_Delay(2000);
	}
	else if ((Left_Distance < THRESHOLD_DISTANCE) && (BlindSpot_Left > SAFE_DISTANCE))
	{
		UART_SendString("Moving Right\r\n");
		move_right();
		HAL_Delay(2000);
		UART_SendString("Moving Backward with left direction\r\n");
		move_backward_left();
		HAL_Delay(4000);
		if (Backward_Distance < SAFE_DISTANCE)
		{
			UART_SendString("Moving Backward with right direction\r\n");
			move_backward_right();
			HAL_Delay(3000);
			if (Backward_Distance < THRESHOLD_DISTANCE)
			{
				UART_SendString("Moving Forward\r\n");
				move_forward();
				HAL_Delay(50);
				UART_SendString("Parking Successfully\r\n");
			}
		}
	}

	else if ((BlindSpot_Right < THRESHOLD_DISTANCE) && (Right_Distance < THRESHOLD_DISTANCE))
		{
		UART_SendString("Moving Forward\r\n");
		move_forward();
		HAL_Delay(2000);
		}
		else if ((Right_Distance < THRESHOLD_DISTANCE) && (BlindSpot_Right > SAFE_DISTANCE))
		{
			UART_SendString("Moving Left\r\n");
			move_left();
			HAL_Delay(3000);
			UART_SendString("Moving Backward with right direction\r\n");
			move_backward_right();
			HAL_Delay(4000);
			if (Backward_Distance < SAFE_DISTANCE)
			{
				UART_SendString("Moving Backward with left direction\r\n");
				HAL_Delay(3000);
				if (Backward_Distance < THRESHOLD_DISTANCE)
				{
					UART_SendString("Moving Forward\r\n");
					HAL_Delay(50);
					UART_SendString("Parking Successfully\r\n");
				}
			}
		}

}


void My_Auto_Parking_FSM()
{
    static ParkingState state = IDLE;

    float Front_Distance 	= HCSR04_Read(HCSR04_SENSOR2);
    float Right_Distance 	= HCSR04_Read(HCSR04_SENSOR1);
    float Left_Distance 	= HCSR04_Read(HCSR04_SENSOR3);
    float Backward_Distance = HCSR04_Read(HCSR04_SENSOR6);
    float BlindSpot_Right 	= HCSR04_Read(HCSR04_SENSOR4);
    float BlindSpot_Left 	= HCSR04_Read(HCSR04_SENSOR5);

    switch(state)
    {
        case IDLE:
            if ((Left_Distance < THRESHOLD_DISTANCE) && (BlindSpot_Left > SAFE_DISTANCE))
            {
                UART_SendString("Left space detected. Aligning...\r\n");
                state = ALIGNING_LEFT;
            }
            else if ((Right_Distance < THRESHOLD_DISTANCE) && (BlindSpot_Right > SAFE_DISTANCE))
            {
                UART_SendString("Right space detected. Aligning...\r\n");
                state = ALIGNING_RIGHT;
            }
            break;

        case ALIGNING_LEFT:
            move_right();
            HAL_Delay(2000);
            stop();
            state = BACKWARD_LEFT;
            break;

        case ALIGNING_RIGHT:
            move_left();
            HAL_Delay(2000);
            stop();
            state = BACKWARD_RIGHT;
            break;

        case BACKWARD_LEFT:
            move_backward_left();
            HAL_Delay(3000);
            stop();
            Backward_Distance = HCSR04_Read(HCSR04_SENSOR6);

            if (Backward_Distance < SAFE_DISTANCE)
                state = BACKWARD_RIGHT;
            else
                state = FINAL_FORWARD;
            break;

        case BACKWARD_RIGHT:
            move_backward_right();
            HAL_Delay(3000);
            stop();
            Backward_Distance = HCSR04_Read(HCSR04_SENSOR6);

            if (Backward_Distance < THRESHOLD_DISTANCE)
                state = FINAL_FORWARD;
            else
                state = PARKED;
            break;

        case FINAL_FORWARD:
            move_forward();
            HAL_Delay(300);
            stop();
            state = PARKED;
            break;

        case PARKED:
            UART_SendString("Parking Completed Successfully\r\n");
            stop();
            // Reset to IDLE if you want to allow re-parking later
            // state = IDLE;
            break;
    }
}

void UART_SendString(char *string){
	uint16_t length = 0;
	while(string[length] != '\0'){
		length++;
	}

	HAL_UART_Transmit(&huart1, (uint8_t *)string, length, HAL_MAX_DELAY);
}
void UART_SendFloat(float num){
    char buffer[20];               // Buffer to hold the string representation of the float
    sprintf(buffer, "%.2f", num);  // Convert float to string with 2 decimal places
    UART_SendString(buffer);       // Send the string via UART
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
