/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * AUTHOR:  Mohammad Ghanatian, PhD Student, Department of Mechanical Engineering, The University of Alabama
 *
 *
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <stdbool.h>


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* Global Variables */
volatile uint8_t msg_ready = 0;
volatile uint8_t update_control_params = 0;
volatile uint8_t update_button=0;
volatile char uart_msg_buffer[50];
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
// limit data to be within bounds (specific to the AK80-9 motor)
#define P_MIN -12.5f	//position
#define P_MAX 12.5f
#define V_MIN -50.0f	//velocity
#define V_MAX 50.0f
#define T_MIN -18.0f	//torque
#define T_MAX 18.0f
#define Kp_MIN 0.0f
#define Kp_MAX 500.0f
#define Kd_MIN 0.0f
#define Kd_MAX 5.0f



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CAN1_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void CAN1_Tx_motor_on(int);
void CAN1_Tx_motor_zero(int);
void CAN1_Tx_motor_off(int);
//void CAN1_Rx_message_read(void);
float interpolateHipAngle(float);
float interpolateKneeAngle(float);
void Stop_Motors(int, int, int, int, int, int, int, int);


/* Function Prototypes */
void updateControlParameters();
void updateRamp(int motorIdx); // NEW: Prototype for updateRamp function



// Prototype of the template function

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;

uint8_t TxData[8];	//08 bytes of data transmit array, data type dictated by HAL
uint8_t RxData[8];	//08 bytes of data receive array, data type dictated by HAL

uint32_t TxMailbox;	//data type dictated by HAL




//Receiving parameter variables
//for motor 1
float position_1 = 0;
float velocity_1 = 0;
float torque_1 = 0;
float temp_1 = 0;
//for motor 2
float position_2 = 0;
float velocity_2 = 0;
float torque_2 = 0;
float temp_2 = 0;
//for motor 3
float position_3 = 0;
float velocity_3 = 0;
float torque_3 = 0;
float temp_3 = 0;
//for motor 4
float position_4 = 0;
float velocity_4 = 0;
float torque_4 = 0;
float temp_4 = 0;
//for motor 5
float position_5 = 0;
float velocity_5 = 0;
float torque_5 = 0;
float temp_5 = 0;
//for motor 6
float position_6 = 0;
float velocity_6 = 0;
float torque_6 = 0;
float temp_6 = 0;
//for motor 7
float position_7 = 0;
float velocity_7 = 0;
float torque_7 = 0;
float temp_7 = 0;
//for motor 8
float position_8 = 0;
float velocity_8 = 0;
float torque_8 = 0;
float temp_8 = 0;

float wPos1 = 0;
float wPos2 = 0;
float wPos3 = 0;
float wPos4 = 0;
float wPos5 = 0;
float wPos6 = 0;
float wPos7 = 0;
float wPos8 = 0;

float posF1 = 0;
float posF2 = 0;
float posF3 = 0;
float posF4 = 0;
float posF5 = 0;
float posF6 = 0;
float posF7 = 0;
float posF8 = 0;


float rAngle1=0;
float rAngle2=0;
float rAngle3=0;
float rAngle4=0;
float rAngle5=0;
float rAngle6=0;
float rAngle7=0;
float rAngle8=0;

float height1=0;
float X1=0;
float Y1=0;

float height2=0;
float X2=0;
float Y2=0;

float height3=0;
float X3=0;
float Y3=0;

float height4=0;
float X4=0;
float Y4=0;

//********************************************************************************





// PID variables
int motorID1 = 1;
int motorID2 = 2;
int motorID3 = 3;
int motorID4 = 4;
int motorID5 = 5;
int motorID6 = 6;
int motorID7 = 7;
int motorID8 = 8;

int motorActivated1 = 1;
int motorActivated2 = 1;
int motorActivated3 = 1;
int motorActivated4 = 1;
int motorActivated5 = 0;
int motorActivated6 = 0;
int motorActivated7 = 0;
int motorActivated8 = 0;

double Kp1=0;
double Kd1=0;

double Kp2=0;
double Kd2=0;

double Kp3=0;
double Kd3=0;

double Kp4=0;
double Kd4=0;

double Kp5=0;
double Kd5=0;

double Kp6=0;
double Kd6=0;

double Kp7=0;
double Kd7=0;

double Kp8=0;
double Kd8=0;





double error1, lastError1 = 0;
double integral1 = 0;
double derivative1;
float torqueF1 = 0;
float speedF1 = 0;
float lastPos1;
float torqSignal1 = 0;

double error2, lastError2 = 0;
double integral2 = 0;
double derivative2;
float torqueF2 = 0;
float speedF2 = 0;
float lastPos2;
float torqSignal2 = 0;

double error3, lastError3 = 0;
double integral3 = 0;
double derivative3;
float torqueF3 = 0;
float speedF3 = 0;
float lastPos3;
float torqSignal3 = 0;

double error4, lastError4 = 0;
double integral4 = 0;
double derivative4;
float torqueF4 = 0;
float speedF4 = 0;
float lastPos4;
float torqSignal4 = 0;

double error5, lastError5 = 0;
double integral5 = 0;
double derivative5;
float torqueF5 = 0;
float speedF5 = 0;
float lastPos5;
float torqSignal5 = 0;

double error6, lastError6 = 0;
double integral6 = 0;
double derivative6;
float torqueF6 = 0;
float speedF6 = 0;
float lastPos6;
float torqSignal6 = 0;

double error7, lastError7 = 0;
double integral7 = 0;
double derivative7;
float torqueF7 = 0;
float speedF7 = 0;
float lastPos7;
float torqSignal7 = 0;

double error8, lastError8 = 0;
double integral8 = 0;
double derivative8;
float torqueF8 = 0;
float speedF8 = 0;
float lastPos8;
float torqSignal8 = 0;

float startTime = 0;
float currentTime = 0;
float ttime = 0;
float ttime1 = 0;
float ttime2 = 0;



// Global constants for link lengths
const float l1 = 14.50 * 0.0254;
const float l2 = 13.50 * 0.0254;



uint32_t init_tick;
volatile uint32_t microseconds_elapsed = 0;
volatile uint32_t timer_overflows = 0;
float seconds_elapsed = 0;


double T = 2.0;
double beta = 0.75;
double L = 0.20;
double H = 0.1;


//********************************************************************************

float saturate(float x, float x_min, float x_max) {
	if (x < x_min)
		x = x_min;
	else if (x > x_max)
		x = x_max;
	return x;
}

//When sending packets, all numbers need to go through the following function to be converted into integer values before being sent to the motor
int float_to_uint(float x, float x_min, float x_max, unsigned int bits) {
	//Converts a float to an unsigned int, given range and number of intended bits
	float span = x_max - x_min;
	if (x < x_min)
		x = x_min;
	else if (x > x_max)
		x = x_max;
	return (int) ((x - x_min) * ((float) ((1 << bits) / span)));
}

//When receiving, convert all values to floating-point numbers using the following function
float uint_to_float(int x_int, float x_min, float x_max, int bits) {
	// converts unsigned int to float, given range and number of intended bits
	float span = x_max - x_min;
	float offset = x_min;
	return ((float) x_int) * span / ((float) ((1 << bits) - 1)) + offset;
}



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
  MX_USART2_UART_Init();
  MX_CAN1_Init();
  MX_TIM6_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	if (HAL_CAN_Start(&hcan1) != HAL_OK) //CAN is started and checked for error.
			{
		Error_Handler();
	}


	// Use a loop to initialize all 8 motors
	int motorIDs[] = {1, 2, 3, 4, 5, 6, 7, 8};

	for (int i = 0; i < 8; ++i) {
	    // 1. Send the motor ON command
	    CAN1_Tx_motor_on(motorIDs[i]);

	    // 2. Send the motor ZERO command to set its home position
	    CAN1_Tx_motor_zero(motorIDs[i]);

	}




	// Start timer
	HAL_TIM_Base_Start_IT(&htim6);
	HAL_TIM_Base_Start_IT(&htim2);





  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */



	  if (update_control_params) {
	      updateControlParameters();
	      update_control_params = 0;
	  }


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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 20;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 5;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

	//This is the CAN filter configuration (part of the CAN_INIT code)
	CAN_FilterTypeDef canfilterconfig;

	canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
	canfilterconfig.FilterBank = 0; // which filter bank to use from the assigned ones
	canfilterconfig.FilterFIFOAssignment = CAN_RX_FIFO0; //this has to be mentioned (FIFO0/ FIFO1)
	canfilterconfig.FilterIdHigh = 0; //Next 04 lines --> no filter mask protocol is going on. All messages are allowed to pass to Nucleo (Accept all frames)
	canfilterconfig.FilterIdLow = 0x0000;
	canfilterconfig.FilterMaskIdHigh = 0;
	canfilterconfig.FilterMaskIdLow = 0x0000;
	canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
	canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
	canfilterconfig.SlaveStartFilterBank = 0; // how many filters to assign to the CAN1 (master can)

	if (HAL_CAN_ConfigFilter(&hcan1, &canfilterconfig) != HAL_OK) {
		Error_Handler();
	}

    // New line to enable the CAN RX interrupt
    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
        Error_Handler();
    }

  /* USER CODE END CAN1_Init 2 */

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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 80-1;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */
  __HAL_TIM_ENABLE_IT(&htim2, TIM_IT_UPDATE);

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 4-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 50000-1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 921600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_RTS_CTS;
  huart2.Init.OverSampling = UART_OVERSAMPLING_8;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin PA6 */
  GPIO_InitStruct.Pin = LD2_Pin|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void CAN1_Tx_motor_on(int motor_id) {
	TxHeader.DLC = 8;	//data length
	TxHeader.ExtId = 0;
	TxHeader.IDE = CAN_ID_STD;	//standard 11 bit data frame, not extended.
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.StdId = motor_id;	//ID of the motor 01
	TxHeader.TransmitGlobalTime = DISABLE;

	TxData[0] = 0xFF;
	TxData[1] = 0xFF;
	TxData[2] = 0xFF;
	TxData[3] = 0xFF;
	TxData[4] = 0xFF;
	TxData[5] = 0xFF;
	TxData[6] = 0xFF;
	TxData[7] = 0xFC;

	while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0) {

	}

	if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK) {
		Error_Handler();
	}




}

void CAN1_Tx_motor_zero(int motor_id) {
	TxHeader.DLC = 8;	//data length
	TxHeader.ExtId = 0;
	TxHeader.IDE = CAN_ID_STD;	//standard 11 bit data frame, not extended.
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.StdId = motor_id;	//ID of the motor 01
	TxHeader.TransmitGlobalTime = DISABLE;

	TxData[0] = 0xFF;
	TxData[1] = 0xFF;
	TxData[2] = 0xFF;
	TxData[3] = 0xFF;
	TxData[4] = 0xFF;
	TxData[5] = 0xFF;
	TxData[6] = 0xFF;
	TxData[7] = 0xFE;

	while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0) {
		HAL_Delay(1);
	}

	if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK) {
		Error_Handler();
	}
}


void CAN1_Tx_motor_off(int motor_id) {
	TxHeader.DLC = 8;	//data length
	TxHeader.ExtId = 0;
	TxHeader.IDE = CAN_ID_STD;	//standard 11 bit data frame, not extended.
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.StdId = motor_id;	//ID of the motor 01
	TxHeader.TransmitGlobalTime = DISABLE;

	TxData[0] = 0xFF;
	TxData[1] = 0xFF;
	TxData[2] = 0xFF;
	TxData[3] = 0xFF;
	TxData[4] = 0xFF;
	TxData[5] = 0xFF;
	TxData[6] = 0xFF;
	TxData[7] = 0xFD;

	while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0) {
		HAL_Delay(1);
	}

	if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK) {
		Error_Handler();
	}

}

void CAN1_Tx_send_to_motor(int motor_id, float pos, float vel, float kp,
		float kd, float torque) {


	TxHeader.DLC = 8;	//data length 08 bytes
	TxHeader.ExtId = 0;
	TxHeader.IDE = CAN_ID_STD;	//standard 11 bit data frame, not extended.
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.StdId = motor_id;	//ID of the motor 01
	TxHeader.TransmitGlobalTime = DISABLE;

	//ensure that the values of pos, vel, kp, kd, and torque fall within specified bounds
	pos = fminf(fmaxf(P_MIN, pos), P_MAX);
	vel = fminf(fmaxf(V_MIN, vel), V_MAX);
	kp = fminf(fmaxf(Kp_MIN, kp), Kp_MAX);
	kd = fminf(fmaxf(Kd_MIN, kd), Kd_MAX);
	torque = fminf(fmaxf(T_MIN, torque), T_MAX);

	// convert floats to unsigned integers of intended bit size (int uses 4 bytes, 32 bits (same as floats) in this compiler).
	int p_int = float_to_uint(pos, P_MIN, P_MAX, 16);
	int v_int = float_to_uint(vel, V_MIN, V_MAX, 12);
	int kp_int = float_to_uint(kp, Kp_MIN, Kp_MAX, 12);
	int kd_int = float_to_uint(kd, Kd_MIN, Kd_MAX, 12);
	int t_int = float_to_uint(torque, T_MIN, T_MAX, 12);

	// pack integer values into the CAN transfer data array
	//here, the integer values are divided into 08 bit outputs
	TxData[0] = p_int >> 8; // Position High 8
	TxData[1] = p_int & 0xFF; // Position Low 8
	TxData[2] = v_int >> 4; // Speed High 8 bits
	TxData[3] = ((v_int & 0xF) << 4) | (kp_int >> 8); // Speed Low 4 bits, KP High 4 bits
	TxData[4] = kp_int & 0xFF; // KP Low 8 bits
	TxData[5] = kd_int >> 4; // Kd High 8 bits
	TxData[6] = ((kd_int & 0xF) << 4) | (t_int >> 8); // KP Low 4 bits, Torque High 4 bits
	TxData[7] = t_int & 0xff; // Torque Low 8 bits



	while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0) {
		HAL_Delay(1);
	}

	if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK) {
		Error_Handler();
	}
}


// New CAN receive callback function
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    // This function runs on a CAN interrupt. It must be non-blocking.
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) {
        // Unpack the received parameters
    	//Unpacking the received parameter integers from the RxData
    	int id = RxData[0]; //motor ID
    	int p_int = (RxData[1] << 8) | RxData[2];	//Motor position data
    	int v_int = (RxData[3] << 4) | (RxData[4] >> 4);	//Motor velocity data
    	int i_int = ((RxData[4] & 0xF) << 8) | RxData[5];	//Motor torque data
    	int T_int = RxData[6];	//temperature data
    	/// convert int to float
    	float p = uint_to_float(p_int, P_MIN, P_MAX, 16);
    	float v = uint_to_float(v_int, V_MIN, V_MAX, 12);
    	float i = uint_to_float(i_int, T_MIN, T_MAX, 12);
    	float T = T_int;


        // Convert and update the correct global variable based on the motor's ID
        switch (id) {
            case 1:
        		position_1 = p * 180.0 / M_PI / 65 * 12;
        		velocity_1 = v;
        		torque_1 = i;
        		temp_1 = T - 40;

                break;
            case 2:
        		position_2 = -p * 180.0 / M_PI / 65 * 12;
        		velocity_2 = v;
        		torque_2 = i;
        		temp_2 = T - 40;
                break;
            case 3:
        		position_3 = p * 180.0 / M_PI / 65 * 12;
        		velocity_3 = v;
        		torque_3 = i;
        		temp_3 = T - 40;
                break;
            case 4:
        		position_4 = -p * 180.0 / M_PI / 65 * 12;
        		velocity_4 = v;
        		torque_4 = i;
        		temp_4 = T - 40;
                break;
            case 5:
        		position_5 = p * 180.0 / M_PI / 65 * 12;
        		velocity_5 = v;
        		torque_5 = i;
        		temp_5 = T - 40;
                break;
            case 6:
        		position_6 = -p * 180.0 / M_PI / 65 * 12;
        		velocity_6 = v;
        		torque_6 = i;
        		temp_6 = T - 40;
                break;
            case 7:
        		position_7 = p * 180.0 / M_PI / 65 * 12;
        		velocity_7 = v;
        		torque_7 = i;
        		temp_7 = T - 40;
                break;
            case 8:
        		position_8 = -p * 180.0 / M_PI / 65 * 12;
        		velocity_8 = v;
        		torque_8 = i;
        		temp_8 = T - 40;
                break;
            default:
                break;
        }
    }
}





void signalMotors(float rAngle1, float rAngle2, float rAngle3, float rAngle4, float rAngle5, float rAngle6, float rAngle7, float rAngle8){


	if (motorActivated1) {
		CAN1_Tx_send_to_motor(motorID1, rAngle1, 0, Kp1, Kd1, 0);
	} else {
		CAN1_Tx_send_to_motor(motorID1, 0, 0, 0, 0, 0);
	}


	if (motorActivated2) {
		CAN1_Tx_send_to_motor(motorID2, -rAngle2, 0, Kp2, Kd2, 0);
	} else {
		CAN1_Tx_send_to_motor(motorID2, 0, 0, 0, 0, 0);
	}


	if (motorActivated3) {
		CAN1_Tx_send_to_motor(motorID3, rAngle3, 0, Kp3, Kd3, 0);
	} else {
		CAN1_Tx_send_to_motor(motorID3, 0, 0, 0, 0, 0);
	}


	if (motorActivated4) {
		CAN1_Tx_send_to_motor(motorID4, -rAngle4, 0, Kp4, Kd4, 0);
	} else {
		CAN1_Tx_send_to_motor(motorID4, 0, 0, 0, 0, 0);
	}


	if (motorActivated5) {
		CAN1_Tx_send_to_motor(motorID5, rAngle5, 0, Kp5, Kd5, 0);
	} else {
		CAN1_Tx_send_to_motor(motorID5, 0, 0, 0, 0, 0);
	}

	if (motorActivated6) {
		CAN1_Tx_send_to_motor(motorID6, -rAngle6, 0, Kp6, Kd6, 0);
	} else {
		CAN1_Tx_send_to_motor(motorID6, 0, 0, 0, 0, 0);
	}


	if (motorActivated7) {
		CAN1_Tx_send_to_motor(motorID7, rAngle7, 0, Kp7, Kd7, 0);
	} else {
		CAN1_Tx_send_to_motor(motorID7, 0, 0, 0, 0, 0);
	}

	if (motorActivated8) {
		CAN1_Tx_send_to_motor(motorID8, -rAngle8, 0, Kp8, Kd8, 0);
	} else {
		CAN1_Tx_send_to_motor(motorID8, 0, 0, 0, 0, 0);
	}

}





double AngleH1 = 0;
double AngleK1 = 0;
double AngleH2 = 0;
double AngleK2 = 0;
double AngleH3 = 0;
double AngleK3 = 0;
double AngleH4 = 0;
double AngleK4 = 0;






void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    update_control_params = 1;
    update_button = 1;
    if (htim->Instance == TIM2) {
        timer_overflows++;
    }
}



// Result struct (like MATLAB multiple outputs)
typedef struct {
    bool reachable;
    float theta1_down;
    float theta2_down;
    float theta1_up;
    float theta2_up;
} IKResult;

// IK function
IKResult IK(float x, float y, float theta2_min, float theta2_max)
{
//    const float l1 = 14.50*0.0254;
//    const float l2 = 13.50*0.0254;



    // Joint limits in radians
    const float theta1_min = -60 * M_PI/180; // -60 deg
    const float theta1_max =  60 * M_PI/180; //  60 deg
    theta2_min = theta2_min * M_PI/180;; // -130 deg
    theta2_max =  theta2_max * M_PI/180;; //  50 deg

    IKResult result;

    float r2 = x*x + y*y;

    // Check reachability in workspace
    if (r2 < (l1 - l2)*(l1 - l2) || r2 > (l1 + l2)*(l1 + l2)) {
        result.reachable = false;
        return result;
    }

    // Compute cos(theta2)
    float cos_theta2 = (r2 - l1*l1 - l2*l2) / (2.0f * l1 * l2);
    if (cos_theta2 > 1.0f) cos_theta2 = 1.0f;
    if (cos_theta2 < -1.0f) cos_theta2 = -1.0f;

    float theta2_down = acosf(cos_theta2);
    float theta2_up   = -acosf(cos_theta2);

    float solve_theta1(float theta2) {
        float phi = atan2f(x, -y);  // matches MATLAB fix
        float k1 = l1 + l2 * cosf(theta2);
        float k2 = l2 * sinf(theta2);
        float psi = atan2f(k2, k1);
        return phi - psi;
    }

    float theta1_down = solve_theta1(theta2_down);
    float theta1_up   = solve_theta1(theta2_up);

    // Clamp to joint limits
    if (theta1_down < theta1_min) theta1_down = theta1_min;
    if (theta1_down > theta1_max) theta1_down = theta1_max;
    if (theta1_up < theta1_min) theta1_up = theta1_min;
    if (theta1_up > theta1_max) theta1_up = theta1_max;

    if (theta2_down < theta2_min) theta2_down = theta2_min;
    if (theta2_down > theta2_max) theta2_down = theta2_max;
    if (theta2_up < theta2_min) theta2_up = theta2_min;
    if (theta2_up > theta2_max) theta2_up = theta2_max;

    result.theta1_down = theta1_down;
    result.theta2_down = theta2_down;
    result.theta1_up   = theta1_up;
    result.theta2_up   = theta2_up;
    result.reachable   = true;

    return result;
}




// Structure to hold the output of leg_xy
typedef struct {
    double x;
    double y;
} LegPosition;

// Function to compute the x and y coordinates for a single leg
LegPosition leg_xy(double t, double T, double beta, double L, double H, double phaseFrac) {
    LegPosition pos;

    double height = l1 + l2 - 0.2;

    double offset = phaseFrac * T;
    double local_t = fmod(t - offset, T);

    if (local_t < 0) {
        local_t += T; // Ensure positive modulo result
    }

    double v = L / (beta * T);

    if (local_t < beta * T) {
        // Stance phase
        pos.x = L / 2.0 - v * local_t;
        pos.y = -height;
    } else {
        // Swing phase
        double tau = (local_t - beta * T) / ((1 - beta) * T);
        pos.x = -L / 2.0 - v * tau + 3.0 * (L + v) * pow(tau, 2) - 2.0 * (L + v) * pow(tau, 3);
        pos.y = -height + H * (1.0 - cos(2.0 * M_PI * tau));
    }
    return pos;
}




// Structure to hold the outputs of getLegPositions
typedef struct {
    LegPosition lf;
    LegPosition rh;
    LegPosition rf;
    LegPosition lh;
} LegPositions;

// Main function to get all four leg positions
LegPositions getLegPositions(double t) {
    LegPositions all_pos;

//    // Global gait parameters
//    double T = 2.0;
//    double beta = 0.75;
//    double L = 0.20;
//    double H = 0.1;

    // Phase offsets
    double ph_LF = 0.00;
    double ph_RH = 0.25;
    double ph_RF = 0.50;
    double ph_LH = 0.75;

    // Gait selection
    int gait = 2; // 1: Walk, 2: Trot

    double l1 = 0.3124;
    double l2 = 0.34;
    double initial_height = l1 + l2 - 0.2;

    if (t <= 2.0) {
        // Initial configuration
        all_pos.lf.x = 0.0; all_pos.lf.y = -initial_height;
        all_pos.rh.x = 0.0; all_pos.rh.y = -initial_height;
        all_pos.rf.x = 0.0; all_pos.rf.y = -initial_height;
        all_pos.lh.x = 0.0; all_pos.lh.y = -initial_height;
    } else {
        if (gait == 1) { // Walk
            all_pos.lf = leg_xy(t, T, beta, L, H, ph_LF);
            all_pos.rh = leg_xy(t, T, beta, L, H, ph_RH);
            all_pos.rf = leg_xy(t, T, beta, L, H, ph_RF);
            all_pos.lh = leg_xy(t, T, beta, L, H, ph_LH);
        } else if (gait == 2) { // Trot
            all_pos.lf = leg_xy(t, T, beta, L, H, ph_LF);
            all_pos.rh = leg_xy(t, T, beta, L, H, ph_LF); // Same phase as LF
            all_pos.rf = leg_xy(t, T, beta, L, H, ph_RF);
            all_pos.lh = leg_xy(t, T, beta, L, H, ph_RF); // Same phase as RF
        }
    }

    return all_pos;
}





void updateControlParameters() {
	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_6);

	// Get a temporary copy of the variables to prevent race conditions
	uint32_t overflows = timer_overflows;
	uint32_t current_tick = __HAL_TIM_GET_COUNTER(&htim2);

	// Check for a pending overflow. If an overflow happens right after we read the counter
	// but before we read the overflow variable, our calculation will be off.
	// The TIM_SR_UIF bit is the Update Interrupt Flag.
	if (__HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_UPDATE) != RESET) {
		overflows++;
	}

	// Calculate total elapsed microseconds
	microseconds_elapsed = (overflows * 65536) + current_tick;
	// Note: The period of 65535 means it counts from 0 to 65535,
	// which is 65536 total ticks.
	seconds_elapsed = (float)microseconds_elapsed / 1000000.0f;



	LegPositions xyCoords = getLegPositions(seconds_elapsed);



	double Kp_stand=1.0;
	double Kd_stand=0.2;




    IKResult sol1 = IK(xyCoords.lh.x , xyCoords.lh.y , -130,50);

    AngleH1=sol1.theta1_up * 180.0f / M_PI;
    AngleK1=sol1.theta2_up * 180.0f / M_PI;

    Kp1=Kp_stand;
    Kd1=Kd_stand;
    Kp2=Kp_stand;
    Kd2=Kd_stand;



    IKResult sol2 = IK(-xyCoords.rh.x, xyCoords.rh.y, -50, 130);

    AngleH2=sol2.theta1_down * 180.0f / M_PI;
    AngleK2=sol2.theta2_down * 180.0f / M_PI;

    Kp3=Kp_stand;
    Kd3=Kd_stand;
    Kp4=Kp_stand;
    Kd4=Kd_stand;


    // **Limb 3 Control
    IKResult sol3 = IK(xyCoords.lh.x, xyCoords.lh.y, -50, 130);

    AngleH3=sol3.theta1_down * 180.0f / M_PI;
    AngleK3=sol3.theta2_down * 180.0f / M_PI;

    Kp5=Kp_stand;
    Kd5=Kd_stand;
    Kp6=Kp_stand;
    Kd6=Kd_stand;

    // **Limb 4 Control


    IKResult sol4 = IK(xyCoords.lh.x, xyCoords.lh.y, -130, 50);

    AngleH4=sol4.theta1_up * 180.0f / M_PI;
    AngleK4=sol4.theta2_up * 180.0f / M_PI;

    Kp7=Kp_stand;
    Kd7=Kd_stand;
    Kp8=Kp_stand;
    Kd8=Kd_stand;



	rAngle1 = AngleH1 * M_PI * 65 / (12 * 180.0);
	rAngle2 = AngleK1 * M_PI * 65 / (12 * 180.0);
	rAngle3 = AngleH2 * M_PI * 65 / (12 * 180.0);
	rAngle4 = AngleK2 * M_PI * 65 / (12 * 180.0);
	rAngle5 = AngleH3 * M_PI * 65 / (12 * 180.0);
	rAngle6 = AngleK3 * M_PI * 65 / (12 * 180.0);
	rAngle7 = AngleH4 * M_PI * 65 / (12 * 180.0);
	rAngle8 = AngleK4 * M_PI * 65 / (12 * 180.0);



    signalMotors(rAngle1, rAngle2, rAngle3, rAngle4, rAngle5, rAngle6, rAngle7, rAngle8);



    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_6);
}












void Stop_Motors(int motor1, int motor2, int motor3, int motor4, int motor5, int motor6, int motor7, int motor8) {

	if (motor1) {
		motorActivated1 = 0;
	}
	if (motor2) {
		motorActivated2 = 0;
	}
	if (motor3) {
		motorActivated3 = 0;
	}
	if (motor4) {
		motorActivated4 = 0;
	}
	if (motor5) {
		motorActivated5 = 0;
	}
	if (motor6) {
		motorActivated6 = 0;
	}
	if (motor7) {
		motorActivated7 = 0;
	}
	if (motor8) {
		motorActivated8 = 0;
	}
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
	char msg[50];
	sprintf(msg, "Error Handler Trigerred!\r\n");
	HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
	Stop_Motors(1, 1, 1, 1, 1, 1, 1, 1);

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
