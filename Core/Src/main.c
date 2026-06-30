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



/* SHTP & BNO085 Includes */
#include "sh2.h"
#include "sh2_util.h"
#include "euler.h"
#include "sh2_err.h"
#include "sh2_SensorValue.h"
#include "sh2_hal.h"


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

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
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



#define RESET_DELAY_US (10000)
#define START_DELAY_US (2000000)
#define DFU_BOOT_DELAY_US (50000)
#define DFU_CS_TIMING_US (20)
#define DFU_BYTE_TIMING_US (28)
#define DFU_CS_DEASSERT_DELAY_RX_US (0)
#define DFU_CS_DEASSERT_DELAY_TX_US (5000)
#define READ_LEN (4)

/* ==============================================================================
   PRIVATE ENUMS & TYPEDEFS
   ============================================================================== */
typedef enum SpiState_e {
    SPI_INIT,
    SPI_DUMMY,
    SPI_DFU,
    SPI_IDLE,
    SPI_RD_HDR,
    SPI_RD_BODY,
    SPI_WRITE
} SpiState_t;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CAN1_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM2_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM3_Init(void);
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

/* SPI HAL Prototypes */
sh2_Hal_t *sh2_hal_init(void);
sh2_Hal_t *dfu_hal_init(void);
static void spiActivate(void);
static void spiCompleted(void);
static uint32_t timeNowUs(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* SH2 Sensor Logic Variables */
float r, i, j, k, acc_deg, x, y, z;

// --- NEW GLOBAL IMU VARIABLES ---
volatile float roll_deg = 0.0f;
volatile float pitch_deg = 0.0f;
volatile float yaw_deg = 0.0f;


// --- Posture PID Control Variables ---
float Kd_bal_pitch = 0.0005f; // Derivative: Damps the motion
float Ki_bal_pitch = 0.001f; // Integral: Removes steady-state offset (start at 0)
float Kp_bal_pitch = 0.01f; // Proportional: Drives error to zero


float Kd_bal_roll = 0.0008f; // Derivative: Damps the motion
float Ki_bal_roll = 0.001f; // Integral: Removes steady-state offset (start at 0)
float Kp_bal_roll = 0.007f; // Proportional: Drives error to zero



// State Tracking
float pitch_integral = 0.0f, roll_integral = 0.0f;
float pitch_prev_error = 0.0f, roll_prev_error = 0.0f;
float last_pid_time = 0.0f;

// Output limits to prevent unexpected extreme motions (in meters)
float MAX_COMPENSATION = 0.1f; // Max 10cm adjustment per leg






// --- Yaw Filtering & Correction ---
float filtered_yaw = 0.0f;
float target_yaw = 0.0f;     // The heading we want to maintain



float alpha_yaw = 0.005f;     // Tuning parameter: start at 0.05. Lower is smoother.
float Kp_yaw = 0.01f;       // Proportional gain (keep this low so it corrects gently)
float delta_L_cm = 0.0f;


// --- Feedforward Variables ---
// 60 lbs (robot) + 20 lbs (payload) = 80 lbs total.
// 80 lbs = 36.3 kg. Total Force = 36.3 * 9.81 = 356 Newtons.
// Force per leg = 356 / 4 = 89 Newtons.
volatile float added_weight = 20.0f;

float last_ff_time = 0.0f;

// To store previous positions for velocity calculation
float prev_rA1=0, prev_rA2=0, prev_rA3=0, prev_rA4=0;
float prev_rA5=0, prev_rA6=0, prev_rA7=0, prev_rA8=0;


volatile float m_t1 = 0, m_t2 = 0, m_t3 = 0, m_t4 = 0, m_t5 = 0, m_t6 = 0, m_t7 = 0, m_t8 = 0;
volatile float v1 = 0, v2 = 0, v3 = 0, v4 = 0, v5 = 0, v6 = 0, v7 = 0, v8 = 0;



sh2_ProductIds_t prodIds;
sh2_Hal_t *pSh2Hal = 0;
bool resetOccurred = false;

/* SPI HAL State Variables */
static const uint8_t txZeros[SH2_HAL_MAX_TRANSFER_IN] = {0};
static SpiState_t spiState = SPI_INIT;
static volatile uint32_t rxTimestamp_us;
static volatile bool inReset;
static volatile bool rxReady;

/* SPI HAL Buffers */
static uint8_t rxBuf[SH2_HAL_MAX_TRANSFER_IN];
static volatile uint32_t rxBufLen;
static volatile bool rxDataReady;
static uint8_t txBuf[SH2_HAL_MAX_TRANSFER_OUT];
static uint32_t txBufLen;

/* SPI HAL Instances */
static sh2_Hal_t sh2Hal;
static sh2_Hal_t dfuHal;
static bool isOpen = false;




CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;

uint8_t TxData[8];	//08 bytes of data transmit array, data type dictated by HAL
uint8_t RxData[8];	//08 bytes of data receive array, data type dictated by HAL

uint32_t TxMailbox;	//data type dictated by HAL




// Receiving parameter variables
float position_1 = 0, position_2 = 0, position_3 = 0, position_4 = 0, position_5 = 0, position_6 = 0, position_7 = 0, position_8 = 0;
float velocity_1 = 0, velocity_2 = 0, velocity_3 = 0, velocity_4 = 0, velocity_5 = 0, velocity_6 = 0, velocity_7 = 0, velocity_8 = 0;
float torque_1 = 0, torque_2 = 0, torque_3 = 0, torque_4 = 0, torque_5 = 0, torque_6 = 0, torque_7 = 0, torque_8 = 0;
float temp_1 = 0, temp_2 = 0, temp_3 = 0, temp_4 = 0, temp_5 = 0, temp_6 = 0, temp_7 = 0, temp_8 = 0;

float wPos1 = 0, wPos2 = 0, wPos3 = 0, wPos4 = 0, wPos5 = 0, wPos6 = 0, wPos7 = 0, wPos8 = 0;
float posF1 = 0, posF2 = 0, posF3 = 0, posF4 = 0, posF5 = 0, posF6 = 0, posF7 = 0, posF8 = 0;
float rAngle1 = 0, rAngle2 = 0, rAngle3 = 0, rAngle4 = 0, rAngle5 = 0, rAngle6 = 0, rAngle7 = 0, rAngle8 = 0;



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
int motorActivated5 = 1;
int motorActivated6 = 1;
int motorActivated7 = 1;
int motorActivated8 = 1;

float Kp1=0.0f;
float Kd1=0.0f;

float Kp2=0.0f;
float Kd2=0.0f;

float Kp3=0.0f;
float Kd3=0.0f;

float Kp4=0.0f;
float Kd4=0.0f;

float Kp5=0.0f;
float Kd5=0.0f;

float Kp6=0.0f;
float Kd6=0.0f;

float Kp7=0.0f;
float Kd7=0.0f;

float Kp8=0.0f;
float Kd8=0.0f;





float error1, lastError1 = 0.0f;
float integral1 = 0.0f;
float derivative1=0.0f;
float torqueF1 = 0.0f;
float speedF1 = 0.0f;
float lastPos1=0.0f;
float torqSignal1 = 0.0f;

float error2, lastError2 = 0.0f;
float integral2 = 0.0f;
float derivative2=0.0f;
float torqueF2 = 0.0f;
float speedF2 = 0.0f;
float lastPos2=0.0f;
float torqSignal2 = 0.0f;

float error3, lastError3 = 0.0f;
float integral3 = 0.0f;
float derivative3=0.0f;
float torqueF3 = 0.0f;
float speedF3 = 0.0f;
float lastPos3=0.0f;
float torqSignal3 = 0.0f;

float error4, lastError4 = 0.0f;
float integral4 = 0.0f;
float derivative4 = 0.0f;
float torqueF4 = 0.0f;
float speedF4 = 0.0f;
float lastPos4 = 0.0f;
float torqSignal4 = 0.0f;

float error5, lastError5 = 0.0f;
float integral5 = 0.0f;
float derivative5 = 0.0f;
float torqueF5 = 0.0f;
float speedF5 = 0.0f;
float lastPos5 = 0.0f;
float torqSignal5 = 0.0f;

float error6, lastError6 = 0.0f;
float integral6 = 0.0f;
float derivative6 = 0.0f;
float torqueF6 = 0.0f;
float speedF6 = 0.0f;
float lastPos6 = 0.0f;
float torqSignal6 = 0.0f;

float error7, lastError7 = 0.0f;
float integral7 = 0.0f;
float derivative7 = 0.0f;
float torqueF7 = 0.0f;
float speedF7 = 0.0f;
float lastPos7 = 0.0f;
float torqSignal7 = 0.0f;

float error8, lastError8 = 0.0f;
float integral8 = 0.0f;
float derivative8 = 0.0f;
float torqueF8 = 0.0f;
float speedF8 = 0.0f;
float lastPos8 = 0.0f;
float torqSignal8 = 0.0f;

float startTime = 0.0f;
float currentTime = 0.0f;
float ttime = 0.0f;
float ttime1 = 0.0f;
float ttime2 = 0.0f;



// Global constants for link lengths
const float l1 = 0.35f;
const float l2 = 0.33f;
float initial_height = l1 + l2 - 0.2f;


uint32_t init_tick;

volatile uint32_t timer_overflows = 0;
volatile uint32_t imu_timer_overflows = 0;
float seconds_elapsed = 0.0f;


float T = 1.0f;
float beta = 0.65f;
float L = 0.2f;
float H = 0.15f;



float Kp_H=6.0f;
float Kd_H=0.8f;

float Kp_K=6.0f;
float Kd_K=0.8f;


// Gait selection
volatile int gait = 1; // 1: Walk, 2: Trot, Squat: 3

volatile int robot_command_state = 0; // 0: Halted/Reset, 1: Running, 2: Paused



// Squat parameters
volatile float lift_period = 3.0f;  //Squat Lift Period
volatile float h_crouch = 0.2f;    // 20cm (Dormant/Loading height)
volatile float h_stand  = 0.6f;    // 45cm (Max Standing height)home position



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






/* ==============================================================================
   SH2 APP LOGIC (Start Reports, Handlers, Callbacks)
   ============================================================================== */
static void startReports() {
    int status;
    static const struct {
        int sensorId;
        sh2_SensorConfig_t config;
    } sensorConfig[] = {
        // Game Rotation Vector, 100Hz
		{SH2_GAME_ROTATION_VECTOR, {.reportInterval_us = 3000}},

    };

    for (int n = 0; n < ARRAY_LEN(sensorConfig); n++) {
        int sensorId = sensorConfig[n].sensorId;
        status = sh2_setSensorConfig(sensorId, &sensorConfig[n].config);
        if (status != 0) {
            printf("Error while enabling sensor %d\n", sensorId);
        }
    }
}

static void eventHandler(void * cookie, sh2_AsyncEvent_t *pEvent) {
    if (pEvent->eventId == SH2_RESET) {
        resetOccurred = true;
    } else if (pEvent->eventId == SH2_SHTP_EVENT) {
        printf("EventHandler  id:SHTP, %d\n", pEvent->shtpEvent);
    } else if (pEvent->eventId == SH2_GET_FEATURE_RESP) {
        // Handled silently
    } else {
        printf("EventHandler, unknown event Id: %d\n", pEvent->eventId);
    }
}

static void printEvent(void * cookie, sh2_SensorEvent_t * event) {
    int rc;
    sh2_SensorValue_t value;
    float scaleRadToDeg = 180.0 / 3.14159265358;
    float t;
    static int skip = 0;

    rc = sh2_decodeSensorEvent(&value, event);
    if (rc != SH2_OK) {
        printf("Error decoding sensor event: %d\n\r", rc);
        return;
    }

    t = value.timestamp / 1000000.0;  // time in seconds.

    switch (value.sensorId) {
        case SH2_ACCELEROMETER:
            printf("%8.4f Acceleration| X: %6.2f | Y: %6.2f | Z: %6.2f\n\r",
                   (double)t, (double)value.un.accelerometer.x,
                   (double)value.un.accelerometer.y, (double)value.un.accelerometer.z);
            break;
        case SH2_GAME_ROTATION_VECTOR:
            r = value.un.gameRotationVector.real; // W
            i = value.un.gameRotationVector.i;    // X
            j = value.un.gameRotationVector.j;    // Y
            k = value.un.gameRotationVector.k;    // Z

            yaw_deg = atan2f(2.0f * (i * j + k * r), i * i - j * j - k * k + r * r) * scaleRadToDeg;
            pitch_deg = asinf(-2.0f * (i * k - j * r)) * scaleRadToDeg;
            roll_deg = atan2f(2.0f * (j * k + i * r), -i * i - j * j + k * k + r * r) * scaleRadToDeg;
            printf("%8.4f Orientation | Roll: %6.2f | Pitch: %6.2f | Yaw: %6.2f\n\r",
                   (double)t, (double)roll_deg, (double)pitch_deg, (double)yaw_deg);
            break;

        default:
            // Handled other sensors silently to save space
            break;
    }
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  int status;

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
  MX_SPI1_Init();
  MX_TIM3_Init();
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
	HAL_TIM_Base_Start_IT(&htim3);



    /* Create HAL instance and Open SH2 interface */
    pSh2Hal = sh2_hal_init();
    status = sh2_open(pSh2Hal, eventHandler, NULL);
    if (status != SH2_OK) {
        printf("Error, %d, from sh2_open.\n", status);
    }

    sh2_setSensorCallback(printEvent, NULL);
    startReports();
    printf("Hello World! The UART is working!\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


      if (resetOccurred) {
          resetOccurred = false;
          startReports();
      }
      sh2_service();
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
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 79;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  htim6.Init.Period = 60000-1;
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
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
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
  HAL_GPIO_WritePin(GPIOC, IMU0_RST_Pin|IMU0_P1_Pin|IMU0_P0_Pin|GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, IMU0_BT_Pin|IMU0_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : IMU0_RST_Pin IMU0_P1_Pin IMU0_P0_Pin PC4 */
  GPIO_InitStruct.Pin = IMU0_RST_Pin|IMU0_P1_Pin|IMU0_P0_Pin|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : IMU0_BT_Pin IMU0_CS_Pin */
  GPIO_InitStruct.Pin = IMU0_BT_Pin|IMU0_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : IMU0_INT_Pin */
  GPIO_InitStruct.Pin = IMU0_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(IMU0_INT_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

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
//		HAL_Delay(1);
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
//		HAL_Delay(1);
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
//		HAL_Delay(1);
	}

	if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK) {
		Error_Handler();
	}
}

/* ==============================================================================
   INTERRUPT CALLBACKS (Moved from spi_hal.c & main.c)
   ============================================================================== */
void HAL_GPIO_EXTI_Callback(uint16_t n) {
	if (n == IMU0_INT_Pin) {
		rxTimestamp_us = timeNowUs();
		inReset = false;
		rxReady = true;
		spiActivate();
	}
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef * hspi1) {
    if (isOpen) spiCompleted();
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef * hspi1) {
	printf("SPI Error Detected! Resetting state machine...\r\n");

	// 1. Abort the broken hardware transaction
	HAL_SPI_Abort_IT(hspi1);

	// 2. Force the software state machine back to IDLE
	spiState = SPI_IDLE;
	rxBufLen = 0;
	txBufLen = 0;
	rxReady = false;

	// 3. De-assert Chip Select to reset the IMU's SPI receiver
	HAL_GPIO_WritePin(IMU0_CS_GPIO_Port, IMU0_CS_Pin, GPIO_PIN_SET);

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
        		position_1 = p * 180.0 / M_PI / 80 * 15;
        		velocity_1 = v;
        		torque_1 = i;
        		temp_1 = T - 40;

                break;
            case 2:
        		position_2 = -p * 180.0 / M_PI / 80 * 15;
        		velocity_2 = v;
        		torque_2 = i;
        		temp_2 = T - 40;
                break;
            case 3:
        		position_3 = p * 180.0 / M_PI / 80 * 15;
        		velocity_3 = v;
        		torque_3 = i;
        		temp_3 = T - 40;
                break;
            case 4:
        		position_4 = -p * 180.0 / M_PI / 80 * 15;
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



void signalMotors(float rAngle1, float rAngle2, float rAngle3, float rAngle4, float rAngle5, float rAngle6, float rAngle7, float rAngle8,
                  float v1, float v2, float v3, float v4, float v5, float v6, float v7, float v8,
                  float t1, float t2, float t3, float t4, float t5, float t6, float t7, float t8){

	if (motorActivated1) CAN1_Tx_send_to_motor(motorID1, rAngle1, v1, Kp_H, Kd_H, t1);
	else CAN1_Tx_send_to_motor(motorID1, 0, 0, 0, 0, 0);

	if (motorActivated2) CAN1_Tx_send_to_motor(motorID2, -rAngle2, -v2, Kp_K, Kd_K, -t2);
	else CAN1_Tx_send_to_motor(motorID2, 0, 0, 0, 0, 0);

	if (motorActivated3) CAN1_Tx_send_to_motor(motorID3, rAngle3, v3, Kp_H, Kd_H, t3);
	else CAN1_Tx_send_to_motor(motorID3, 0, 0, 0, 0, 0);

	if (motorActivated4) CAN1_Tx_send_to_motor(motorID4, -rAngle4, -v4, Kp_K, Kd_K, -t4);
	else CAN1_Tx_send_to_motor(motorID4, 0, 0, 0, 0, 0);

	if (motorActivated5) CAN1_Tx_send_to_motor(motorID5, rAngle5, v5, Kp_H, Kd_H, t5);
	else CAN1_Tx_send_to_motor(motorID5, 0, 0, 0, 0, 0);

	if (motorActivated6) CAN1_Tx_send_to_motor(motorID6, -rAngle6, -v6, Kp_K, Kd_K, -t6);
	else CAN1_Tx_send_to_motor(motorID6, 0, 0, 0, 0, 0);

	if (motorActivated7) CAN1_Tx_send_to_motor(motorID7, rAngle7, v7, Kp_H, Kd_H, t7);
	else CAN1_Tx_send_to_motor(motorID7, 0, 0, 0, 0, 0);

	if (motorActivated8) CAN1_Tx_send_to_motor(motorID8, -rAngle8, -v8, Kp_K, Kd_K, -t8);
	else CAN1_Tx_send_to_motor(motorID8, 0, 0, 0, 0, 0);
}





float AngleH1 = 0.0f;
float AngleK1 = 0.0f;
float AngleH2 = 0.0f;
float AngleK2 = 0.0f;
float AngleH3 = 0.0f;
float AngleK3 = 0.0f;
float AngleH4 = 0.0f;
float AngleK4 = 0.0f;






void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM6) {
		updateControlParameters();
    }if (htim->Instance == TIM2) {
        timer_overflows++;
    }if (htim->Instance == TIM3) {
        imu_timer_overflows++;
    }
}



typedef struct {
    bool reachable;
    float theta1;
    float theta2;
} IKResult;

// IK function (Now only calculates the requested configuration)
IKResult IK(float x, float y, float theta2_min, float theta2_max, bool use_up_solution)
{
    const float theta1_min = -60.0f * M_PI/180.0f;
    const float theta1_max =  60.0f * M_PI/180.0f;
    theta2_min = theta2_min * M_PI/180.0f;
    theta2_max = theta2_max * M_PI/180.0f;

    IKResult result;

    float r2 = x*x + y*y;

    if (r2 < (l1 - l2)*(l1 - l2) || r2 > (l1 + l2)*(l1 + l2)) {
        result.reachable = false;
        return result;
    }

    float cos_theta2 = (r2 - l1*l1 - l2*l2) / (2.0f * l1 * l2);
    if (cos_theta2 > 1.0f) cos_theta2 = 1.0f;
    if (cos_theta2 < -1.0f) cos_theta2 = -1.0f;

    // ONLY calculate the requested solution
    float theta2 = use_up_solution ? -acosf(cos_theta2) : acosf(cos_theta2);

    float phi = atan2f(x, -y);
    float k1 = l1 + l2 * cosf(theta2);
    float k2 = l2 * sinf(theta2);
    float psi = atan2f(k2, k1);
    float theta1 = phi - psi;

    // Clamp to joint limits
    if (theta1 < theta1_min) theta1 = theta1_min;
    if (theta1 > theta1_max) theta1 = theta1_max;
    if (theta2 < theta2_min) theta2 = theta2_min;
    if (theta2 > theta2_max) theta2 = theta2_max;

    result.theta1 = theta1;
    result.theta2 = theta2;
    result.reachable = true;

    return result;
}




// Structure to hold the output of leg_xy
typedef struct {
    float x;
    float y;
} LegPosition;

// Function to compute the x and y coordinates for a single leg
LegPosition leg_xy(float t, float T, float beta, float L, float H, float phaseFrac) {
    LegPosition pos;

    float offset = phaseFrac * T;
    float local_t = fmodf(t - offset, T);

    if (local_t < 0.0f) {
        local_t += T;
    }

    float v = L / (beta * T);

    if (local_t < beta * T) {
        // Stance phase: The foot is on the ground.
        pos.x = L / 2.0f - v * local_t;
        // Apply the active PID stabilization offset here
        pos.y = -initial_height;
    } else {
        // Swing phase: The foot is in the air.
        float tau = (local_t - beta * T) / ((1.0f - beta) * T);
        float tau_sq = tau * tau;
        float tau_cub = tau_sq * tau;

        pos.x = -L / 2.0f - v * tau + 3.0f * (L + v) * tau_sq - 2.0f * (L + v) * tau_cub;

        float s = sinf(M_PI * tau);
        // Do NOT apply PID offsets here. Let the foot clear the ground naturally.
        pos.y = -initial_height + H * (s * s);
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
LegPositions getLegPositions(float t) {
    LegPositions all_pos;


	// Fixed Parameters
//	double initial_height = l1 + l2 - 0.2;

	// Phase offsets
	float ph_LF = 0.00;
	float ph_RH = 0.25;
	float ph_RF = 0.50;
	float ph_LH = 0.75;



	//1. FILTER THE RAW YAW ---
	// Update the filtered_yaw using the Exponential Moving Average formula
	filtered_yaw = (alpha_yaw * yaw_deg) + ((1.0f - alpha_yaw) * filtered_yaw);

	// --- 2. CALCULATE CORRECTION ---
	// Use the SMOOTHED yaw to find the error
	float yaw_error = target_yaw - filtered_yaw;

	// Calculate the stride adjustment
	float delta_L = Kp_yaw * yaw_error;
	delta_L_cm = delta_L * 100;


	// 1. Check for Home/Reset State
	if (t <= 2.0 || gait == 0) {
		all_pos.lf.x = 0.0; all_pos.lf.y = -initial_height;
		all_pos.rh.x = 0.0; all_pos.rh.y = -initial_height;
		all_pos.rf.x = 0.0; all_pos.rf.y = -initial_height;
		all_pos.lh.x = 0.0; all_pos.lh.y = -initial_height;
		return all_pos;
	}


	// 2. --- CALCULATE ACTIVE POSTURE PID OFFSETS ---
	float delta_y_pitch = 0.0f;
	float delta_y_roll = 0.0f;

	// Calculate PID continuously if we are in a gait that uses it
	if (gait == 5) {


		float dt = t - last_pid_time;
		if (dt <= 0.0) dt = 0.001;
		last_pid_time = t;

		float pitch_error = pitch_deg;
		float roll_error  = roll_deg;

		float P_pitch = Kp_bal_pitch * pitch_error;
		float P_roll  = Kp_bal_roll * roll_error;

		pitch_integral += pitch_error * dt;
		roll_integral  += roll_error * dt;

		pitch_integral = saturate(pitch_integral, -50.0, 50.0);
		roll_integral  = saturate(roll_integral, -50.0, 50.0);

		float I_pitch = Ki_bal_pitch * pitch_integral;
		float I_roll  = Ki_bal_roll * roll_integral;

		float pitch_deriv = (pitch_error - pitch_prev_error) / dt;
		float roll_deriv  = (roll_error - roll_prev_error) / dt;

		float D_pitch = Kd_bal_pitch * (pitch_deriv);
		float D_roll  = Kd_bal_roll * (roll_deriv);

		//		D_pitch = Kd_bal_pitch * (-gyro_pitch_rate);
		//		D_roll  = Kd_bal_roll * (-gyro_roll_rate);
		pitch_prev_error = pitch_error;
		roll_prev_error  = roll_error;

		delta_y_pitch = P_pitch + I_pitch + D_pitch;
		delta_y_roll  = P_roll  + I_roll  + D_roll;

		delta_y_pitch = saturate(delta_y_pitch, -MAX_COMPENSATION, MAX_COMPENSATION);
		delta_y_roll  = saturate(delta_y_roll, -MAX_COMPENSATION, MAX_COMPENSATION);


		// Apply Offsets to Legs (Y is negative going down)
		// Front Left (LF)
		all_pos.lf.x = 0.0;
		all_pos.lf.y = -initial_height + delta_y_pitch - delta_y_roll;

		// Right Front (RF)
		all_pos.rf.x = 0.0;
		all_pos.rf.y = -initial_height + delta_y_pitch + delta_y_roll;

		// Left Hind (LH)
		all_pos.lh.x = 0.0;
		all_pos.lh.y = -initial_height - delta_y_pitch - delta_y_roll;

		// Right Hind (RH)
		all_pos.rh.x = 0.0;
		all_pos.rh.y = -initial_height - delta_y_pitch + delta_y_roll;
		}




	// FORWARD WALK WITH YAW CORRETION
		if (gait == 6) { // Walk

			// Saturate for safety
			float max_delta_L = 0.2f;   // Max allowable stride change (in meters)
			delta_L = saturate(delta_L, -max_delta_L, max_delta_L);

			// Calculate independent left and right stride lengths
			float L_left = L - delta_L;
			float L_right = L + delta_L;

			// Feed the modified stride lengths to the respective legs
			all_pos.lf = leg_xy(t, T, beta, L_left, H, ph_LF, 0.0f);
			all_pos.lh = leg_xy(t, T, beta, L_left, H, ph_LH, 0.0f);

			all_pos.rf = leg_xy(t, T, beta, L_right, H, ph_RF, 0.0f);
			all_pos.rh = leg_xy(t, T, beta, L_right, H, ph_RH, 0.0f);
		}


		// BACKWARD WALK WITH YAW CORRECTION
		if (gait == -6) { // Walk backward with yaw correction

			// Saturate for safety
			float max_delta_L = 0.2f;   // Max allowable stride change (in meters)
			delta_L = saturate(delta_L, -max_delta_L, max_delta_L);

			// Calculate independent left and right stride lengths
			// NOTE: delta_L application is inverted (+ for left, - for right)
			// compared to forward walking. Steering backward requires the opposite
			// stride bias to correct the nose heading in the same direction.
			float L_left = L + delta_L;
			float L_right = L - delta_L;

			// Feed the modified stride lengths and reversed phases (from gait -1)
			all_pos.lf = leg_xy(t, T, beta, L_left, H, ph_LH, 0.0f);
			all_pos.rh = leg_xy(t, T, beta, L_right, H, ph_RF, 0.0f);
			all_pos.rf = leg_xy(t, T, beta, L_right, H, ph_RH, 0.0f);
			all_pos.lh = leg_xy(t, T, beta, L_left, H, ph_LF, 0.0f);

			// Invert the x-axis to physically move backward (from gait -1)
			all_pos.lf.x = -all_pos.lf.x;
			all_pos.rh.x = -all_pos.rh.x;
			all_pos.rf.x = -all_pos.rf.x;
			all_pos.lh.x = -all_pos.lh.x;
		}



	// 2. Calculate Raw Leg Positions (Local Trajectories)
	if (gait == 1) { // Walk
		all_pos.lf = leg_xy(t, T, beta, L, H, ph_LF, 0.0f);
		all_pos.rh = leg_xy(t, T, beta, L, H, ph_RH, 0.0f);
		all_pos.rf = leg_xy(t, T, beta, L, H, ph_RF, 0.0f);
		all_pos.lh = leg_xy(t, T, beta, L, H, ph_LH, 0.0f);
	}
	if (gait == -1) { // Walk backward
		all_pos.lf = leg_xy(t, T, beta, L, H, ph_LH, 0.0f);
		all_pos.rh = leg_xy(t, T, beta, L, H, ph_RF, 0.0f);
		all_pos.rf = leg_xy(t, T, beta, L, H, ph_RH, 0.0f);
		all_pos.lh = leg_xy(t, T, beta, L, H, ph_LF, 0.0f);


		all_pos.lf.x = -all_pos.lf.x;
		all_pos.rh.x = -all_pos.rh.x;
		all_pos.rf.x = -all_pos.rf.x;
		all_pos.lh.x = -all_pos.lh.x;
	}
	if (gait == 2) { // Trot
		all_pos.lf = leg_xy(t, T, beta, L, H, ph_LF, 0.0f);
		all_pos.rh = leg_xy(t, T, beta, L, H, ph_LF, 0.0f);
		all_pos.rf = leg_xy(t, T, beta, L, H, ph_RF, 0.0f);
		all_pos.lh = leg_xy(t, T, beta, L, H, ph_RF, 0.0f);
	}
	if (gait == 3) { //  Heavy Lift Squat Test

		// --- SATURATION LOGIC ---
		// Define physical limits based on leg lengths (l1+l2 = 0.71m)
		// We use 0.60m to keep a safety margin from the singularity.
		float h_min_limit = 0.10;
		float h_max_limit = 0.70;

		// Apply saturation using fmin/fmax (standard math.h functions)
		// This ensures the variables stay within [h_min_limit, h_max_limit]
		h_crouch = fmax(h_min_limit, fmin(h_crouch, h_max_limit));
		h_stand  = fmax(h_min_limit, fmin(h_stand, h_max_limit));
		// ------------------------

	    // Calculate current target height using a Cosine wave
	    // At t=0, cos(0)=1 -> Height = avg + amp = h_stand (Fully Standing)
	    // At t=5, cos(pi)=-1 -> Height = avg - amp = h_crouch (Starts Low)

		float amp = (h_stand - h_crouch) / 2.0f;
		float avg = (h_stand + h_crouch) / 2.0f;
		float phase = 2.0f * M_PI * (t / lift_period);

		float current_h = avg + amp * cos(phase);

	    // Command all 4 legs to this height (Feet stay at x=0 relative to hip)
	    all_pos.lf.x = 0.0f; all_pos.lf.y = -current_h;
	    all_pos.rh.x = 0.0f; all_pos.rh.y = -current_h;
	    all_pos.rf.x = 0.0f; all_pos.rf.y = -current_h;
	    all_pos.lh.x = 0.0f; all_pos.lh.y = -current_h;
	}

	if (gait == 4) {
		// Front legs (Targets 22 deg, 122 deg) -> X = -0.210m, Y = -0.273m
		all_pos.lf.x = -0.210f; all_pos.lf.y = -0.273f;
		all_pos.rf.x = -0.210f; all_pos.rf.y = -0.273f;

		// Back legs (Targets 36 deg, 125 deg) -> X = 0.138m, Y = -0.297m
		all_pos.lh.x =  0.138f; all_pos.lh.y = -0.297f;
		all_pos.rh.x =  0.138f; all_pos.rh.y = -0.297f;
		return all_pos;
	}



	return all_pos;
}





void updateControlParameters() {
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_4);





	switch (robot_command_state) {
		case 0: // Halted/Reset State
			HAL_TIM_Base_Stop_IT(&htim2);       // Stop the hardware timer
			__HAL_TIM_SET_COUNTER(&htim2, 0);   // Reset hardware counter
			timer_overflows = 0;                // Reset software counter
			seconds_elapsed = 0.0f;             // Reset final time
			// You could also command the motors to a "home" position here
			// For now, we'll just stop everything.
			return; // Exit the function

		case 2: // Paused State
			HAL_TIM_Base_Stop_IT(&htim2);       // Stop the timer
			return; // Exit, freezing the robot in its current position

		case 1: // Running State
		default:
			HAL_TIM_Base_Start_IT(&htim2);      // Ensure timer is running
			break; // Proceed with the rest of the function
	}




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
	uint32_t microseconds_elapsed = (overflows * 65536) + current_tick;
	// Note: The period of 65535 means it counts from 0 to 65535,
	// which is 65536 total ticks.
	seconds_elapsed = (float)microseconds_elapsed / 1000000.0f;



	LegPositions xyCoords = getLegPositions(seconds_elapsed);



    IKResult sol1 = IK(xyCoords.rf.x , xyCoords.rf.y , -130,50, true);

    AngleH1=sol1.theta1 * 180.0f / M_PI - 22.0f;
    AngleK1=sol1.theta2 * 180.0f / M_PI + 122.0f;


    IKResult sol2 = IK(-xyCoords.lf.x, xyCoords.lf.y, -50, 130, false);

    AngleH2=sol2.theta1 * 180.0f / M_PI + 22.0f;
    AngleK2=sol2.theta2 * 180.0f / M_PI - 122.0f;




    // **Limb 3 Control
    IKResult sol3 = IK(xyCoords.rh.x, xyCoords.rh.y, -50, 130, false);

    AngleH3=sol3.theta1 * 180.0f / M_PI + 36.0f;
    AngleK3=sol3.theta2 * 180.0f / M_PI - 125.0f;


    // **Limb 4 Control


    IKResult sol4 = IK(-xyCoords.lh.x, xyCoords.lh.y, -130, 50, true);

    AngleH4=sol4.theta1 * 180.0f / M_PI - 36.0f;
    AngleK4=sol4.theta2 * 180.0f / M_PI + 125.0f;




	rAngle1 = AngleH1 * M_PI * 80 / (15 * 180.0);
	rAngle2 = AngleK1 * M_PI * 80 / (15 * 180.0);
	rAngle3 = AngleH2 * M_PI * 80 / (15 * 180.0);
	rAngle4 = AngleK2 * M_PI * 80 / (15 * 180.0);
	rAngle5 = AngleH3 * M_PI * 65 / (12 * 180.0);
	rAngle6 = AngleK3 * M_PI * 65 / (12 * 180.0);
	rAngle7 = AngleH4 * M_PI * 65 / (12 * 180.0);
	rAngle8 = AngleK4 * M_PI * 65 / (12 * 180.0);



	// --- 1. CALCULATE TIME STEP (dt) ---
	float current_time = seconds_elapsed;
	float dt = current_time - last_ff_time;
	if (dt <= 0.0001f) dt = 0.001f; // Prevent division by zero
	last_ff_time = current_time;


	// --- 2. VELOCITY FEEDFORWARD (Numerical Derivative) ---
	// Calculates how fast the motor commands are changing
	v1 = (rAngle1 - prev_rA1) / dt;
	v2 = (rAngle2 - prev_rA2) / dt;
	v3 = (rAngle3 - prev_rA3) / dt;
	v4 = (rAngle4 - prev_rA4) / dt;
	v5 = (rAngle5 - prev_rA5) / dt;
	v6 = (rAngle6 - prev_rA6) / dt;
	v7 = (rAngle7 - prev_rA7) / dt;
	v8 = (rAngle8 - prev_rA8) / dt;

	// Save current angles for the next loop
	prev_rA1 = rAngle1; prev_rA2 = rAngle2; prev_rA3 = rAngle3; prev_rA4 = rAngle4;
	prev_rA5 = rAngle5; prev_rA6 = rAngle6; prev_rA7 = rAngle7; prev_rA8 = rAngle8;


	// --- 3. GRAVITY/TORQUE FEEDFORWARD (Jacobian) ---
	// Calculate physical joint torques required to support the weight (F_y)
	// Equation: Tau = J^T * F
	float F_y_per_leg = -(60.0f + added_weight) * 0.453f * 9.81f / 4.0f;

	float t_hip1  = F_y_per_leg * (l1 * sinf(sol1.theta1) + l2 * sinf(sol1.theta1 + sol1.theta2));
	float t_knee1 = F_y_per_leg * (l2 * sinf(sol1.theta1 + sol1.theta2));

	float t_hip2  = F_y_per_leg * (l1 * sinf(sol2.theta1) + l2 * sinf(sol2.theta1 + sol2.theta2));
	float t_knee2 = F_y_per_leg * (l2 * sinf(sol2.theta1 + sol2.theta2));

	float t_hip3  = F_y_per_leg * (l1 * sinf(sol3.theta1) + l2 * sinf(sol3.theta1 + sol3.theta2));
	float t_knee3 = F_y_per_leg * (l2 * sinf(sol3.theta1 + sol3.theta2));

	float t_hip4  = F_y_per_leg * (l1 * sinf(sol4.theta1) + l2 * sinf(sol4.theta1 + sol4.theta2));
	float t_knee4 = F_y_per_leg * (l2 * sinf(sol4.theta1 + sol4.theta2));


	// --- 4. CONVERT JOINT TORQUE TO MOTOR TORQUE ---
	// Motor torque = Joint torque / Gear Ratio
	float gr_front = 80.0f / 15.0f; // Front belt ratio
	float gr_back  = 65.0f / 12.0f; // Back belt ratio

	m_t1 = t_hip1 / gr_front;
	m_t2 = t_knee1 / gr_front;
	m_t3 = t_hip2 / gr_front;
	m_t4 = t_knee2 / gr_front;

	m_t5 = t_hip3 / gr_back;
	m_t6 = t_knee3 / gr_back;
	m_t7 = t_hip4 / gr_back;
	m_t8 = t_knee4 / gr_back;





	// --- 5. SEND ALL DATA TO MOTORS ---
	signalMotors(rAngle1, rAngle2, rAngle3, rAngle4, rAngle5, rAngle6, rAngle7, rAngle8,
				 v1, v2, v3, v4, v5, v6, v7, v8,
				 m_t1, m_t2, m_t3, m_t4, m_t5, m_t6, m_t7, m_t8);



    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_4);
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





/* ==============================================================================
   SPI HAL IMPLEMENTATION (Integrated from spi_hal.c)
   ============================================================================== */
static void bootn(bool state) { HAL_GPIO_WritePin(IMU0_BT_GPIO_Port, IMU0_BT_Pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET); }
static void rstn(bool state) { HAL_GPIO_WritePin(IMU0_RST_GPIO_Port, IMU0_RST_Pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET); }
static void ps0_waken(bool state) { HAL_GPIO_WritePin(IMU0_P0_GPIO_Port, IMU0_P0_Pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET); }
static void ps1(bool state) { HAL_GPIO_WritePin(IMU0_P1_GPIO_Port, IMU0_P1_Pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET); }
static void csn(bool state) { HAL_GPIO_WritePin(IMU0_CS_GPIO_Port, IMU0_CS_Pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET); }
static uint32_t timeNowUs(void) {
    uint32_t overflows = imu_timer_overflows;
    uint32_t current_tick = __HAL_TIM_GET_COUNTER(&htim3); // Read TIM3

    // Catch race conditions if the timer rolls over while we are reading it
    if (__HAL_TIM_GET_FLAG(&htim3, TIM_FLAG_UPDATE) != RESET) {
        overflows++;
    }

    // Return the true continuous microsecond count
    return (overflows * 65536) + current_tick;
}

static void enableInts(void) {
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
    HAL_NVIC_EnableIRQ(SPI1_IRQn);
}

static void disableInts() {
    HAL_NVIC_DisableIRQ(SPI1_IRQn);
    HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
}

void delayUs(uint32_t delay) {
    volatile uint32_t now = timeNowUs();
    uint32_t start = now;
    while ((now - start) < delay) { now = timeNowUs(); }
}

void resetDelayUs(uint32_t delay) {
    volatile uint32_t now = timeNowUs();
    uint32_t start = now;
    while (((now - start) < delay) && (inReset)) { now = timeNowUs(); }
}

static void spiDummyOp(void) {
    uint8_t dummyTx[1];
    uint8_t dummyRx[1];
    memset(dummyTx, 0xAA, sizeof(dummyTx));
    HAL_SPI_TransmitReceive(&hspi1, dummyTx, dummyRx, sizeof(dummyTx), 2);
}

static void spiActivate(void) {
    if ((spiState == SPI_IDLE) && (rxBufLen == 0)) {
        if (rxReady) {
            rxReady = false;
            csn(false);
            if (txBufLen > 0) {
                spiState = SPI_WRITE;
                HAL_SPI_TransmitReceive_IT(&hspi1, txBuf, rxBuf, txBufLen);
                ps0_waken(true);
            } else {
                spiState = SPI_RD_HDR;
                HAL_SPI_TransmitReceive_IT(&hspi1, (uint8_t *)txZeros, rxBuf, READ_LEN);
            }
        }
    }
}

static void spiCompleted(void) {
    uint16_t rxLen = (rxBuf[0] + (rxBuf[1] << 8)) & ~0x8000;
    if (rxLen > sizeof(rxBuf)) { rxLen = sizeof(rxBuf); }

    if (spiState == SPI_DUMMY) {
        spiState = SPI_IDLE;
    } else if (spiState == SPI_RD_HDR) {
        if (rxLen > READ_LEN) {
            spiState = SPI_RD_BODY;
            HAL_SPI_TransmitReceive_IT(&hspi1, (uint8_t *)txZeros, rxBuf+READ_LEN, rxLen-READ_LEN);
        } else {
            csn(true);
            rxBufLen = 0;
            spiState = SPI_IDLE;
            spiActivate();
        }
    } else if (spiState == SPI_RD_BODY) {
        csn(true);
        rxBufLen = rxLen;
        spiState = SPI_IDLE;
        spiActivate();
    } else if (spiState == SPI_WRITE) {
        csn(true);
        rxBufLen = (txBufLen < rxLen) ? txBufLen : rxLen;
        txBufLen = 0;
        spiState = SPI_IDLE;
        spiActivate();
    }
}

static int sh2_spi_hal_open(sh2_Hal_t *self) {
    int retval = SH2_OK;
    if (isOpen) return SH2_ERR;
    isOpen = true;

    rstn(false);
    csn(true);
    rxBufLen = 0;
    txBufLen = 0;
    rxDataReady = false;
    rxReady = false;
    inReset = true;

    spiState = SPI_DUMMY;
    spiDummyOp();
    spiState = SPI_IDLE;

    delayUs(RESET_DELAY_US);
    ps0_waken(true);
    ps1(true);

    bootn(true);
    rstn(true);

    enableInts();
    resetDelayUs(START_DELAY_US);
    return retval;
}

static void sh2_spi_hal_close(sh2_Hal_t *self) {
    disableInts();
    spiState = SPI_INIT;
    rstn(false);
    csn(true);
    HAL_SPI_DeInit(&hspi1);
    __HAL_TIM_DISABLE(&htim3);
    isOpen = false;
}

static int sh2_spi_hal_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, uint32_t *t) {
    int retval = 0;
    if (rxBufLen > 0) {
        if (len >= rxBufLen) {
            memcpy(pBuffer, rxBuf, rxBufLen);
            retval = rxBufLen;
            *t = rxTimestamp_us;
            rxBufLen = 0;
        } else {
            retval = SH2_ERR_BAD_PARAM;
            rxBufLen = 0;
        }
        disableInts();
        spiActivate();
        enableInts();
    }
    return retval;
}

static int sh2_spi_hal_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len) {
    int retval = SH2_OK;
    if ((self == 0) || (len > sizeof(txBuf)) || ((len > 0) && (pBuffer == 0))) { return SH2_ERR_BAD_PARAM; }
    if (txBufLen != 0) { return 0; }

    memcpy(txBuf, pBuffer, len);
    txBufLen = len;
    retval = len;

    disableInts();
    ps0_waken(false);
    enableInts();

    return retval;
}

static uint32_t sh2_spi_hal_getTimeUs(sh2_Hal_t *self) { return timeNowUs(); }

sh2_Hal_t *sh2_hal_init(void) {
    sh2Hal.open = sh2_spi_hal_open;
    sh2Hal.close = sh2_spi_hal_close;
    sh2Hal.read = sh2_spi_hal_read;
    sh2Hal.write = sh2_spi_hal_write;
    sh2Hal.getTimeUs = sh2_spi_hal_getTimeUs;
    return &sh2Hal;
}

/* ==============================================================================
   SYSTEM OVERRIDES & ERROR HANDLING
   ============================================================================== */
int _write(int file, char *ptr, int len) {
	HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, HAL_MAX_DELAY);
	return len;
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
