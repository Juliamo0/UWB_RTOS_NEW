/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>  // ← เพิ่มบรรทัดนี้สำหรับ va_list
#include <math.h>
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "deca_device_api.h"
#include "deca_regs.h"
#include "deca_spi.h"

extern UART_HandleTypeDef huart2;
extern SPI_HandleTypeDef hspi1;
extern TIM_HandleTypeDef htim1;

// Forward declarations
extern void reset_DW1000(void);
extern int openspi(void);
extern int dw1000_select_chip(uint8_t cs_num);
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

// ============= Debug Helper =============
void debug_print(const char* msg) {
    taskENTER_CRITICAL();
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
    taskEXIT_CRITICAL();
}

void debug_printf(const char* format, ...) {
    char buffer[128];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    taskENTER_CRITICAL();
    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), 100);
    taskEXIT_CRITICAL();
}

// ============= Shared Data (UWB) =============
float distances[4] = {0.0, 0.0, 0.0, 0.0};
volatile uint8_t uwb_data_ready = 0;

// ============= Motor Variables =============
typedef struct {
    int motor1;
    int motor2;
    int motor3;
    int motor4;
} MotorCmd_t;

int motor_1_speed = 0;
int motor_2_speed = 0;
int motor_3_speed = 0;
int motor_4_speed = 0;

int motor_1_target = 0;
int motor_2_target = 0;
int motor_3_target = 0;
int motor_4_target = 0;

// ============= UART RX Variables =============
uint8_t rxBuffer[50];
uint8_t rxIndex = 0;
uint8_t rxData;

/* USER CODE END Variables */
/* Definitions for UWB */
osThreadId_t UWBHandle;
const osThreadAttr_t UWB_attributes = {
  .name = "UWB",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh7,
};
/* Definitions for UART_TX */
osThreadId_t UART_TXHandle;
const osThreadAttr_t UART_TX_attributes = {
  .name = "UART_TX",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal6,
};
/* Definitions for Motor */
osThreadId_t MotorHandle;
const osThreadAttr_t Motor_attributes = {
  .name = "Motor",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal4,
};
/* Definitions for idle */
osThreadId_t idleHandle;
const osThreadAttr_t idle_attributes = {
  .name = "idle",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow1,
};
/* Definitions for MotorQueue */
osMessageQueueId_t MotorQueueHandle;
const osMessageQueueAttr_t MotorQueue_attributes = {
  .name = "MotorQueue"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

// Forward declaration
void Motor_Control(int motor_num, int speed);

// UART Interrupt Callback
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2) {
    if (rxData == '\n' || rxData == '\r') {
      if (rxIndex > 0) {
        rxBuffer[rxIndex] = '\0';

        // Parse คำสั่ง M:m1,m2,m3,m4
        if (strncmp((char*)rxBuffer, "M:", 2) == 0) {
          MotorCmd_t cmd = {0, 0, 0, 0};

          sscanf((char*)rxBuffer, "M:%d,%d,%d,%d",
                 &cmd.motor1, &cmd.motor2, &cmd.motor3, &cmd.motor4);

          // ส่งไป Motor Task
          osMessageQueuePut(MotorQueueHandle, &cmd, 0, 0);
        }

        rxIndex = 0;
      }
    }
    else if (rxIndex < sizeof(rxBuffer) - 1) {
      rxBuffer[rxIndex++] = rxData;
    }

    HAL_UART_Receive_IT(&huart2, &rxData, 1);
  }
}

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartTask02(void *argument);
void StartTask03(void *argument);
void StartTask04(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of MotorQueue */
  MotorQueueHandle = osMessageQueueNew (5, sizeof(uint16_t), &MotorQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of UWB */
  UWBHandle = osThreadNew(StartDefaultTask, NULL, &UWB_attributes);

  /* creation of UART_TX */
  UART_TXHandle = osThreadNew(StartTask02, NULL, &UART_TX_attributes);

  /* creation of Motor */
  MotorHandle = osThreadNew(StartTask03, NULL, &Motor_attributes);

  /* creation of idle */
  idleHandle = osThreadNew(StartTask04, NULL, &idle_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the UWB thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
    /* USER CODE BEGIN StartDefaultTask */
	debug_print("\r\n[UWB Task] Starting...\r\n");
	// เริ่ม UART Receive Interrupt
	HAL_UART_Receive_IT(&huart2, &rxData, 1);
	debug_print("[UWB Task] UART RX Started\r\n\r\n");

	// Init UWB
	openspi();
	osDelay(100);

	// เริ่ม PWM สำหรับ Motor (ทำครั้งเดียวตอน init)
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

	debug_print("[UWB Task] PWM Started\r\n");
	debug_print("[UWB Task] Ready!\r\n\r\n");

	TickType_t xLastWakeTime = xTaskGetTickCount();
	const TickType_t xFrequency = pdMS_TO_TICKS(100); // 100ms = 10Hz

	/* Infinite loop */
	for(;;)
	{
	  // LED Heartbeat
	  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);

	  // TODO: อ่าน UWB 4 CS (ทำในขั้นต่อไป)
	  // ตอนนี้ส่งค่า dummy ก่อน
	  distances[0] = 1.23;
	  distances[1] = 2.34;
	  distances[2] = 3.45;
	  distances[3] = 4.56;

	  // บอก UART Task ว่าข้อมูลพร้อม
	  uwb_data_ready = 1;
	  xTaskNotifyGive(UART_TXHandle);

	  // Debug
	  //debug_printf("[UWB] Tick: %lu\r\n", xTaskGetTickCount());

	  // รอจนครบ 100ms พอดี (guarantee timing)
	  vTaskDelayUntil(&xLastWakeTime, xFrequency);
	}
	/* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the UART_TX thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument)
{
	/* USER CODE BEGIN StartTask02 */
	char tx_buffer[100];

	debug_print("[UART TX Task] Starting...\r\n");

	/* Infinite loop */
	for(;;)
	{
	  // รอให้ UWB Task บอกว่าข้อมูลพร้อม
	  ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

	  if(uwb_data_ready) {
	      // ส่งข้อมูล UWB ทั้ง 4 CS
	      taskENTER_CRITICAL();  // ป้องกัน race condition
	      sprintf(tx_buffer, "$%.2f,%.2f,%.2f,%.2f*\r\n",
	              distances[0], distances[1],
	              distances[2], distances[3]);
	      uwb_data_ready = 0;
	      taskEXIT_CRITICAL();

	      HAL_UART_Transmit(&huart2, (uint8_t*)tx_buffer, strlen(tx_buffer), 100);
	  }
	}
	/* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
* @brief Function implementing the Motor thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask03 */
void StartTask03(void *argument)
{
	/* USER CODE BEGIN StartTask03 */
	MotorCmd_t cmd;

	debug_print("[Motor Task] Starting...\r\n");

	/* Infinite loop */
	for(;;)
	{
	  // รอคำสั่งจาก Queue
	  if(osMessageQueueGet(MotorQueueHandle, &cmd, NULL, 100) == osOK) {

	    motor_1_target = cmd.motor1;
	    motor_2_target = cmd.motor2;
	    motor_3_target = cmd.motor3;
	    motor_4_target = cmd.motor4;

	    debug_printf("[Motor] Received: M1=%d M2=%d M3=%d M4=%d\r\n",
	                 cmd.motor1, cmd.motor2, cmd.motor3, cmd.motor4);

	    // TODO: Motor_Soft_Update() (ทำในขั้นต่อไป)
	    // ตอนนี้ควบคุมโดยตรงก่อน
	    Motor_Control(1, motor_1_target);
	    Motor_Control(2, motor_2_target);
	    Motor_Control(3, motor_3_target);
	    Motor_Control(4, motor_4_target);
	  }

	  osDelay(20);
	}
	/* USER CODE END StartTask03 */
}

/* USER CODE BEGIN Header_StartTask04 */
/**
* @brief Function implementing the idle thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask04 */
void StartTask04(void *argument)
{
  /* USER CODE BEGIN StartTask04 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartTask04 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

void Motor_Control(int motor_num, int speed)
{
  GPIO_TypeDef* dir_port;
  uint16_t dir_pin;
  uint32_t pwm_channel;

  switch(motor_num) {
    case 1:
      dir_port = GPIOB;
      dir_pin = GPIO_PIN_10;
      pwm_channel = TIM_CHANNEL_1;
      break;
    case 2:
      dir_port = GPIOB;
      dir_pin = GPIO_PIN_4;
      pwm_channel = TIM_CHANNEL_2;
      break;
    case 3:
      dir_port = GPIOB;
      dir_pin = GPIO_PIN_5;
      pwm_channel = TIM_CHANNEL_3;
      break;
    case 4:
      dir_port = GPIOB;
      dir_pin = GPIO_PIN_3;
      pwm_channel = TIM_CHANNEL_4;
      break;
    default:
      return;
  }

  if (speed == 0) {
    __HAL_TIM_SET_COMPARE(&htim1, pwm_channel, 0);
  }
  else if (speed > 0) {
    HAL_GPIO_WritePin(dir_port, dir_pin, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&htim1, pwm_channel, (speed * 420) / 100);
  }
  else {
    HAL_GPIO_WritePin(dir_port, dir_pin, GPIO_PIN_SET);
    __HAL_TIM_SET_COMPARE(&htim1, pwm_channel, ((-speed) * 420) / 100);
  }
}

/* USER CODE END Application */

