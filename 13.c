/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "string.h"
#include "tim.h"
#include "gpio.h"
#include "i2c.h"
#include "usart.h"
#include "dma.h"
#include "stdio.h"

#include "MS5837.h"
#include "ROV.h"
//
#include "ds18b20.h"
//
#include "INA219.h"
//
#include "24cxx.h"
//
#include "GPSNEMA.h"
//
#include "file_handling.h"
//
#include "mpu9255.h"
//
#include "iwdg.h"
//
#include "adc.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
struct I2C_Module
{
  I2C_HandleTypeDef   instance;
  uint16_t            sdaPin;
  GPIO_TypeDef*       sdaPort;
  uint16_t            sclPin;
  GPIO_TypeDef*       sclPort;
};
struct I2C_Module i2cc = {I2C2, GPIO_PIN_0, GPIOF, GPIO_PIN_1, GPIOF};
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
//void I2C_ClearBusyFlagErratum(struct I2C_Module* i2c);
static uint8_t wait_for_gpio_state_timeout(GPIO_TypeDef* port, uint16_t pin, GPIO_PinState state, uint8_t timeout);
void generateClocks(uint8_t numClocks, uint8_t sendStopBits);
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
DataInput Sat_Nav;
DataInput Raspberry;
NEMA_t GPS;
ROV_Data Decoded_Data_Raspberry, Decoded_Data_Raspberry_filtered;
ROV_Data Previous_data_controller;
ROV_Data Original_State;
ROV_Data Decoded_Data_ROV;
Controller_output Controller_output_data;
depth_Controller_output depth_cont_output;
_Bool FirstTimeFlag = 1, Water_leak = 0, Water_leak_Read[4] = {0,0,0,0};
HAL_StatusTypeDef Uart1_DMA_Init_Status, Uart2_DMA_Init_Status, Uart6_DMA_Init_Status;
// INA219 Variables
INA219_t ina219;
float vbus;
// EEPROM Variables
float read_num;
// Lat Lon to be sent
int32_t Lat_int, Lon_int;
// Depth Sensor
MS5837 Depth_Sensor;
uint16_t SS;
HAL_StatusTypeDef MS5837_Init_Flag = HAL_ERROR;
//
size_t heap_size_remained;
//
MPU9255_t MPU9255;
uint8_t mpu_flag = 1;
//
AVG_Param ublox_avg;
//
HAL_StatusTypeDef ret;
//
_Bool Sat_nav_first_time_flag = 1;
//
uint16_t Impact_det_Val[1];
uint8_t Impact_det = 0;

Saturation Satur_Rasp;
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 1000 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Controller */
osThreadId_t ControllerHandle;
const osThreadAttr_t Controller_attributes = {
  .name = "Controller",
  .stack_size = 2000 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for AHRS */
osThreadId_t AHRSHandle;
const osThreadAttr_t AHRS_attributes = {
  .name = "AHRS",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void Controll_Task(void *argument);
void AHRS_FCN(void *argument);

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

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of Controller */
  ControllerHandle = osThreadNew(Controll_Task, NULL, &Controller_attributes);

  /* creation of AHRS */
  AHRSHandle = osThreadNew(AHRS_FCN, NULL, &AHRS_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
// uart 1 = GPS
// uart 2 = Raspberry
// uart 6 = Sat-nav Board

	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
	TIM4->CCR1 = 1500;
	TIM4->CCR2 = 1500;
	TIM4->CCR3 = 1500;
	TIM4->CCR4 = 1500;

	ublox_avg.buff_CNT = 0;

	Uart2_DMA_Init_Status = HAL_UARTEx_ReceiveToIdle_DMA(&huart2, Raspberry.Control_Unit, sizeof(Raspberry.Control_Unit));
//	Uart1_DMA_Init_Status = HAL_UART_Receive_DMA(&huart1, GPS.Raw, sizeof(GPS.Raw));
	Uart6_DMA_Init_Status = HAL_UARTEx_ReceiveToIdle_DMA(&huart6, Sat_Nav.Control_Unit, sizeof(Sat_Nav.Control_Unit));

	Ds18b20_Init(osPriorityLow7); // Reading Temperature Sensor
	HAL_ADC_Start_DMA(&hadc3, Impact_det_Val, 1);
	Impact_det = 0;
//	HAL_TIM_Base_Start_IT(&htim14); // GPS timeout timer
//	HAL_TIM_Base_Start_IT(&htim13); // GPS timeout timer
//	mount_sd();

//	AT24CXX_Init();
//	osDelay(500);
//	AT24CXX_Write_Float(10, 102.2);
//	osDelay(500);
//	read_num = AT24CXX_Read_Float(10);

  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
//    Water Leak detection Begin
//    HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
    Water_leak_Read[0] = HAL_GPIO_ReadPin(Water_Leak_1_GPIO_Port, Water_Leak_1_Pin);
    Water_leak_Read[1] = HAL_GPIO_ReadPin(Water_Leak_2_GPIO_Port, Water_Leak_2_Pin);
    Water_leak_Read[2] = HAL_GPIO_ReadPin(Water_Leak_3_GPIO_Port, Water_Leak_3_Pin);
    Water_leak_Read[3] = HAL_GPIO_ReadPin(Water_Leak_4_GPIO_Port, Water_Leak_4_Pin);
    if(Water_leak_Read[0] | !Water_leak_Read[1] | Water_leak_Read[2] | Water_leak_Read[3])
    {
    	Water_leak = 1;
    }
    else
    {
    	Water_leak = 0;
    }
//    Water Leak detection END
    // Impact detection Begin
//    Impact_det_Val[0] = HAL_ADC_GetValue(&hadc3);
//    Impact_det_Val[0] = 0;
    if(Impact_det_Val[0]>Decoded_Data_Raspberry.Data_Len)
    {
    	Impact_det = 1;
    }
    // Impact detection End
    if(Decoded_Data_Raspberry.Depth)
	{
		Impact_det = 0;
	}

    /////////// Saturation of Motors //////////////
    Satur_Rasp.BottomUp = Decoded_Data_Raspberry.Counter - keepMotion;
    Satur_Rasp.LeftRightLow = Decoded_Data_Raspberry.Pitch;
    Satur_Rasp.LeftRightUp = Decoded_Data_Raspberry.Yaw;
//    vbus = INA219_ReadBusVoltage(&ina219);
//    Transmitting data to Raspberry
    // write the data to raspberry
    if(Raspberry.RX_Flag_1)
    {
    	HAL_UART_Transmit(&huart2, (uint8_t*)&Raspberry.Control_Unit[0], 2, 10);
    	HAL_UART_Transmit(&huart2, (uint8_t*)&Decoded_Data_Raspberry.Mode, 1, 10);
    	HAL_UART_Transmit(&huart2, (uint8_t*)&Decoded_Data_ROV.Counter, 4, 10);
    	HAL_UART_Transmit(&huart2, (uint8_t*)&Decoded_Data_ROV.Data_Len, 2, 10);
    	HAL_UART_Transmit(&huart2, (uint8_t*)&Decoded_Data_ROV.Lattitude, 8, 10);
    	HAL_UART_Transmit(&huart2, (uint8_t*)&Decoded_Data_ROV.Longitude, 8, 10);
    	HAL_UART_Transmit(&huart2, (uint8_t*)&Depth_Sensor.depth, 4, 10);
    	HAL_UART_Transmit(&huart2, (uint8_t*)&Decoded_Data_ROV.Velocity_N_Dir, 4, 10);
    	HAL_UART_Transmit(&huart2, (uint8_t*)&Decoded_Data_ROV.Velocity_E_Dir, 4, 10);
    	HAL_UART_Transmit(&huart2, (uint8_t*)&Decoded_Data_ROV.Velocity_D_Dir, 4, 10);
    	HAL_UART_Transmit(&huart2, (uint8_t*)&Decoded_Data_ROV.Roll, 4, 10);
    	HAL_UART_Transmit(&huart2, (uint8_t*)&Decoded_Data_ROV.Pitch, 4, 10);
    	HAL_UART_Transmit(&huart2, (uint8_t*)&Decoded_Data_ROV.Yaw, 4, 10);
    	HAL_UART_Transmit(&huart2, (uint8_t*)&Decoded_Data_ROV.W_X_Ax, 4, 10);
    	HAL_UART_Transmit(&huart2, (uint8_t*)&Decoded_Data_ROV.W_Y_Ax, 4, 10);
    	HAL_UART_Transmit(&huart2, (uint8_t*)&Decoded_Data_ROV.W_Z_Ax, 4, 10);
    	HAL_UART_Transmit(&huart2, (uint8_t*)&Decoded_Data_Raspberry.Lattitude_Tgt, 4, 10);
    	HAL_UART_Transmit(&huart2, (uint8_t*)&Decoded_Data_Raspberry.Longitude_Tgt, 4, 10);
    	HAL_UART_Transmit(&huart2, (uint8_t*)&Decoded_Data_Raspberry.Depth_Tgt, 4, 10);
    	HAL_UART_Transmit(&huart2, (uint8_t*)&Decoded_Data_ROV.CK_A, 1, 10);
    	HAL_UART_Transmit(&huart2, (uint8_t*)&Decoded_Data_ROV.CK_B, 1, 10);
    	HAL_UART_Transmit(&huart2, (uint8_t*)&Raspberry.Control_Unit[72], 1, 10);

    	HAL_UART_Transmit(&huart2, (uint8_t*)&Controller_output_data.PID_M[1], 4, 10);
    	HAL_UART_Transmit(&huart2, (uint8_t*)&Controller_output_data.PID_M[2], 4, 10);
    	HAL_UART_Transmit(&huart2, (uint8_t*)&Controller_output_data.f_u, 4, 10);
    	HAL_UART_Transmit(&huart2, (uint8_t*)&Controller_output_data.tau_p, 4, 10);
    	HAL_UART_Transmit(&huart2, (uint8_t*)&Controller_output_data.k_pf, 4, 10);
    	HAL_UART_Transmit(&huart2, (uint8_t*)&Controller_output_data.k_ptau, 4, 10);
    	HAL_UART_Transmit(&huart2, (uint8_t*)&Controller_output_data.previous_distance, 4, 10);
    	HAL_UART_Transmit(&huart2, (uint8_t*)&Controller_output_data.target_distance, 4, 10);
    	HAL_UART_Transmit(&huart2, (uint8_t*)&vbus, 4, 10);
    	HAL_UART_Transmit(&huart2, (uint8_t*)&Controller_output_data.Motor_Current[1], 4, 10);
    	HAL_UART_Transmit(&huart2, (uint8_t*)&Controller_output_data.Motor_Current[2], 4, 10);
    	HAL_UART_Transmit(&huart2, (uint8_t*)&Controller_output_data.Motor_Current[3], 4, 10);
    	HAL_UART_Transmit(&huart2, (uint8_t*)&TIM4->CCR1, 4, 10);
    	HAL_UART_Transmit(&huart2, (uint8_t*)&TIM4->CCR2, 4, 10);
    	HAL_UART_Transmit(&huart2, (uint8_t*)&TIM4->CCR3, 4, 10);
    	HAL_UART_Transmit(&huart2, (uint8_t*)&TIM4->CCR4, 4, 10);
    	HAL_UART_Transmit(&huart2, (uint8_t*)&Controller_output_data.path_angle_Error, 4, 10);
    	HAL_UART_Transmit(&huart2, (uint8_t*)&Original_State.Lattitude, 8, 10);
    	HAL_UART_Transmit(&huart2, (uint8_t*)&Original_State.Longitude, 8, 10);
    	HAL_UART_Transmit(&huart2, (uint8_t*)&Controller_output_data.step_angle, 4, 10);
    	HAL_UART_Transmit(&huart2, (uint8_t*)&Controller_output_data.X, 4, 10);
    	HAL_UART_Transmit(&huart2, (uint8_t*)&Controller_output_data.Y, 4, 10);

    	HAL_UART_Transmit(&huart2, (uint8_t*)&ds18b20[0].Temperature, 4, 10);
    	HAL_UART_Transmit(&huart2, (uint8_t*)&ds18b20[1].Temperature, 4, 10);
    	HAL_UART_Transmit(&huart2, (uint8_t*)&ds18b20[2].Temperature, 4, 10);
    	HAL_UART_Transmit(&huart2, (uint8_t*)&ds18b20[3].Temperature, 4, 10);

    	HAL_UART_Transmit(&huart2, (uint8_t*)&Impact_det, 1, 10);
    	// gps fix status
    	// 4 Temp Sensor
    	// water leak
    	// data byte 192
    	Raspberry.RX_Flag_1 = 0;
    }
// 	  Automatic depth control Mode
//	if(Decoded_Data_Raspberry.Mode == Auto) // Auto = 1, depth control
//	{
//		depth_cont_output = depthControl(Decoded_Data_Raspberry);
//		FirstTimeFlag = 1;
//	}
	// Manual Control Mode
	if(Decoded_Data_Raspberry.Mode == Manual) // Manual = 4
	{
		TIM4->CCR1 = Decoded_Data_Raspberry.Roll;
		TIM4->CCR2 = Decoded_Data_Raspberry.Velocity_N_Dir;
		TIM4->CCR3 = Decoded_Data_Raspberry.Velocity_E_Dir;
		TIM4->CCR4 = Decoded_Data_Raspberry.Velocity_D_Dir;
		FirstTimeFlag = 1;
	}
	// P2P Control with GPS
	if(Decoded_Data_Raspberry.Mode == 3) // GPS Controller mode = 3
	{
//		Decoded_Data_ROV.Lattitude = GPS.Lat;
//		Decoded_Data_ROV.Longitude = GPS.Lon;
		if(FirstTimeFlag)
		{
			Original_State = Decoded_Data_Raspberry_filtered;
		}
//		if(Controller_output_data.target_distance> 5 || FirstTimeFlag)
		if(Decoded_Data_Raspberry.Mode == 3 || FirstTimeFlag) // it must be declared in Raspberry pi
		{
			Controller_output_data = GPS_Controller(Decoded_Data_ROV, Decoded_Data_Raspberry, Previous_data_controller, Original_State, Satur_Rasp);
			Previous_data_controller = Decoded_Data_Raspberry_filtered;
		}
		else
		{
			TIM4->CCR1 = 1500;
			TIM4->CCR2 = 1500;
			TIM4->CCR3 = 1500;
			TIM4->CCR4 = 1500;
		}
			FirstTimeFlag = 0;
	}
	if(Decoded_Data_Raspberry.Mode == 9) // emergency sotp mode
	{
		TIM4->CCR1 = 1500;
		TIM4->CCR2 = 1500;
		TIM4->CCR3 = 1500;
		TIM4->CCR4 = 1500;
	 }
  }

  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_Controll_Task */
/**
* @brief Function implementing the Controller thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Controll_Task */
void Controll_Task(void *argument)
{
  /* USER CODE BEGIN Controll_Task */

	INA219_Init(&ina219, &hi2c3, INA219_ADDRESS);

	Depth_Sensor.ofst = 90000; // 90.000 pa atmospheric pressure
	HAL_TIM_Base_Start_IT(&htim14); // GPS timeout timer
	HAL_TIM_Base_Start_IT(&htim13); // Sat-nav timeout timer
	HAL_TIM_Base_Start_IT(&htim10); // RaSpberry timeout timer



  /* Infinite loop */
  for(;;)
  {
//	heap_size_remained = xPortGetFreeHeapSize();
//	NEMA_Sep(&GPS, &ublox_avg, &Decoded_Data_Raspberry_filtered);

    vbus = (float)INA219_ReadBusVoltage(&ina219)*0.001;

//    Controller_output_data.bus_voltage = (float)vbus*0.001;
    osDelay(100);
    if(Sat_Nav.Timeout)
    {
    	HAL_UART_AbortReceive(&huart6);
    	memset(Sat_Nav.Control_Unit, 0, 80);
    	Uart6_DMA_Init_Status = HAL_UARTEx_ReceiveToIdle_DMA(&huart6, Sat_Nav.Control_Unit, sizeof(Sat_Nav.Control_Unit));
//    	Sat_Nav.Timeout = 0;
    	TIM13->CNT = 0;
    	Sat_nav_first_time_flag = 1;
    }


    if(Raspberry.Timeout)
	{
		HAL_UART_AbortReceive(&huart2);
		memset(Raspberry.Control_Unit, 0, 80);
		Uart6_DMA_Init_Status = HAL_UARTEx_ReceiveToIdle_DMA(&huart2, Raspberry.Control_Unit, sizeof(Raspberry.Control_Unit));
//		Raspberry.Timeout = 0;
		TIM10->CNT = 0;
		TIM4->CCR1 = 1500;
		TIM4->CCR2 = 1500;
		TIM4->CCR3 = 1500;
		TIM4->CCR4 = 1500;
		Decoded_Data_Raspberry.Mode = 9;
	}

//    if(GPS.Timeout)
//	{
//    	HAL_UART_MspDeInit(&huart1);
//    	memset(GPS, 0, 500);
//    	USART1->SR = 0xd0;
//    	USART1->DR = 0x0;
//    	USART1->BRR = 0x2220;
//    	USART1->CR1 = 0x200C;
//    	USART1->CR2 = 0;
//    	USART1->CR3 = 0x41;
//    	USART1->GTPR = 0;
//    	MX_USART1_UART_Init();
//    	HAL_UART_MspInit(&huart1);
//		Uart1_DMA_Init_Status = HAL_UART_Receive_DMA(&huart1, GPS.Raw, sizeof(GPS.Raw));
//		TIM14->CNT = 0;
//		GPS.Timeout = 0;
//	}
//    HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
  }
  /* USER CODE END Controll_Task */
}

/* USER CODE BEGIN Header_AHRS_FCN */
/**
* @brief Function implementing the AHRS thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_AHRS_FCN */
void AHRS_FCN(void *argument)
{
  /* USER CODE BEGIN AHRS_FCN */
//	for(int i=1; i<128; i++)
//	{
//		osDelay(10);
//		ret = HAL_I2C_IsDeviceReady(&hi2c2, (uint16_t)(i<<1), 3, 5);
//		if (ret != HAL_OK) /* No ACK Received At That Address */
//		{
////            MS5837_Init_Flag = 0;
//		}
//		else if(ret == HAL_OK)
//		{
////		  sprintf(Buffer, "0x%X", i);
////		  MS5837_Init_Flag = 1;
//		}
//	}
//	ret = HAL_I2C_IsDeviceReady(&hi2c2, MPU9250_ADDRESS, 3, 5);

	while((I2C2->SR2 & 0b00000010) == 2 || (I2C2->CR1 & 0b0000000000000001) == 0)
	{
//		I2C2->CR1 = 0b1000000000000000;
//		osDelay(100);
//		I2C2->CR1 = 0b0000000000000000;
//		HAL_I2C_MspDeInit(&hi2c2);
//		osDelay(100);
//		MX_I2C2_Init();
		generateClocks(1, 1);
	}
//	while (mpu_flag == 1)
//	{
//		mpu_flag = MPU9255_Init(&hi2c2);
//		if((I2C2->SR2 & 0b00000010) == 2 || (I2C2->CR1 & 0b0000000000000001) == 0)
//		{
//			generateClocks(1, 1);
//		}
////		osDelay(300);
//	}
//	osDelay(10000);
//	while(MPU9255_Init(&hi2c2) == 1);
	osDelay(5000);
	mpu_flag = 1;
	MS5837_Init_Flag = MS5837_Init(&Depth_Sensor, &hi2c2);
	HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
  /* Infinite loop */
  for(;;)
  {
//	  readAll(&hi2c2, &MPU9255);
	  // pressure sensor
	  if(MS5837_Init_Flag == HAL_OK)
	  {
		MS5837_Init_Flag = MS5837_read(&Depth_Sensor, &hi2c2);
	  }
	  if(MS5837_Init_Flag != HAL_OK)
	  {
		MS5837_Init_Flag = MS5837_Init(&Depth_Sensor, &hi2c2);
	  }
	  if((I2C2->SR2 & 0b00000010) == 2 || (I2C2->CR1 & 0b0000000000000001) == 0)
	  {
		generateClocks(1, 1);
	  }
	  osDelay(1);
  }
  /* USER CODE END AHRS_FCN */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//	if(huart == &huart1)
//	{
//		HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
//		if(!GPS.decoded)
//		{
//			memcpy(GPS.toDecode, GPS.Raw,Raw_Data_Size);
//			GPS.decoded = 1;
//		}
//		TIM14->CNT = 0;
//		GPS.Timeout = 0;
//		memset(GPS.Raw, 0, Raw_Data_Size);
//	}
//}

//void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
//{
//	if(hadc == &hadc3)
//	{
//		if(Impact_det_Val[0]>200)
//		{
//			Impact_det = 1;
//		}
//	}
//}


void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	/// Raspberry system ///
	if(huart == &huart2)
	{
		if(Size == 72)
		{
			if(Raspberry.Control_Unit[0] == 0x8E)
			{
				Decoded_Data_Raspberry = Decode(Raspberry.Control_Unit);
//				Lat_int = Raspberry.Control_Unit[12] << 24 | Raspberry.Control_Unit[11] << 16 |Raspberry.Control_Unit[10] << 8 | Raspberry.Control_Unit[9];
//				Lon_int = Raspberry.Control_Unit[16] << 24 | Raspberry.Control_Unit[15] << 16 |Raspberry.Control_Unit[14] << 8 | Raspberry.Control_Unit[13];
				Raspberry.RX_Flag_1 = 1;
				Raspberry.Timeout = 0;
				TIM10->CNT = 0;
				HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
				memset(Raspberry.Control_Unit, 0, 72);
			}
			else
			{
				memset(Raspberry.Control_Unit, 0, 72);
			}
		}
	}
	///// Sat-Nav //////
	if(huart == &huart6)
	{
		if(Size == 72)
		{
			if(Sat_nav_first_time_flag)
			{
				memset(Sat_Nav.Control_Unit, 0, 72);
				Sat_nav_first_time_flag = 0;
			}
			if(!Sat_nav_first_time_flag)
			{
				if(Sat_Nav.Control_Unit[0] == 142)//0x8E
				{
					Decoded_Data_ROV = Decode(Sat_Nav.Control_Unit);
					Sat_Nav.RX_Flag_1 = 1;
					HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
					TIM13->CNT = 0;
					Sat_Nav.Timeout = 0;
					HAL_IWDG_Refresh(&hiwdg);
					memset(Sat_Nav.Control_Unit, 0, 72);
					if(ublox_avg.buff_CNT == Buff_Size)
					{
						Decoded_Data_Raspberry_filtered.Lattitude = (double)Mean(ublox_avg.Lat_buff);
						Decoded_Data_Raspberry_filtered.Longitude = (double)Mean(ublox_avg.Lon_buff);
						ublox_avg.Lat_buff[0] = *shiftForward(ublox_avg.Lat_buff, Decoded_Data_ROV.Lattitude);
						ublox_avg.Lon_buff[0] = *shiftForward(ublox_avg.Lon_buff, Decoded_Data_ROV.Longitude);
					}
					else
					{
						ublox_avg.Lat_buff[ublox_avg.buff_CNT] = Decoded_Data_ROV.Lattitude;
						ublox_avg.Lon_buff[ublox_avg.buff_CNT] = Decoded_Data_ROV.Longitude;
						ublox_avg.buff_CNT++;
					}
				}
				else
				{
					memset(Sat_Nav.Control_Unit, 0, 72);
				}
			}
		}

	}
}

void I2C_ClearBusyFlagErratum(struct I2C_Module* i2c)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  // 1. Clear PE bit.
  i2c->instance.Instance->CR1 &= ~(0x0001);

  //  2. Configure the SCL and SDA I/Os as General Purpose Output Open-Drain, High level (Write 1 to GPIOx_ODR).
  GPIO_InitStructure.Mode         = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStructure.Alternate    = GPIO_AF4_I2C2;
  GPIO_InitStructure.Pull         = GPIO_PULLUP;
  GPIO_InitStructure.Speed        = GPIO_SPEED_FREQ_HIGH;

  GPIO_InitStructure.Pin          = i2c->sclPin;
  HAL_GPIO_Init(i2c->sclPort, &GPIO_InitStructure);
  HAL_GPIO_WritePin(i2c->sclPort, i2c->sclPin, GPIO_PIN_SET);

  GPIO_InitStructure.Pin          = i2c->sdaPin;
  HAL_GPIO_Init(i2c->sdaPort, &GPIO_InitStructure);
  HAL_GPIO_WritePin(i2c->sdaPort, i2c->sdaPin, GPIO_PIN_SET);

  // 3. Check SCL and SDA High level in GPIOx_IDR.
//  while (GPIO_PIN_SET != HAL_GPIO_ReadPin(i2c->sclPort, i2c->sclPin))
//  {
//    asm("nop");
//  }
//
//  while (GPIO_PIN_SET != HAL_GPIO_ReadPin(i2c->sdaPort, i2c->sdaPin))
//  {
//    asm("nop");
//  }

  // 4. Configure the SDA I/O as General Purpose Output Open-Drain, Low level (Write 0 to GPIOx_ODR).
  HAL_GPIO_WritePin(i2c->sdaPort, i2c->sdaPin, GPIO_PIN_RESET);

  //  5. Check SDA Low level in GPIOx_IDR.
//  while (GPIO_PIN_RESET != HAL_GPIO_ReadPin(i2c->sdaPort, i2c->sdaPin))
//  {
//    asm("nop");
//  }

  // 6. Configure the SCL I/O as General Purpose Output Open-Drain, Low level (Write 0 to GPIOx_ODR).
  HAL_GPIO_WritePin(i2c->sclPort, i2c->sclPin, GPIO_PIN_RESET);

  //  7. Check SCL Low level in GPIOx_IDR.
//  while (GPIO_PIN_RESET != HAL_GPIO_ReadPin(i2c->sclPort, i2c->sclPin))
//  {
//    asm("nop");
//  }

  // 8. Configure the SCL I/O as General Purpose Output Open-Drain, High level (Write 1 to GPIOx_ODR).
  HAL_GPIO_WritePin(i2c->sclPort, i2c->sclPin, GPIO_PIN_SET);

  // 9. Check SCL High level in GPIOx_IDR.
//  while (GPIO_PIN_SET != HAL_GPIO_ReadPin(i2c->sclPort, i2c->sclPin))
//  {
//    asm("nop");
//  }

  // 10. Configure the SDA I/O as General Purpose Output Open-Drain , High level (Write 1 to GPIOx_ODR).
  HAL_GPIO_WritePin(i2c->sdaPort, i2c->sdaPin, GPIO_PIN_SET);

  // 11. Check SDA High level in GPIOx_IDR.
//  while (GPIO_PIN_SET != HAL_GPIO_ReadPin(i2c->sdaPort, i2c->sdaPin))
//  {
//    asm("nop");
//  }

  // 12. Configure the SCL and SDA I/Os as Alternate function Open-Drain.
  GPIO_InitStructure.Mode         = GPIO_MODE_AF_OD;
  GPIO_InitStructure.Alternate    = GPIO_AF4_I2C2;

  GPIO_InitStructure.Pin          = i2c->sclPin;
  HAL_GPIO_Init(i2c->sclPort, &GPIO_InitStructure);

  GPIO_InitStructure.Pin          = i2c->sdaPin;
  HAL_GPIO_Init(i2c->sdaPort, &GPIO_InitStructure);

  // 13. Set SWRST bit in I2Cx_CR1 register.
  i2c->instance.Instance->CR1 |= 0x8000;

  asm("nop");

  // 14. Clear SWRST bit in I2Cx_CR1 register.
  i2c->instance.Instance->CR1 &= ~0x8000;

  asm("nop");

  // 15. Enable the I2C peripheral by setting the PE bit in I2Cx_CR1 register
  i2c->instance.Instance->CR1 |= 0x0001;

  // Call initialization function.
  HAL_I2C_Init(&(i2c->instance));
}


void generateClocks(uint8_t numClocks, uint8_t sendStopBits){
/* This function big-bangs the I2C master clock
*
* https://electronics.stackexchange.com/questions/267972/i2c-busy-flag-strange-behaviour/281046#281046
* https://community.st.com/thread/35884-cant-reset-i2c-in-stm32f407-to-release-i2c-lines
* https://electronics.stackexchange.com/questions/272427/stm32-busy-flag-is-set-after-i2c-initialization
* http://www.st.com/content/ccc/resource/technical/document/errata_sheet/f5/50/c9/46/56/db/4a/f6/CD00197763.pdf/files/CD00197763.pdf/jcr:content/translations/en.CD00197763.pdf
*
*
* Arguments: numClocks, the number of times to cycle the I2C master clock
* sendStopBits, 1 if stop bits are to be sent on SDA
*
* Returns: none
*/

static struct I2C_Module{
	I2C_HandleTypeDef*   instance;
	uint16_t            sdaPin;
	GPIO_TypeDef*       sdaPort;
	uint16_t            sclPin;
	GPIO_TypeDef*       sclPort;
}i2cmodule = {&hi2c2, GPIO_PIN_0, GPIOF, GPIO_PIN_1, GPIOF};
static struct I2C_Module* i2c = &i2cmodule;
static uint8_t timeout = 1;

GPIO_InitTypeDef GPIO_InitStructure;

I2C_HandleTypeDef* handler = NULL;

handler = i2c->instance;

// 1. Clear PE bit.
CLEAR_BIT(handler->Instance->CR1, I2C_CR1_PE);

GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_OD;
GPIO_InitStructure.Pull = GPIO_NOPULL;
GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

GPIO_InitStructure.Pin = i2c->sclPin;
HAL_GPIO_Init(i2c->sclPort, &GPIO_InitStructure);

GPIO_InitStructure.Pin = i2c->sdaPin;
HAL_GPIO_Init(i2c->sdaPort, &GPIO_InitStructure);

for(uint8_t i = 0; i < numClocks; i++){
	// 3. Check SCL and SDA High level in GPIOx_IDR.
	if(sendStopBits){HAL_GPIO_WritePin(i2c->sdaPort, i2c->sdaPin, GPIO_PIN_SET);}
	HAL_GPIO_WritePin(i2c->sclPort, i2c->sclPin, GPIO_PIN_SET);

	wait_for_gpio_state_timeout(i2c->sclPort, i2c->sclPin, GPIO_PIN_SET, timeout);
	if(sendStopBits){wait_for_gpio_state_timeout(i2c->sdaPort, i2c->sdaPin, GPIO_PIN_SET, timeout);}

	// 4. Configure the SDA I/O as General Purpose Output Open-Drain, Low level (Write 0 to GPIOx_ODR).
	if(sendStopBits){
		HAL_GPIO_WritePin(i2c->sdaPort, i2c->sdaPin, GPIO_PIN_RESET);
		wait_for_gpio_state_timeout(i2c->sdaPort, i2c->sdaPin, GPIO_PIN_RESET, timeout); // 5. Check SDA Low level in GPIOx_IDR
	}

	// 6. Configure the SCL I/O as General Purpose Output Open-Drain, Low level (Write 0 to GPIOx_ODR).
	HAL_GPIO_WritePin(i2c->sclPort, i2c->sclPin, GPIO_PIN_RESET);
	wait_for_gpio_state_timeout(i2c->sclPort, i2c->sclPin, GPIO_PIN_RESET, timeout); // 7. Check SCL Low level in GPIOx_IDR.

	// 8. Configure the SCL I/O as General Purpose Output Open-Drain, High level (Write 1 to GPIOx_ODR).
	HAL_GPIO_WritePin(i2c->sclPort, i2c->sclPin, GPIO_PIN_SET);
	wait_for_gpio_state_timeout(i2c->sclPort, i2c->sclPin, GPIO_PIN_SET, timeout); // 9. Check SCL High level in GPIOx_IDR.

	// 10. Configure the SDA I/O as General Purpose Output Open-Drain , High level (Write 1 to GPIOx_ODR).
	if(sendStopBits){
		HAL_GPIO_WritePin(i2c->sdaPort, i2c->sdaPin, GPIO_PIN_SET);
		wait_for_gpio_state_timeout(i2c->sdaPort, i2c->sdaPin, GPIO_PIN_SET, timeout); // 11. Check SDA High level in GPIOx_IDR.
	}
}

// 12. Configure the SCL and SDA I/Os as Alternate function Open-Drain.
GPIO_InitStructure.Mode = GPIO_MODE_AF_OD;
GPIO_InitStructure.Alternate = GPIO_AF4_I2C3;

GPIO_InitStructure.Pin = i2c->sclPin;
HAL_GPIO_Init(i2c->sclPort, &GPIO_InitStructure);

GPIO_InitStructure.Pin = i2c->sdaPin;
HAL_GPIO_Init(i2c->sdaPort, &GPIO_InitStructure);

// 13. Set SWRST bit in I2Cx_CR1 register.
SET_BIT(handler->Instance->CR1, I2C_CR1_SWRST);
asm("nop");

/* 14. Clear SWRST bit in I2Cx_CR1 register. */
CLEAR_BIT(handler->Instance->CR1, I2C_CR1_SWRST);
asm("nop");

/* 15. Enable the I2C peripheral by setting the PE bit in I2Cx_CR1 register */
SET_BIT(handler->Instance->CR1, I2C_CR1_PE);
asm("nop");

HAL_I2C_Init(handler);
}

static uint8_t wait_for_gpio_state_timeout(GPIO_TypeDef* port, uint16_t pin, GPIO_PinState state, uint8_t timeout){
/* Helper function for generateClocks.
*
* Arguments: port, points to the SDA port
* pin, the SDA pin number
* state, the state to compare SDA to (i.e. desired SDA state)
* timeout, the number of ms to wait before leaving function
*
* Returns: 1 if SDA pin is read to be state, 0 if timeout
*/

uint32_t Tickstart = HAL_GetTick();
uint8_t ret = 1;
/* Wait until flag is set */
while((state != HAL_GPIO_ReadPin(port, pin)) && (1 == ret)){
    /* Check for the timeout */
	if ((timeout == 0U) || (HAL_GetTick() - Tickstart >= timeout)){
		ret = 0;
	}
    asm("nop");
}
return ret;
}
/* USER CODE END Application */

