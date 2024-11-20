/* USER CODE BEGIN Header */
// This is an open source non-commercial project. Dear PVS-Studio, please check it.
// PVS-Studio Static Code Analyzer for C, C++, C#, and Java: https://pvs-studio.com
/**
 ******************************************************************************
 * File Name          : app_freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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
#include "spi.h"
#include "i2c.h"
#include "usart.h"

#include "hardware_check.h"
#include "stm32g0xx_it.h"
#include <string.h>
#include <stdio.h>
#include <stdint.h>

#include <math.h>
#include "MyFlash.h"
#include <stdlib.h>

#include "lis331dlh.h"
#include "calc_length.h"
#include "debug.h"

#include "lps22.h"
#include "hdc1080.h"
#include "sensors_state.h"

#include <stdbool.h>
#include "rtc.h"
#include "ADS1232.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
	PARSER_S4x = 0,
		PARSER_Sxx,
		PARSER_S98,
		PARSER_MSV70,
		PARSER_BDR,
		PARSER_MSV7,
		PARSER_IDN7,
		PARSER_ADR7,
		PARSER_ADRNUM,
		PARSER_EMPT
} parser_state_t; //7==?


struct SensorsState sensorsState;

typedef struct {
	uint8_t number;
	uint8_t number_ch;
} platform_switches_state_t;

void ADS_Callback(uint32_t value);
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define timer_period 600000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
uint8_t transmitting_command[22];
uint8_t received_command[22];	//буфер приема данных от терминала
uint16_t received_number = 0;
//uint32_t received_BDR = 0;
uint32_t ads_val;
uint8_t is_data_received = 0;
uint32_t test1 = 0x78730B00;
uint32_t test2 = 0x000B7378;
int IDN=0;
uint8_t RX_command_buff[1]={0};
int RX_command_count=0;
int MSV0 = 0;
int MSV = 0;
int ADR = 0;
int END_Cmd=0;
parser_state_t terminal_parser_state = PARSER_EMPT;

LPS22HB_sensor LPS_data;

HDC_sensor HDC_config = {
		.temperature_resolution = Temperature_Resolution_14_bit,
		.humidity_resolution = Humidity_Resolution_14_bit,
};

lis331dlh_t config = {
	.data_rate = DATA_RATE_100,
	.range = FULL_SCALE_2G,
};

platform_switches_state_t platform_number= {'0','0'};

//const char str_idn[]="HBM,C16iC3/30t     ,3319402,P80\r\n";
struct sensor_inf {
	uint8_t platform_adr[2];
	uint32_t received_BDR;
	//uint8_t crc_platform;
} sensor_inf __attribute__((aligned(8)));
int offset = 0;
uint16_t serial_number =0;// СЕРИЙНЫЙ НОМЕР ПЛАТЫ (от 0 до 65535 включительно)
int16_t serial_number_control=0;
bool is_error = 0;
bool case_opened = 0;

bool period_expired = false;

float maximum_move_in_period = 0;
float max_acceleration_in_period = 0;

extern TIM_HandleTypeDef htim2;


/* USER CODE END Variables */
/* Definitions for debugTask */
osThreadId_t debugTaskHandle;
const osThreadAttr_t debugTask_attributes = {
  .name = "debugTask",
  .priority = (osPriority_t) osPriorityRealtime7,
  .stack_size = 384 * 4
};
/* Definitions for rxCommandsTask */
osThreadId_t rxCommandsTaskHandle;
const osThreadAttr_t rxCommandsTask_attributes = {
  .name = "rxCommandsTask",
  .priority = (osPriority_t) osPriorityRealtime1,
  .stack_size = 384 * 4
};
/* Definitions for accelTask */
osThreadId_t accelTaskHandle;
const osThreadAttr_t accelTask_attributes = {
  .name = "accelTask",
  .priority = (osPriority_t) osPriorityRealtime3,
  .stack_size = 1024 * 4
};
/* Definitions for sensorsPolling */
osThreadId_t sensorsPollingHandle;
const osThreadAttr_t sensorsPolling_attributes = {
  .name = "sensorsPolling",
  .priority = (osPriority_t) osPriorityRealtime7,
  .stack_size = 384 * 4
};
/* Definitions for ADS1232Task */
osThreadId_t ADS1232TaskHandle;
const osThreadAttr_t ADS1232Task_attributes = {
  .name = "ADS1232Task",
  .priority = (osPriority_t) osPriorityRealtime2,
  .stack_size = 256 * 4
};
/* Definitions for maximumsPeriodTimer */
osTimerId_t maximumsPeriodTimerHandle;
const osTimerAttr_t maximumsPeriodTimer_attributes = {
  .name = "maximumsPeriodTimer"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void check_errors();

void get_platform_number() {

//	platform_number.number |= (!HAL_GPIO_ReadPin(PLATFORM_ADDR_1_GPIO_Port, PLATFORM_ADDR_1_Pin) << 0);
//	platform_number.number |= (!HAL_GPIO_ReadPin(PLATFORM_ADDR_2_GPIO_Port, PLATFORM_ADDR_2_Pin) << 1);
//	platform_number.number |= (!HAL_GPIO_ReadPin(PLATFORM_ADDR_3_GPIO_Port, PLATFORM_ADDR_3_Pin) << 2);
//	platform_number.number |= (!HAL_GPIO_ReadPin(PLATFORM_ADDR_4_GPIO_Port, PLATFORM_ADDR_4_Pin) << 3);
//
//	for (int i = 0; i < 4; i++) {
//		if (platform_number.number & (1 << i)) {
//			platform_number.number = i;
//			break;
//		}
//	}
//
//	platform_number.number_ch = '0' + platform_number.number;
//
//	debug("PLATFORM NUMBER: %d \r\n", platform_number.number);
}


/* USER CODE END FunctionPrototypes */

void StartDebugTask(void *argument);
void StartTaskRxCommands(void *argument);
void StartTaskAccelerometer(void *argument);
void StartSensorsPolling(void *argument);
void StartADS1232Task(void *argument);
void maximumsPeriodTimer_callback(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

	serial_number =(uint16_t)(crc32b((uint8_t *)UID_BASE, 8));
	serial_number_control = abs((int16_t)serial_number);
	ReadDeviceAddressOffset((uint8_t*) &sensor_inf, sizeof(sensor_inf), offset);
	//memset(transmitting_command, 0, sizeof(transmitting_command));
	while (sensor_inf.platform_adr[0]!=0xFF)
	{
		if(offset>=248){
//
			offset=0;
			clearFlash();
			break;
		}
		offset+=sizeof(sensor_inf);
		ReadDeviceAddressOffset((uint8_t*) &sensor_inf, sizeof(sensor_inf), offset);
	}
	if(offset<8){

		//ReadDeviceAddressOffset((uint8_t*) &sensor_inf, sizeof(sensor_inf), offset);
		memset(&sensor_inf, 0, sizeof(sensor_inf));
		sensor_inf.platform_adr[0]='0';
		sensor_inf.platform_adr[1]='1';
		sensor_inf.received_BDR=38400;
		WriteDeviceAddressOffset((uint8_t*) &sensor_inf, sizeof(sensor_inf), offset);
		offset+=sizeof(sensor_inf);
	}
	if(offset>=8){

		offset-=sizeof(sensor_inf);
		ReadDeviceAddressOffset((uint8_t*) &sensor_inf, sizeof(sensor_inf), offset);
		offset+=sizeof(sensor_inf);
	}

	 while (!(USART2->ISR & USART_ISR_TC)) {
	 // Ожидание, пока передача завершится
	 }

	 // Отключаем USART перед изменением настроек
	 USART2->CR1 &= ~USART_CR1_UE;
     // Изменение скорости
	 USART2->BRR = (SystemCoreClock+12800) / sensor_inf.received_BDR;
	 // Включаем USART обратно
	 USART2->CR1 |= USART_CR1_UE;

	//ReadDeviceAddressOffset((uint8_t*) &sensor_inf, sizeof(sensor_inf), offset);
	//offset+=sizeof(sensor_inf);
	//clearFlash();
	//WriteDeviceAddressOffset((uint8_t*)&sensor_inf, sizeof(sensor_inf), offset);
	sensorsState.hdc_fail = 0;
	sensorsState.lps_fail = 0;
	 set_ADS_pins(GPIOA,
	  			  GPIOA,
	  			  GPIOA,
	  			  GPIO_PIN_0,
	  			  GPIO_PIN_1,
				  GPIO_PIN_2,
				  &ADS_Callback);
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* creation of maximumsPeriodTimer */
  maximumsPeriodTimerHandle = osTimerNew(maximumsPeriodTimer_callback, osTimerPeriodic, NULL, &maximumsPeriodTimer_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of debugTask */
  debugTaskHandle = osThreadNew(StartDebugTask, NULL, &debugTask_attributes);

  /* creation of rxCommandsTask */
  rxCommandsTaskHandle = osThreadNew(StartTaskRxCommands, NULL, &rxCommandsTask_attributes);

  /* creation of accelTask */
  accelTaskHandle = osThreadNew(StartTaskAccelerometer, NULL, &accelTask_attributes);

  /* creation of sensorsPolling */
  sensorsPollingHandle = osThreadNew(StartSensorsPolling, NULL, &sensorsPolling_attributes);

  /* creation of ADS1232Task */
  ADS1232TaskHandle = osThreadNew(StartADS1232Task, NULL, &ADS1232Task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDebugTask */
/**
  * @brief  Function implementing the debugTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDebugTask */
void StartDebugTask(void *argument)
{
  /* USER CODE BEGIN StartDebugTask */

  /* Infinite loop */
	check_errors();

	get_platform_number();

	osTimerStart(maximumsPeriodTimerHandle, timer_period);	// запуск таймера с периодом срабатывания 10 минут
	debug("\r\nSN: %05d\r\n", serial_number);
	debug("\r\nMax mediums timer set to %d ms\r\n", timer_period);

	if (!debug_enabled()) {

		uint8_t message_sn[64] = { 0, };
		memset(message_sn, 0, sizeof(message_sn));

		int size = snprintf((char *)message_sn, sizeof(message_sn), "SN: %05d \r\n", serial_number);

		if (size > 0) {
			HAL_UART_Transmit(debug_uart, message_sn, size, 100);
		}

	}

	vTaskDelay(100);

	for (;;) {

		send_debug_messages();

		if (!debug_enabled()) {

			float max_acceleration = fmax(
					round_and_limit_float(get_max_positive_acceleration()),
					round_and_limit_float(get_max_negative_acceleration()));

			float maximum_move = fmax(
					round_and_limit_float(get_max_positive_move()),
					round_and_limit_float(get_max_negative_move()));

			uint8_t message[256] = { 0, };
			memset(message, 0, sizeof(message));

			int size = snprintf((char *)message, sizeof(message),
				"ACCEL: 0x%08X, LENGTH: 0x%08X, MAX_ACCEL: 0x%08X, MAX_LENGTH: 0x%08X, PRESSURE: %.2f HUMIDITY: %d, TEMP %.2f\r\n\nACCEL: %06.2f,     LENGTH: %06.2f,     MAX_ACCEL: %06.2f,     MAX_LENGTH: %06.2f\r\n\n",
				*(uint32_t*)&max_acceleration, *(uint32_t*)&maximum_move,
				*(uint32_t*)&max_acceleration_in_period, *(uint32_t*)&maximum_move_in_period,
				(LPS_data.last_pressure / 1024.0), HDC_config.last_humidity,
				HDC_config.last_temperature,
				max_acceleration, maximum_move,
				max_acceleration_in_period, maximum_move_in_period);

			if (size > 0) {
				HAL_UART_Transmit(debug_uart, message, size, 100);
			}

		}

		HAL_GPIO_TogglePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin);
		vTaskDelay(100);
	}

  /* USER CODE END StartDebugTask */
}

/* USER CODE BEGIN Header_StartTaskRxCommands */
/**
* @brief Function implementing the rxCommandsTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskRxCommands */
void StartTaskRxCommands(void *argument)
{
  /* USER CODE BEGIN StartTaskRxCommands */
	/* Infinite loop */
	uint32_t ulNotifiedValue;
	const TickType_t xBlockTime = pdMS_TO_TICKS( 500 );
	memset(received_command, 0x0, sizeof(received_command));
	memset(transmitting_command, 0x0, sizeof(transmitting_command));

	//receive(terminal_uart, received_command, 1);
	  receive(terminal_uart, RX_command_buff, 1);
	for (;;) {

		    ulNotifiedValue = ulTaskNotifyTake( pdFALSE, xBlockTime );

		    if( ulNotifiedValue == 0 )
			{
		    	continue;
			}

			HAL_GPIO_WritePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin, GPIO_PIN_SET);

			debug("Received <<%s>>\r\n", received_command);

			if (terminal_parser_state == PARSER_S4x) { // если посылка S4x;

				float maximum = fmax(
						round_and_limit_float(get_max_positive_move()),
						round_and_limit_float(get_max_negative_move()));

				float max_acceleration = fmax(
						round_and_limit_float(get_max_positive_acceleration()),
						round_and_limit_float(get_max_negative_acceleration()));

				uint8_t flags = 0;
				flags |= (case_opened << 0);
				flags |= (is_error << 1);
				if (is_error) is_error = false;// сбрасываем флаг ошибки после отправки на терминал

					transmitting_command[0] = HDC_config.last_temperature;
					transmitting_command[1] = HDC_config.last_humidity;
					transmitting_command[2] = flags;
					transmitting_command[3] = (((LPS_data.last_pressure / 1000) - 0.5) / 1.5 * 100);

					memcpy(&transmitting_command[4], &maximum, 4);
					memcpy(&transmitting_command[8], &max_acceleration, 4);
					memcpy(&transmitting_command[12], &maximum_move_in_period, 4);
					memcpy(&transmitting_command[16], &max_acceleration_in_period, 4);
					memcpy(&transmitting_command[20], &serial_number_control, 2);

					HAL_UART_Transmit_IT(terminal_uart, transmitting_command, 22);

					//memset(transmitting_command, 0, sizeof(transmitting_command));
					terminal_parser_state =	PARSER_EMPT;

					debug("Transmit to terminal: <%02x %02x %02x %02x  %02x %02x %02x %02x  %02x %02x %02x %02x  %02x %02x %02x %02x  %02x %02x %02x %02x  %02x %02x>",
							transmitting_command[0], transmitting_command[1],
							transmitting_command[2], transmitting_command[3],
							transmitting_command[4], transmitting_command[5],
							transmitting_command[6], transmitting_command[7],
							transmitting_command[8], transmitting_command[9],
							transmitting_command[10], transmitting_command[11],
							transmitting_command[12], transmitting_command[13],
							transmitting_command[14], transmitting_command[15],
							transmitting_command[16], transmitting_command[17],
							transmitting_command[18], transmitting_command[19],
							transmitting_command[20], transmitting_command[21]);

			}

			if (terminal_parser_state == PARSER_Sxx) { // если посылка S0x;



						uint8_t flags = 0;
						flags |= (case_opened << 0);
						flags |= (is_error << 1);
						if (is_error) is_error = false;// сбрасываем флаг ошибки после отправки на терминал
						IDN=1;
						if (MSV0==1&&ADR==0) {	// Анализируем третий символ, отвечающий за конкретный БК

							uint8_t buf[4] = {0,0,0,0};
							//uint32_t val = (ads_val*100)/421 ;
							//uint32_t val = (8388607*100)/421 ;
							    //uint32_t val =  1401366;
							uint32_t val = (ads_val);
									//	отправ	EE FF 0B 00
							//0x78730B00;
							    buf[3] = (val >> (2*8)) & 0xFF;
							    buf[2] = (val >> (1*8)) & 0xFF;
							    buf[1] = (val >> (0*8)) & 0xFF;
							    buf[0] = buf[1]^buf[2]^buf[3];

							HAL_UART_Transmit_IT(terminal_uart, buf, 4);
							debug("Transmit to terminal: <%x>",
									buf);
						}
						if (ADR==1&&MSV==0) {	// Анализируем третий символ, отвечающий за конкретный БК
							 char str_adr[20];
							 sprintf(str_adr,"%c%c\r\n",sensor_inf.platform_adr[0],sensor_inf.platform_adr[1]);
							 HAL_UART_Transmit_IT(terminal_uart, (uint8_t*)str_adr, strlen(str_adr));

							 ADR=0;
						}
						if (MSV==1&&ADR==0&&MSV0==0) {	// Анализируем третий символ, отвечающий за конкретный БК

						   uint8_t buf[4] = {0,0,0,0};
													//uint32_t val = (ads_val*100)/421 ;
													//uint32_t val = (8388607*100)/421 ;
													    //uint32_t val =  1401366;
						   uint32_t val = (ads_val);
															//	отправ	EE FF 0B 00
													//0x78730B00;
						   buf[3] = (val >> (2*8)) & 0xFF;
						   buf[2] = (val >> (1*8)) & 0xFF;
						   buf[1] = (val >> (0*8)) & 0xFF;
						   buf[0] = buf[1]^buf[2]^buf[3];

							HAL_UART_Transmit_IT(terminal_uart, buf, 4);
							debug("Transmit to terminal: <%x>",
							buf);

							MSV=0;
						}


							terminal_parser_state =	PARSER_EMPT;
					}

			if (terminal_parser_state == PARSER_MSV70) { // если посылка Sxx;



									uint8_t flags = 0;
									flags |= (case_opened << 0);
									flags |= (is_error << 1);
									if (is_error) is_error = false;// сбрасываем флаг ошибки после отправки на терминал

										// Анализируем третий символ, отвечающий за конкретный БК

										MSV0=1;
										//HAL_UART_Transmit_IT(terminal_uart, &MSV, 1);
										debug("Transmit to terminal: <%x>",
												&MSV0);
										terminal_parser_state =	PARSER_EMPT;

								}
			if (terminal_parser_state == PARSER_MSV7) { // если посылка Sxx;



					uint8_t flags = 0;
					flags |= (case_opened << 0);
					flags |= (is_error << 1);
				if (is_error) is_error = false;// сбрасываем флаг ошибки после отправки на терминал

						// Анализируем третий символ, отвечающий за конкретный БК

						MSV=1;
				//HAL_UART_Transmit_IT(terminal_uart, &MSV, 1);
				debug("Transmit to terminal: <%x>",&MSV);
				terminal_parser_state =	PARSER_EMPT;

			}
			if (terminal_parser_state == PARSER_ADR7) { // если посылка S0x;



				uint8_t flags = 0;
				flags |= (case_opened << 0);
				flags |= (is_error << 1);
				if (is_error) is_error = false;// сбрасываем флаг ошибки после отправки на терминал

				// Анализируем третий символ, отвечающий за конкретный БК
				ADR=1;
				terminal_parser_state =	PARSER_EMPT;

				}
			if (terminal_parser_state == PARSER_ADRNUM) { // если посылка S0x;


				uint8_t flags = 0;
				flags |= (case_opened << 0);
				flags |= (is_error << 1);
				if (is_error) is_error = false;// сбрасываем флаг ошибки после отправки на терминал
				received_number=0;
				for (int i = 7; i < 14; i++) {
					if (received_command[i] >= '0' && received_command[i] <= '9') {
						received_number = received_number * 10 + (received_command[i] - '0');
					}

				}

				// Проверка serial_number
				 if (received_number == serial_number) {
					 memset(sensor_inf.platform_adr, '0', sizeof(sensor_inf.platform_adr));
					 sensor_inf.platform_adr[0]=received_command[3];
					 sensor_inf.platform_adr[1]=received_command[4];
					// clearFlash();
					 //offset=0;
					 if(offset>=248){

					 		offset=0;
					 		clearFlash();
					 	}
					 taskENTER_CRITICAL();
					 WriteDeviceAddressOffset((uint8_t*) &sensor_inf, sizeof(sensor_inf), offset);
					 taskEXIT_CRITICAL();

					 offset+=sizeof(sensor_inf);
				 }

				 //sensor_inf.crc_platform=(uint8_t)(crc32b((uint8_t *)sensor_inf.platform_adr, 2));
				 terminal_parser_state =PARSER_EMPT;

			}

			if (terminal_parser_state == PARSER_BDR) { // если посылка S0x;


							uint8_t flags = 0;
							flags |= (case_opened << 0);
							flags |= (is_error << 1);
							if (is_error) is_error = false;// сбрасываем флаг ошибки после отправки на терминал
							sensor_inf.received_BDR=0;

							for (int i = 0; i < 22; i++) {
								if (received_command[i] != ';') {
									END_Cmd = END_Cmd+1;
								}
								if (received_command[i] == ';') {
									i=22;
								}
							}

							for (int i = 3; i < END_Cmd; i++) {
								if (received_command[i] >= '0' && received_command[i] <= '9') {
									sensor_inf.received_BDR = sensor_inf.received_BDR * 10 + (received_command[i] - '0');
								}

							}

				  while (!(USART2->ISR & USART_ISR_TC)) {
				         // Ожидание, пока передача завершится
				     }

				     // Отключаем USART перед изменением настроек
				     USART2->CR1 &= ~USART_CR1_UE;

				     // Изменение скорости
				     USART2->BRR = (SystemCoreClock+12800) / sensor_inf.received_BDR;

				     // Включаем USART обратно
				     USART2->CR1 |= USART_CR1_UE;

				     taskENTER_CRITICAL();
				     WriteDeviceAddressOffset((uint8_t*) &sensor_inf, sizeof(sensor_inf), offset);
				     taskEXIT_CRITICAL();
				     offset+=sizeof(sensor_inf);

							 terminal_parser_state =PARSER_EMPT;

						}



			if (terminal_parser_state == PARSER_IDN7) { // если посылка S0x;



				uint8_t flags = 0;
				flags |= (case_opened << 0);
				flags |= (is_error << 1);
				if (is_error) is_error = false;// сбрасываем флаг ошибки после отправки на терминал
				if(IDN==1){
				char str_idn[50];
				sprintf(str_idn,"CAS,BCA5/5kg     ,%d,P80\r\n",serial_number);

					HAL_UART_Transmit_IT(terminal_uart, (uint8_t*)str_idn, strlen(str_idn));
					IDN=0;
					terminal_parser_state =	PARSER_EMPT;
					//debug("Transmit to terminal: <%x>", &str_idn);
				}
			}

			memset(received_command, 0, sizeof(received_command));
			//receive(terminal_uart, received_command, 1);
			//receive(terminal_uart, RX_command_buff, 1);
			debug("Receive from task\r\n");
		}
  /* USER CODE END StartTaskRxCommands */
}

/* USER CODE BEGIN Header_StartTaskAccelerometer */
/**
* @brief Function implementing the accelTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskAccelerometer */
void StartTaskAccelerometer(void *argument)
{
  /* USER CODE BEGIN StartTaskAccelerometer */
  /* Infinite loop */
	if(lis331dlh_init_spi(&config, ACCELEROMETER_SPI, ACCEL_CS_GPIO_Port, ACCEL_CS_Pin)) {
		debug("LIS331DLH DOESN'T WORK OR DOESN'T SOLDERED");
		HardFault_Handler();
	}
	lis331dlh_update_config(&config);

//	lis331dlh_print_config(&config);
	// нахождение угла наклона
	find_degree(&config);

	while (1) {

		// проверка готовности
		lis331dlh_read_status(&config);
		check_overrun(&config);

		if (is_measurement_ready(&config)) {
			// обновление данных с датчика ускорения
			lis331dlh_update_accelaration(&config);

			// сброс максимумом и минимумов по таймерам
			reset_move_maximums();
			reset_acceleration_maximums();

			// алгоритм вычисления длины перемещений
			calc_length(&config);
		}

		vTaskDelay(5);

		float maximum = fmax(
				round_and_limit_float(get_max_positive_move()),
				round_and_limit_float(get_max_negative_move()));

		float max_acceleration = fmax(
				round_and_limit_float(get_max_positive_acceleration()),
				round_and_limit_float(get_max_negative_acceleration()));

		if(period_expired) {
			debug("\r\nTaskAccelerometer: Timer done. current maximum = %f maximum_in_period = %f current max_acceleration = %f max_acceleration_in_period = %f \r\n",
					maximum, maximum_move_in_period,
					max_acceleration, max_acceleration_in_period);
			period_expired = false;
			maximum_move_in_period = 0;
			max_acceleration_in_period = 0;
			debug("\r\nTaskAccelerometer: Timer resetted. maximum_in_period = %f max_acceleration_in_period = %f \r\n", maximum_move_in_period, max_acceleration_in_period);
		}

		if(maximum > maximum_move_in_period) maximum_move_in_period = maximum;
		if(max_acceleration > max_acceleration_in_period) max_acceleration_in_period = max_acceleration;
	}
  /* USER CODE END StartTaskAccelerometer */
}

/* USER CODE BEGIN Header_StartSensorsPolling */
/**
* @brief Function implementing the sensorsPolling thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSensorsPolling */
void StartSensorsPolling(void *argument)
{
  /* USER CODE BEGIN StartSensorsPolling */
 /* Infinite loop */

	//vTaskDelay(200);
	taskENTER_CRITICAL();
	if(!hdc1080_init(SENSORS_I2C, HDC_config.temperature_resolution, HDC_config.humidity_resolution)) {
		debug("HDC1080 DOESN'T WORK OR DOESN'T SOLDERED");
		HardFault_Handler();
	}

	if(!lps22hb_init(SENSORS_I2C)) {
		debug("LPS22HB DOESN'T WORK OR DOESN'T SOLDERED");
		HardFault_Handler();
	}
	taskEXIT_CRITICAL();
			osDelay(200);

	for (;;) {

		uint8_t status = lps22hb_get_status(&config);
		lps22hb_check_pressure_overrun(status);
		lps22hb_check_temperature_overrun(status);

		if (lps22hb_check_pressure_data_available(status) && lps22hb_check_temperature_data_available(status))
		{
			hdc1080_start_measurement(&HDC_config.last_temperature, &HDC_config.last_humidity);
						debug("HDC1080 TEMP: %f HDC1080 HUMIDITY: %f\n\r",
								HDC_config.last_temperature,
								HDC_config.last_humidity);

						LPS_data.last_pressure = lps22hb_read_pressure();
						LPS_data.last_temperature = lps22hb_read_temperature();
						debug("LPS22HB TEMP: %f LPS22HB PRESSURE: %f\n\r",
								LPS_data.last_temperature,
								LPS_data.last_pressure);
//			hdc1080_start_measurement(&HDC_config.last_temperature, &HDC_config.last_humidity);
//			debug("HDC1080 RAW TEMP: 0x%4x RAW HUMIDITY: 0x%4x\r\n",
//					HDC_config.last_temperature,
//					HDC_config.last_humidity);
//
//			LPS_data.last_pressure = lps22hb_read_pressure();
//			LPS_data.last_temperature = lps22hb_read_temperature();
//			debug("LPS22HB RAW TEMP: 0x%4x RAW PRESSURE: 0x%4x\r\n",
//					LPS_data.last_temperature,
//					LPS_data.last_pressure);

			vTaskDelay(1000);
//						char last_temperatureLps_prin[50]; //size of the number
//														int len4 =  sprintf(last_temperatureLps_prin, "tempLPS: %f\n\r", LPS_data.last_temperature);
//
//
//														HAL_UART_Transmit (&huart3, last_temperatureLps_prin, len4, 100);


		}
		else {
			vTaskDelay(10);
		}

//		if(HAL_GPIO_ReadPin(TAMPER_GPIO_Port, TAMPER_Pin) == GPIO_PIN_RESET){
//			case_opened = true;
		//}
	}
  /* USER CODE END StartSensorsPolling */
}

/* USER CODE BEGIN Header_StartADS1232Task */
/**
* @brief Function implementing the ADS1232Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartADS1232Task */
void StartADS1232Task(void *argument)
{
  /* USER CODE BEGIN StartADS1232Task */
  /* Infinite loop */

	HAL_TIM_Base_Start_IT(&htim2);
	Start_read(&htim2, TIM2);

  for(;;)
  {
	 // Start_read(&htim2, TIM2);
	      vTaskDelay(100);


  }
  /* USER CODE END StartADS1232Task */
}

/* maximumsPeriodTimer_callback function */
void maximumsPeriodTimer_callback(void *argument)
{
  /* USER CODE BEGIN maximumsPeriodTimer_callback */
	debug("\r\n===Timer 10 minutes: reloaded===\r\n");
	period_expired = true;
  /* USER CODE END maximumsPeriodTimer_callback */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {	//Callback-функция завершения приема данных

	BaseType_t xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken = pdFALSE;

	if(huart == &huart2) {
			//extended_debug("Current state = %d, receive <%c>\r\n", terminal_parser_state, received_command[terminal_parser_state]);

			if ((RX_command_buff[0] == 'S'||RX_command_buff[0] == 'M'||RX_command_buff[0] == 'I'||RX_command_buff[0] == 'B'||RX_command_buff[0] == 'A')&&RX_command_count==0) {
				received_command[RX_command_count]=RX_command_buff[0];
				RX_command_count=RX_command_count+1;
			}
			else if (RX_command_buff[0]!= ';'&&RX_command_count!=0&&RX_command_count<=17) {
				received_command[RX_command_count]=RX_command_buff[0];
				RX_command_count=RX_command_count+1;
			}
			else if (RX_command_buff[0]== ';'&&RX_command_count!=0) {
				received_command[RX_command_count]=RX_command_buff[0];
				RX_command_count=0;



				if (received_command[0]=='S'){

					if (received_command[1]=='4'&&received_command[2]==platform_number.number_ch){

						terminal_parser_state = PARSER_S4x;

					}
					else if (received_command[1]==sensor_inf.platform_adr[0]&& (received_command[2]==(sensor_inf.platform_adr[1]))){

						terminal_parser_state = PARSER_Sxx;
					}
					else if (received_command[1]=='9'&&received_command[2]=='8'){

						terminal_parser_state = PARSER_S98; //(Ничего не отвечаем)
					}
				}
				else if (received_command[0]=='M'&&received_command[1]=='S'&&received_command[2]=='V'&&received_command[3]=='?'&&received_command[4]=='0') {

					terminal_parser_state = PARSER_MSV70;
				}
				else if (received_command[0]=='M'&&received_command[1]=='S'&&received_command[2]=='V'&&received_command[3]=='?'&&received_command[4]!='0') {

					terminal_parser_state = PARSER_MSV7;
				}
				else if (received_command[0]=='I'&&received_command[1]=='D'&&received_command[2]=='N'&&received_command[3]=='?') {

					terminal_parser_state = PARSER_IDN7;
				}
				else if (received_command[0]=='A'&&received_command[1]=='D'&&received_command[2]=='R'&&received_command[3]=='?') {

					terminal_parser_state = PARSER_ADR7;
				}
				else if (received_command[0]=='A'&&received_command[1]=='D'&&received_command[2]=='R'&&received_command[3]!='?') {

					terminal_parser_state = PARSER_ADRNUM;
					}
				else if (received_command[0]=='B'&&received_command[1]=='D'&&received_command[2]=='R') {

				     terminal_parser_state = PARSER_BDR;
				}
				RX_command_count = 0;
				RX_command_buff[0] = 0;
				vTaskNotifyGiveFromISR( rxCommandsTaskHandle, &xHigherPriorityTaskWoken );
			}
		extended_debug("New state = %d\r\n", terminal_parser_state);
		HAL_UART_Receive_IT(terminal_uart, RX_command_buff, 1);
		extended_debug("Receive from handler\r\n");
		}



}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {	//Callback-функция завершения передачи данных
	if (huart == debug_uart) {
		reset_debug_variables();
	}
}

void check_errors() {
	if(HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR0) == 0xBBBB){
		HAL_GPIO_WritePin(LED_ERROR_GPIO_Port, LED_ERROR_Pin, GPIO_PIN_SET);
		HAL_PWR_EnableBkUpAccess();
		HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR0, 0x0000);
		is_error = true;
		debug("==================== REBOOT REASON: HardFault ====================\r\n");
	}

	if(HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR0) == 0xAAAA){
		HAL_GPIO_WritePin(LED_ERROR_GPIO_Port, LED_ERROR_Pin, GPIO_PIN_SET);
		HAL_PWR_EnableBkUpAccess();
		HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR0, 0x0000);
		is_error = true;
		debug("==================== REBOOT REASON: ErrorHandler ====================\r\n");
	}
}
void ADS_Callback(uint32_t value){

	 ads_val = value;
		char ADS_val_prin[50]; //size of the number
		int len5 =  sprintf(ADS_val_prin, "ADS_VAL: %d\n\r", ads_val);

		HAL_UART_Transmit (&huart3, ADS_val_prin, len5, 100);

}
/* USER CODE END Application */

