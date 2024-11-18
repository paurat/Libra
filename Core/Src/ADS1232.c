/*
 * ADS1232.c
 *
 *  Created on: Aug 2, 2024
 *
 */

#include "main.h"
#include <stdbool.h>
static int counter_ads=0; //пакет ads1232-24 бита, макс. значение счетчика-48
static int counter_ads_max=50;
static int MISO_Val=0;
static int bit_ads=0;
static uint32_t value_ads;
TIM_HandleTypeDef *htim;
uint32_t Period_htim;
//uint32_t ARR_Chek;
TIM_TypeDef *TIM;
GPIO_TypeDef *IN_PORT;
GPIO_TypeDef *OUT_PORT;
GPIO_TypeDef *PDWN_PORT;
uint16_t IN_PIN;
uint16_t OUT_PIN;
uint16_t PDWDN_PIN;

void (*ads_callback) (int) = NULL;

void set_ADS_pins(GPIO_TypeDef *in_port,
				  GPIO_TypeDef *out_port,
				  GPIO_TypeDef *pdwn_port,
				  uint16_t in_pin,
				  uint16_t out_pin,
				  uint16_t pdwdn_pin,
				  void(*f)(int))
{
	IN_PORT = in_port;
	OUT_PORT = out_port;
	PDWN_PORT=pdwn_port;
	IN_PIN=in_pin;
	OUT_PIN=out_pin;
	PDWDN_PIN=pdwdn_pin;
	ads_callback = f;
}

void Timer_Event() {

	MISO_Val=HAL_GPIO_ReadPin(IN_PORT, IN_PIN);
	if (counter_ads==0 && MISO_Val==1){
		return;
	}
	if (counter_ads==0){
		//TIM->ARR=((Period_htim+1)*10)-1;
		TIM->ARR=Period_htim;
		//ARR_Chek=TIM->ARR;
	}
	if (counter_ads%2==0){
		HAL_GPIO_WritePin(OUT_PORT, OUT_PIN, GPIO_PIN_SET);
		counter_ads++;
	}
	else{
		HAL_GPIO_WritePin(OUT_PORT, OUT_PIN, GPIO_PIN_RESET);
		if (counter_ads_max-2>=counter_ads){
			value_ads|=(MISO_Val<<23-bit_ads);//23-bit_ads отзеркаливаем значение value_ads

		}

		counter_ads++;
		bit_ads++;
	}
	if(counter_ads==counter_ads_max){
		//HAL_TIM_Base_Stop_IT(htim);
		HAL_GPIO_WritePin(OUT_PORT, OUT_PIN, GPIO_PIN_RESET);
		if (ads_callback != NULL)
		{
			ads_callback(value_ads);
		}

		TIM->ARR=((TIM->ARR+1)*10)-1;
		//ARR_Chek=TIM->ARR;
		counter_ads=0;
		value_ads=0;
		bit_ads=0;
		//HAL_GPIO_WritePin(PDWN_PORT, PDWDN_PIN, GPIO_PIN_RESET);
	}
}
void Start_read(TIM_HandleTypeDef *htim_ptr, TIM_TypeDef *TIM_ads){
	HAL_GPIO_WritePin(PDWN_PORT, PDWDN_PIN, GPIO_PIN_SET);
	if (counter_ads>0&&counter_ads<counter_ads_max){
		return;
	}
	htim = htim_ptr;
	TIM = TIM_ads;
	Period_htim=htim->Init.Period;
	counter_ads=0;
	value_ads=0;
	bit_ads=0;
	HAL_GPIO_WritePin(OUT_PORT, OUT_PIN, GPIO_PIN_RESET);
	//HAL_TIM_Base_Start_IT(htim);
}



//bool readDataADS(uint32_t *data_ads){
//	*data_ads=value_ads;
//	return (counter_ads==counter_ads_max);
//}



