/*
 * MyFlash.c
 *
 *  Created on: 18 мар. 2022 г.
 *      Author: User
 */
#include "MyFlash.h"
#include "main.h"
#define CONFIGURATION_START_ADDR 0x0801f800
void clearFlash(){

	static FLASH_EraseInitTypeDef EraseInitStruct;
	/* Get the 1st sector to erase */
	uint32_t FirstPage = 63;//flash memory sector
	/* Get the number of sector to erase from 1st sector*/
	uint32_t NbOfPages = 1;
	//uint32_t Flash_BANK = 1;

	/* Fill EraseInit structure*/
	EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
	//EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
	//EraseInitStruct.Banks = FLASH_BANK_1;
	EraseInitStruct.Page = FirstPage;
	EraseInitStruct.NbPages = NbOfPages;
	uint32_t PageError = 0;
	HAL_FLASH_Lock();
	HAL_FLASH_Unlock();
	//FLASH_PageErase(FLASH_BANK_1,FirstPage);
	if(HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK)
	{
		/*Error occurred while sector erase.
User can add here some code to deal with this error.
SectorError will contain the faulty sector and then to know the code error on this sector,
user can call function 'HAL_FLASH_GetError()'
		 */
		uint32_t errorcode = HAL_FLASH_GetError();
		/*FLASH_ErrorTypeDef errorcode = HAL_FLASH_GetError();
		 */
		//Error_Handler();
		osDelay(1);
	}
	//CLEAR_BIT(FLASH->CR, FLASH_CR_PER);

	HAL_FLASH_Lock();

}
void WriteDeviceAddressOffset(uint8_t* data, int size, int offset) {
	uint32_t Address = CONFIGURATION_START_ADDR+offset;
	HAL_FLASH_Lock();
	HAL_FLASH_Unlock();
	//osDelay(10);
	for (int i = 0; i<size; i+=8){
		uint64_t data_64 = *(uint64_t*)(&data[i]);
		if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, Address+i, data_64) != HAL_OK){
			/* Error occurred while writing data in Flash memory.
User can add here some code to deal with this error */
			/*
FLASH_ErrorTypeDef errorcode = HAL_FLASH_GetError();
			 */
			//Error_Handler();
			uint32_t errorcode = HAL_FLASH_GetError();
			osDelay(1);
			break;
		}
	}
	/* Lock the Flash to disable the flash control register access (recommended
	to protect the FLASH memory against possible unwanted operation) *********/
	HAL_FLASH_Lock();
}
void ReadDeviceAddressOffset(uint8_t* Dout, int size, int offset)
{
	uint32_t Address = CONFIGURATION_START_ADDR+offset;

	for (int i = 0; i<size; i++){
		Dout[i] = *(__IO uint8_t*)(Address+i);
	}
}
