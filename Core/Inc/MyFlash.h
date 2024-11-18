/*
 * MyFlash.h
 *
 *  Created on: Nov 8, 2024
 *      Author: Алексей
 */

#ifndef INC_MYFLASH_H_
#define INC_MYFLASH_H_
#include "stdio.h"
#include <stdlib.h>
#include "stm32g0xx_hal.h"
void clearFlash();
void WriteDeviceAddressOffset(uint8_t* data, int size, int offset);
void ReadDeviceAddressOffset(uint8_t* Dout, int size, int offset);

#endif /* INC_MYFLASH_H_ */
