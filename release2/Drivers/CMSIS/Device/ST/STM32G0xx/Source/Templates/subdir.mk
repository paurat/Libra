################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/CMSIS/Device/ST/STM32G0xx/Source/Templates/system_stm32g0xx.c 

C_DEPS += \
./Drivers/CMSIS/Device/ST/STM32G0xx/Source/Templates/system_stm32g0xx.d 

OBJS += \
./Drivers/CMSIS/Device/ST/STM32G0xx/Source/Templates/system_stm32g0xx.o 


# Each subdirectory must supply rules for building sources it contributes
Drivers/CMSIS/Device/ST/STM32G0xx/Source/Templates/%.o Drivers/CMSIS/Device/ST/STM32G0xx/Source/Templates/%.su Drivers/CMSIS/Device/ST/STM32G0xx/Source/Templates/%.cyclo: ../Drivers/CMSIS/Device/ST/STM32G0xx/Source/Templates/%.c Drivers/CMSIS/Device/ST/STM32G0xx/Source/Templates/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -DUSE_HAL_DRIVER -DSTM32G071xx '-DCMSIS_device_header=<stm32g0xx.h>' -c -I../Core/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G0xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM0 -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Drivers-2f-CMSIS-2f-Device-2f-ST-2f-STM32G0xx-2f-Source-2f-Templates

clean-Drivers-2f-CMSIS-2f-Device-2f-ST-2f-STM32G0xx-2f-Source-2f-Templates:
	-$(RM) ./Drivers/CMSIS/Device/ST/STM32G0xx/Source/Templates/system_stm32g0xx.cyclo ./Drivers/CMSIS/Device/ST/STM32G0xx/Source/Templates/system_stm32g0xx.d ./Drivers/CMSIS/Device/ST/STM32G0xx/Source/Templates/system_stm32g0xx.o ./Drivers/CMSIS/Device/ST/STM32G0xx/Source/Templates/system_stm32g0xx.su

.PHONY: clean-Drivers-2f-CMSIS-2f-Device-2f-ST-2f-STM32G0xx-2f-Source-2f-Templates

