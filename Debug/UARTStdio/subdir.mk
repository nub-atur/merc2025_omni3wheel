################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../UARTStdio/uartstdio.c 

OBJS += \
./UARTStdio/uartstdio.o 

C_DEPS += \
./UARTStdio/uartstdio.d 


# Each subdirectory must supply rules for building sources it contributes
UARTStdio/%.o UARTStdio/%.su UARTStdio/%.cyclo: ../UARTStdio/%.c UARTStdio/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -DUSE_FULL_LL_DRIVER -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I"C:/Users/quocv/Downloads/MERC2025-master/MERC2025-master/UARTStdio" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-UARTStdio

clean-UARTStdio:
	-$(RM) ./UARTStdio/uartstdio.cyclo ./UARTStdio/uartstdio.d ./UARTStdio/uartstdio.o ./UARTStdio/uartstdio.su

.PHONY: clean-UARTStdio

