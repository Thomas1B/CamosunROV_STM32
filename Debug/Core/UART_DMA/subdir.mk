################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/UART_DMA/UART_DMA.c 

OBJS += \
./Core/UART_DMA/UART_DMA.o 

C_DEPS += \
./Core/UART_DMA/UART_DMA.d 


# Each subdirectory must supply rules for building sources it contributes
Core/UART_DMA/%.o Core/UART_DMA/%.su Core/UART_DMA/%.cyclo: ../Core/UART_DMA/%.c Core/UART_DMA/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Core/Bar30 -I../Core/BNO055 -I../Core/Thermistor -I../Core/Motors -I../Core/MyUtils -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-UART_DMA

clean-Core-2f-UART_DMA:
	-$(RM) ./Core/UART_DMA/UART_DMA.cyclo ./Core/UART_DMA/UART_DMA.d ./Core/UART_DMA/UART_DMA.o ./Core/UART_DMA/UART_DMA.su

.PHONY: clean-Core-2f-UART_DMA

