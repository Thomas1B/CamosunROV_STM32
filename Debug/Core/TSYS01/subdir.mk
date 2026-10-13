################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/TSYS01/TSYS01.c 

OBJS += \
./Core/TSYS01/TSYS01.o 

C_DEPS += \
./Core/TSYS01/TSYS01.d 


# Each subdirectory must supply rules for building sources it contributes
Core/TSYS01/%.o Core/TSYS01/%.su Core/TSYS01/%.cyclo: ../Core/TSYS01/%.c Core/TSYS01/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Core/UART_DMA -I../Core/Bar30 -I../Core/BNO055 -I../Core/Thermistor -I../Core/Motors -I../Core/TSYS01 -I../Core/MyUtils -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-TSYS01

clean-Core-2f-TSYS01:
	-$(RM) ./Core/TSYS01/TSYS01.cyclo ./Core/TSYS01/TSYS01.d ./Core/TSYS01/TSYS01.o ./Core/TSYS01/TSYS01.su

.PHONY: clean-Core-2f-TSYS01

