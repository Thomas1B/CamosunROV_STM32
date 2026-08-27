################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Bar30/bar30.c 

OBJS += \
./Core/Bar30/bar30.o 

C_DEPS += \
./Core/Bar30/bar30.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Bar30/%.o Core/Bar30/%.su Core/Bar30/%.cyclo: ../Core/Bar30/%.c Core/Bar30/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Core/Bar30 -I../Core/BNO055 -I../Core/Thermistor -I../Core/Motors -I../Core/MyUtils -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Bar30

clean-Core-2f-Bar30:
	-$(RM) ./Core/Bar30/bar30.cyclo ./Core/Bar30/bar30.d ./Core/Bar30/bar30.o ./Core/Bar30/bar30.su

.PHONY: clean-Core-2f-Bar30

