################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/BNO055/bno055.c 

OBJS += \
./Core/BNO055/bno055.o 

C_DEPS += \
./Core/BNO055/bno055.d 


# Each subdirectory must supply rules for building sources it contributes
Core/BNO055/%.o Core/BNO055/%.su Core/BNO055/%.cyclo: ../Core/BNO055/%.c Core/BNO055/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Core/BNO055 -I../Core/Thermistor -I../Core/Motors -I../Core/MyUtils -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-BNO055

clean-Core-2f-BNO055:
	-$(RM) ./Core/BNO055/bno055.cyclo ./Core/BNO055/bno055.d ./Core/BNO055/bno055.o ./Core/BNO055/bno055.su

.PHONY: clean-Core-2f-BNO055

