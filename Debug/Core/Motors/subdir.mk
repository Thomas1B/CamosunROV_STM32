################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Motors/motors.c 

OBJS += \
./Core/Motors/motors.o 

C_DEPS += \
./Core/Motors/motors.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Motors/%.o Core/Motors/%.su Core/Motors/%.cyclo: ../Core/Motors/%.c Core/Motors/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Core/Motors -I../Core/MyUtils -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Motors

clean-Core-2f-Motors:
	-$(RM) ./Core/Motors/motors.cyclo ./Core/Motors/motors.d ./Core/Motors/motors.o ./Core/Motors/motors.su

.PHONY: clean-Core-2f-Motors

