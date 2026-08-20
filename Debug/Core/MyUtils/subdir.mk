################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/MyUtils/my_utils.c 

OBJS += \
./Core/MyUtils/my_utils.o 

C_DEPS += \
./Core/MyUtils/my_utils.d 


# Each subdirectory must supply rules for building sources it contributes
Core/MyUtils/%.o Core/MyUtils/%.su Core/MyUtils/%.cyclo: ../Core/MyUtils/%.c Core/MyUtils/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Core/Motors -I../Core/MyUtils -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-MyUtils

clean-Core-2f-MyUtils:
	-$(RM) ./Core/MyUtils/my_utils.cyclo ./Core/MyUtils/my_utils.d ./Core/MyUtils/my_utils.o ./Core/MyUtils/my_utils.su

.PHONY: clean-Core-2f-MyUtils

