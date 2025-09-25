################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/control/controlSourceFiles/kalmanFilter.c \
../Core/Src/control/controlSourceFiles/pidController.c 

OBJS += \
./Core/Src/control/controlSourceFiles/kalmanFilter.o \
./Core/Src/control/controlSourceFiles/pidController.o 

C_DEPS += \
./Core/Src/control/controlSourceFiles/kalmanFilter.d \
./Core/Src/control/controlSourceFiles/pidController.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/control/controlSourceFiles/%.o Core/Src/control/controlSourceFiles/%.su Core/Src/control/controlSourceFiles/%.cyclo: ../Core/Src/control/controlSourceFiles/%.c Core/Src/control/controlSourceFiles/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-control-2f-controlSourceFiles

clean-Core-2f-Src-2f-control-2f-controlSourceFiles:
	-$(RM) ./Core/Src/control/controlSourceFiles/kalmanFilter.cyclo ./Core/Src/control/controlSourceFiles/kalmanFilter.d ./Core/Src/control/controlSourceFiles/kalmanFilter.o ./Core/Src/control/controlSourceFiles/kalmanFilter.su ./Core/Src/control/controlSourceFiles/pidController.cyclo ./Core/Src/control/controlSourceFiles/pidController.d ./Core/Src/control/controlSourceFiles/pidController.o ./Core/Src/control/controlSourceFiles/pidController.su

.PHONY: clean-Core-2f-Src-2f-control-2f-controlSourceFiles

