################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/_systemFiles/syscalls.c \
../Core/Src/_systemFiles/sysmem.c \
../Core/Src/_systemFiles/system_stm32f1xx.c 

OBJS += \
./Core/Src/_systemFiles/syscalls.o \
./Core/Src/_systemFiles/sysmem.o \
./Core/Src/_systemFiles/system_stm32f1xx.o 

C_DEPS += \
./Core/Src/_systemFiles/syscalls.d \
./Core/Src/_systemFiles/sysmem.d \
./Core/Src/_systemFiles/system_stm32f1xx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/_systemFiles/%.o Core/Src/_systemFiles/%.su Core/Src/_systemFiles/%.cyclo: ../Core/Src/_systemFiles/%.c Core/Src/_systemFiles/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-_systemFiles

clean-Core-2f-Src-2f-_systemFiles:
	-$(RM) ./Core/Src/_systemFiles/syscalls.cyclo ./Core/Src/_systemFiles/syscalls.d ./Core/Src/_systemFiles/syscalls.o ./Core/Src/_systemFiles/syscalls.su ./Core/Src/_systemFiles/sysmem.cyclo ./Core/Src/_systemFiles/sysmem.d ./Core/Src/_systemFiles/sysmem.o ./Core/Src/_systemFiles/sysmem.su ./Core/Src/_systemFiles/system_stm32f1xx.cyclo ./Core/Src/_systemFiles/system_stm32f1xx.d ./Core/Src/_systemFiles/system_stm32f1xx.o ./Core/Src/_systemFiles/system_stm32f1xx.su

.PHONY: clean-Core-2f-Src-2f-_systemFiles

