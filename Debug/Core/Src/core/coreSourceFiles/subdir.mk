################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/core/coreSourceFiles/aStar.c \
../Core/Src/core/coreSourceFiles/floodfill.c \
../Core/Src/core/coreSourceFiles/movement.c 

OBJS += \
./Core/Src/core/coreSourceFiles/aStar.o \
./Core/Src/core/coreSourceFiles/floodfill.o \
./Core/Src/core/coreSourceFiles/movement.o 

C_DEPS += \
./Core/Src/core/coreSourceFiles/aStar.d \
./Core/Src/core/coreSourceFiles/floodfill.d \
./Core/Src/core/coreSourceFiles/movement.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/core/coreSourceFiles/%.o Core/Src/core/coreSourceFiles/%.su Core/Src/core/coreSourceFiles/%.cyclo: ../Core/Src/core/coreSourceFiles/%.c Core/Src/core/coreSourceFiles/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-core-2f-coreSourceFiles

clean-Core-2f-Src-2f-core-2f-coreSourceFiles:
	-$(RM) ./Core/Src/core/coreSourceFiles/aStar.cyclo ./Core/Src/core/coreSourceFiles/aStar.d ./Core/Src/core/coreSourceFiles/aStar.o ./Core/Src/core/coreSourceFiles/aStar.su ./Core/Src/core/coreSourceFiles/floodfill.cyclo ./Core/Src/core/coreSourceFiles/floodfill.d ./Core/Src/core/coreSourceFiles/floodfill.o ./Core/Src/core/coreSourceFiles/floodfill.su ./Core/Src/core/coreSourceFiles/movement.cyclo ./Core/Src/core/coreSourceFiles/movement.d ./Core/Src/core/coreSourceFiles/movement.o ./Core/Src/core/coreSourceFiles/movement.su

.PHONY: clean-Core-2f-Src-2f-core-2f-coreSourceFiles

