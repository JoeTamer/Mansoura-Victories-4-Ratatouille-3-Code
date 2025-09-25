################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/hardware/hardwareSourceFiles/encoders.c \
../Core/Src/hardware/hardwareSourceFiles/infraredSensors.c \
../Core/Src/hardware/hardwareSourceFiles/motors.c \
../Core/Src/hardware/hardwareSourceFiles/mpu6050.c 

OBJS += \
./Core/Src/hardware/hardwareSourceFiles/encoders.o \
./Core/Src/hardware/hardwareSourceFiles/infraredSensors.o \
./Core/Src/hardware/hardwareSourceFiles/motors.o \
./Core/Src/hardware/hardwareSourceFiles/mpu6050.o 

C_DEPS += \
./Core/Src/hardware/hardwareSourceFiles/encoders.d \
./Core/Src/hardware/hardwareSourceFiles/infraredSensors.d \
./Core/Src/hardware/hardwareSourceFiles/motors.d \
./Core/Src/hardware/hardwareSourceFiles/mpu6050.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/hardware/hardwareSourceFiles/%.o Core/Src/hardware/hardwareSourceFiles/%.su Core/Src/hardware/hardwareSourceFiles/%.cyclo: ../Core/Src/hardware/hardwareSourceFiles/%.c Core/Src/hardware/hardwareSourceFiles/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-hardware-2f-hardwareSourceFiles

clean-Core-2f-Src-2f-hardware-2f-hardwareSourceFiles:
	-$(RM) ./Core/Src/hardware/hardwareSourceFiles/encoders.cyclo ./Core/Src/hardware/hardwareSourceFiles/encoders.d ./Core/Src/hardware/hardwareSourceFiles/encoders.o ./Core/Src/hardware/hardwareSourceFiles/encoders.su ./Core/Src/hardware/hardwareSourceFiles/infraredSensors.cyclo ./Core/Src/hardware/hardwareSourceFiles/infraredSensors.d ./Core/Src/hardware/hardwareSourceFiles/infraredSensors.o ./Core/Src/hardware/hardwareSourceFiles/infraredSensors.su ./Core/Src/hardware/hardwareSourceFiles/motors.cyclo ./Core/Src/hardware/hardwareSourceFiles/motors.d ./Core/Src/hardware/hardwareSourceFiles/motors.o ./Core/Src/hardware/hardwareSourceFiles/motors.su ./Core/Src/hardware/hardwareSourceFiles/mpu6050.cyclo ./Core/Src/hardware/hardwareSourceFiles/mpu6050.d ./Core/Src/hardware/hardwareSourceFiles/mpu6050.o ./Core/Src/hardware/hardwareSourceFiles/mpu6050.su

.PHONY: clean-Core-2f-Src-2f-hardware-2f-hardwareSourceFiles

