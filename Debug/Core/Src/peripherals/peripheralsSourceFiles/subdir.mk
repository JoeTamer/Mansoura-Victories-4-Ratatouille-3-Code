################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/peripherals/peripheralsSourceFiles/adc.c \
../Core/Src/peripherals/peripheralsSourceFiles/gpio.c \
../Core/Src/peripherals/peripheralsSourceFiles/i2c.c \
../Core/Src/peripherals/peripheralsSourceFiles/interrupts.c \
../Core/Src/peripherals/peripheralsSourceFiles/systick.c \
../Core/Src/peripherals/peripheralsSourceFiles/timers.c \
../Core/Src/peripherals/peripheralsSourceFiles/uart.c 

OBJS += \
./Core/Src/peripherals/peripheralsSourceFiles/adc.o \
./Core/Src/peripherals/peripheralsSourceFiles/gpio.o \
./Core/Src/peripherals/peripheralsSourceFiles/i2c.o \
./Core/Src/peripherals/peripheralsSourceFiles/interrupts.o \
./Core/Src/peripherals/peripheralsSourceFiles/systick.o \
./Core/Src/peripherals/peripheralsSourceFiles/timers.o \
./Core/Src/peripherals/peripheralsSourceFiles/uart.o 

C_DEPS += \
./Core/Src/peripherals/peripheralsSourceFiles/adc.d \
./Core/Src/peripherals/peripheralsSourceFiles/gpio.d \
./Core/Src/peripherals/peripheralsSourceFiles/i2c.d \
./Core/Src/peripherals/peripheralsSourceFiles/interrupts.d \
./Core/Src/peripherals/peripheralsSourceFiles/systick.d \
./Core/Src/peripherals/peripheralsSourceFiles/timers.d \
./Core/Src/peripherals/peripheralsSourceFiles/uart.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/peripherals/peripheralsSourceFiles/%.o Core/Src/peripherals/peripheralsSourceFiles/%.su Core/Src/peripherals/peripheralsSourceFiles/%.cyclo: ../Core/Src/peripherals/peripheralsSourceFiles/%.c Core/Src/peripherals/peripheralsSourceFiles/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-peripherals-2f-peripheralsSourceFiles

clean-Core-2f-Src-2f-peripherals-2f-peripheralsSourceFiles:
	-$(RM) ./Core/Src/peripherals/peripheralsSourceFiles/adc.cyclo ./Core/Src/peripherals/peripheralsSourceFiles/adc.d ./Core/Src/peripherals/peripheralsSourceFiles/adc.o ./Core/Src/peripherals/peripheralsSourceFiles/adc.su ./Core/Src/peripherals/peripheralsSourceFiles/gpio.cyclo ./Core/Src/peripherals/peripheralsSourceFiles/gpio.d ./Core/Src/peripherals/peripheralsSourceFiles/gpio.o ./Core/Src/peripherals/peripheralsSourceFiles/gpio.su ./Core/Src/peripherals/peripheralsSourceFiles/i2c.cyclo ./Core/Src/peripherals/peripheralsSourceFiles/i2c.d ./Core/Src/peripherals/peripheralsSourceFiles/i2c.o ./Core/Src/peripherals/peripheralsSourceFiles/i2c.su ./Core/Src/peripherals/peripheralsSourceFiles/interrupts.cyclo ./Core/Src/peripherals/peripheralsSourceFiles/interrupts.d ./Core/Src/peripherals/peripheralsSourceFiles/interrupts.o ./Core/Src/peripherals/peripheralsSourceFiles/interrupts.su ./Core/Src/peripherals/peripheralsSourceFiles/systick.cyclo ./Core/Src/peripherals/peripheralsSourceFiles/systick.d ./Core/Src/peripherals/peripheralsSourceFiles/systick.o ./Core/Src/peripherals/peripheralsSourceFiles/systick.su ./Core/Src/peripherals/peripheralsSourceFiles/timers.cyclo ./Core/Src/peripherals/peripheralsSourceFiles/timers.d ./Core/Src/peripherals/peripheralsSourceFiles/timers.o ./Core/Src/peripherals/peripheralsSourceFiles/timers.su ./Core/Src/peripherals/peripheralsSourceFiles/uart.cyclo ./Core/Src/peripherals/peripheralsSourceFiles/uart.d ./Core/Src/peripherals/peripheralsSourceFiles/uart.o ./Core/Src/peripherals/peripheralsSourceFiles/uart.su

.PHONY: clean-Core-2f-Src-2f-peripherals-2f-peripheralsSourceFiles

