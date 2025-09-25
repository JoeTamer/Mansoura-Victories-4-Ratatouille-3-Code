################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/peripherals/clkSourceFiles/clk.c 

OBJS += \
./Core/Src/peripherals/clkSourceFiles/clk.o 

C_DEPS += \
./Core/Src/peripherals/clkSourceFiles/clk.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/peripherals/clkSourceFiles/%.o Core/Src/peripherals/clkSourceFiles/%.su Core/Src/peripherals/clkSourceFiles/%.cyclo: ../Core/Src/peripherals/clkSourceFiles/%.c Core/Src/peripherals/clkSourceFiles/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-peripherals-2f-clkSourceFiles

clean-Core-2f-Src-2f-peripherals-2f-clkSourceFiles:
	-$(RM) ./Core/Src/peripherals/clkSourceFiles/clk.cyclo ./Core/Src/peripherals/clkSourceFiles/clk.d ./Core/Src/peripherals/clkSourceFiles/clk.o ./Core/Src/peripherals/clkSourceFiles/clk.su

.PHONY: clean-Core-2f-Src-2f-peripherals-2f-clkSourceFiles

