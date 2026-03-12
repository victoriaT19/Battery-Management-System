################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Analog_libs/src/bms_balancing.c \
../Core/Analog_libs/src/bms_generic.c \
../Core/Analog_libs/src/bms_safety.c \
../Core/Analog_libs/src/comm.c \
../Core/Analog_libs/src/mcu_wrapper.c 

OBJS += \
./Core/Analog_libs/src/bms_balancing.o \
./Core/Analog_libs/src/bms_generic.o \
./Core/Analog_libs/src/bms_safety.o \
./Core/Analog_libs/src/comm.o \
./Core/Analog_libs/src/mcu_wrapper.o 

C_DEPS += \
./Core/Analog_libs/src/bms_balancing.d \
./Core/Analog_libs/src/bms_generic.d \
./Core/Analog_libs/src/bms_safety.d \
./Core/Analog_libs/src/comm.d \
./Core/Analog_libs/src/mcu_wrapper.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Analog_libs/src/%.o Core/Analog_libs/src/%.su Core/Analog_libs/src/%.cyclo: ../Core/Analog_libs/src/%.c Core/Analog_libs/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_NUCLEO_64 -DUSE_HAL_DRIVER -DSTM32G474xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/BSP/STM32G4xx_Nucleo -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Analog_libs-2f-src

clean-Core-2f-Analog_libs-2f-src:
	-$(RM) ./Core/Analog_libs/src/bms_balancing.cyclo ./Core/Analog_libs/src/bms_balancing.d ./Core/Analog_libs/src/bms_balancing.o ./Core/Analog_libs/src/bms_balancing.su ./Core/Analog_libs/src/bms_generic.cyclo ./Core/Analog_libs/src/bms_generic.d ./Core/Analog_libs/src/bms_generic.o ./Core/Analog_libs/src/bms_generic.su ./Core/Analog_libs/src/bms_safety.cyclo ./Core/Analog_libs/src/bms_safety.d ./Core/Analog_libs/src/bms_safety.o ./Core/Analog_libs/src/bms_safety.su ./Core/Analog_libs/src/comm.cyclo ./Core/Analog_libs/src/comm.d ./Core/Analog_libs/src/comm.o ./Core/Analog_libs/src/comm.su ./Core/Analog_libs/src/mcu_wrapper.cyclo ./Core/Analog_libs/src/mcu_wrapper.d ./Core/Analog_libs/src/mcu_wrapper.o ./Core/Analog_libs/src/mcu_wrapper.su

.PHONY: clean-Core-2f-Analog_libs-2f-src

