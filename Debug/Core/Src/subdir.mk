################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/current_sensor.c \
../Core/Src/hw_io.c \
../Core/Src/kalman.c \
../Core/Src/main.c \
../Core/Src/modbus_bridge.c \
../Core/Src/modbus_rtu.c \
../Core/Src/motor_controller.c \
../Core/Src/stm32g4xx_hal_msp.c \
../Core/Src/stm32g4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32g4xx.c \
../Core/Src/telemetry_hub.c 

OBJS += \
./Core/Src/current_sensor.o \
./Core/Src/hw_io.o \
./Core/Src/kalman.o \
./Core/Src/main.o \
./Core/Src/modbus_bridge.o \
./Core/Src/modbus_rtu.o \
./Core/Src/motor_controller.o \
./Core/Src/stm32g4xx_hal_msp.o \
./Core/Src/stm32g4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32g4xx.o \
./Core/Src/telemetry_hub.o 

C_DEPS += \
./Core/Src/current_sensor.d \
./Core/Src/hw_io.d \
./Core/Src/kalman.d \
./Core/Src/main.d \
./Core/Src/modbus_bridge.d \
./Core/Src/modbus_rtu.d \
./Core/Src/motor_controller.d \
./Core/Src/stm32g4xx_hal_msp.d \
./Core/Src/stm32g4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32g4xx.d \
./Core/Src/telemetry_hub.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G474xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/current_sensor.cyclo ./Core/Src/current_sensor.d ./Core/Src/current_sensor.o ./Core/Src/current_sensor.su ./Core/Src/hw_io.cyclo ./Core/Src/hw_io.d ./Core/Src/hw_io.o ./Core/Src/hw_io.su ./Core/Src/kalman.cyclo ./Core/Src/kalman.d ./Core/Src/kalman.o ./Core/Src/kalman.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/modbus_bridge.cyclo ./Core/Src/modbus_bridge.d ./Core/Src/modbus_bridge.o ./Core/Src/modbus_bridge.su ./Core/Src/modbus_rtu.cyclo ./Core/Src/modbus_rtu.d ./Core/Src/modbus_rtu.o ./Core/Src/modbus_rtu.su ./Core/Src/motor_controller.cyclo ./Core/Src/motor_controller.d ./Core/Src/motor_controller.o ./Core/Src/motor_controller.su ./Core/Src/stm32g4xx_hal_msp.cyclo ./Core/Src/stm32g4xx_hal_msp.d ./Core/Src/stm32g4xx_hal_msp.o ./Core/Src/stm32g4xx_hal_msp.su ./Core/Src/stm32g4xx_it.cyclo ./Core/Src/stm32g4xx_it.d ./Core/Src/stm32g4xx_it.o ./Core/Src/stm32g4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32g4xx.cyclo ./Core/Src/system_stm32g4xx.d ./Core/Src/system_stm32g4xx.o ./Core/Src/system_stm32g4xx.su ./Core/Src/telemetry_hub.cyclo ./Core/Src/telemetry_hub.d ./Core/Src/telemetry_hub.o ./Core/Src/telemetry_hub.su

.PHONY: clean-Core-2f-Src

