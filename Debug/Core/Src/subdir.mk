################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables
C_SRCS += \
../Core/Src/config.c \
../Core/Src/current_sensor.c \
../Core/Src/encoder.c \
../Core/Src/encoder_hal.c \
../Core/Src/gripper.c \
../Core/Src/hw_io.c \
../Core/Src/input_shaper.c \
../Core/Src/kalman.c \
../Core/Src/kalman_lib.c \
../Core/Src/main.c \
../Core/Src/modbus_bridge.c \
../Core/Src/modbus_frame.c \
../Core/Src/modbus_rtu.c \
../Core/Src/motor_controller.c \
../Core/Src/motor_homing.c \
../Core/Src/pid.c \
../Core/Src/pwm_output.c \
../Core/Src/safety.c \
../Core/Src/scurve.c \
../Core/Src/sequencer.c \
../Core/Src/stm32g4xx_hal_msp.c \
../Core/Src/stm32g4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32g4xx.c \
../Core/Src/telemetry_hub.c \
../Core/Src/trajectory.c

OBJS += \
./Core/Src/config.o \
./Core/Src/current_sensor.o \
./Core/Src/encoder.o \
./Core/Src/encoder_hal.o \
./Core/Src/gripper.o \
./Core/Src/hw_io.o \
./Core/Src/input_shaper.o \
./Core/Src/kalman.o \
./Core/Src/kalman_lib.o \
./Core/Src/main.o \
./Core/Src/modbus_bridge.o \
./Core/Src/modbus_frame.o \
./Core/Src/modbus_rtu.o \
./Core/Src/motor_controller.o \
./Core/Src/motor_homing.o \
./Core/Src/pid.o \
./Core/Src/pwm_output.o \
./Core/Src/safety.o \
./Core/Src/scurve.o \
./Core/Src/sequencer.o \
./Core/Src/stm32g4xx_hal_msp.o \
./Core/Src/stm32g4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32g4xx.o \
./Core/Src/telemetry_hub.o \
./Core/Src/trajectory.o

C_DEPS += \
./Core/Src/config.d \
./Core/Src/current_sensor.d \
./Core/Src/encoder.d \
./Core/Src/encoder_hal.d \
./Core/Src/gripper.d \
./Core/Src/hw_io.d \
./Core/Src/input_shaper.d \
./Core/Src/kalman.d \
./Core/Src/kalman_lib.d \
./Core/Src/main.d \
./Core/Src/modbus_bridge.d \
./Core/Src/modbus_frame.d \
./Core/Src/modbus_rtu.d \
./Core/Src/motor_controller.d \
./Core/Src/motor_homing.d \
./Core/Src/pid.d \
./Core/Src/pwm_output.d \
./Core/Src/safety.d \
./Core/Src/scurve.d \
./Core/Src/sequencer.d \
./Core/Src/stm32g4xx_hal_msp.d \
./Core/Src/stm32g4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32g4xx.d \
./Core/Src/telemetry_hub.d \
./Core/Src/trajectory.d


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G474xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/config.cyclo ./Core/Src/config.d ./Core/Src/config.o ./Core/Src/config.su ./Core/Src/current_sensor.cyclo ./Core/Src/current_sensor.d ./Core/Src/current_sensor.o ./Core/Src/current_sensor.su ./Core/Src/encoder.cyclo ./Core/Src/encoder.d ./Core/Src/encoder.o ./Core/Src/encoder.su ./Core/Src/encoder_hal.cyclo ./Core/Src/encoder_hal.d ./Core/Src/encoder_hal.o ./Core/Src/encoder_hal.su ./Core/Src/gripper.cyclo ./Core/Src/gripper.d ./Core/Src/gripper.o ./Core/Src/gripper.su ./Core/Src/hw_io.cyclo ./Core/Src/hw_io.d ./Core/Src/hw_io.o ./Core/Src/hw_io.su ./Core/Src/input_shaper.cyclo ./Core/Src/input_shaper.d ./Core/Src/input_shaper.o ./Core/Src/input_shaper.su ./Core/Src/kalman.cyclo ./Core/Src/kalman.d ./Core/Src/kalman.o ./Core/Src/kalman.su ./Core/Src/kalman_lib.cyclo ./Core/Src/kalman_lib.d ./Core/Src/kalman_lib.o ./Core/Src/kalman_lib.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/modbus_bridge.cyclo ./Core/Src/modbus_bridge.d ./Core/Src/modbus_bridge.o ./Core/Src/modbus_bridge.su ./Core/Src/modbus_frame.cyclo ./Core/Src/modbus_frame.d ./Core/Src/modbus_frame.o ./Core/Src/modbus_frame.su ./Core/Src/modbus_rtu.cyclo ./Core/Src/modbus_rtu.d ./Core/Src/modbus_rtu.o ./Core/Src/modbus_rtu.su ./Core/Src/motor_controller.cyclo ./Core/Src/motor_controller.d ./Core/Src/motor_controller.o ./Core/Src/motor_controller.su ./Core/Src/motor_homing.cyclo ./Core/Src/motor_homing.d ./Core/Src/motor_homing.o ./Core/Src/motor_homing.su ./Core/Src/pid.cyclo ./Core/Src/pid.d ./Core/Src/pid.o ./Core/Src/pid.su ./Core/Src/pwm_output.cyclo ./Core/Src/pwm_output.d ./Core/Src/pwm_output.o ./Core/Src/pwm_output.su ./Core/Src/safety.cyclo ./Core/Src/safety.d ./Core/Src/safety.o ./Core/Src/safety.su ./Core/Src/scurve.cyclo ./Core/Src/scurve.d ./Core/Src/scurve.o ./Core/Src/scurve.su ./Core/Src/sequencer.cyclo ./Core/Src/sequencer.d ./Core/Src/sequencer.o ./Core/Src/sequencer.su ./Core/Src/stm32g4xx_hal_msp.cyclo ./Core/Src/stm32g4xx_hal_msp.d ./Core/Src/stm32g4xx_hal_msp.o ./Core/Src/stm32g4xx_hal_msp.su ./Core/Src/stm32g4xx_it.cyclo ./Core/Src/stm32g4xx_it.d ./Core/Src/stm32g4xx_it.o ./Core/Src/stm32g4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32g4xx.cyclo ./Core/Src/system_stm32g4xx.d ./Core/Src/system_stm32g4xx.o ./Core/Src/system_stm32g4xx.su ./Core/Src/telemetry_hub.cyclo ./Core/Src/telemetry_hub.d ./Core/Src/telemetry_hub.o ./Core/Src/telemetry_hub.su ./Core/Src/trajectory.cyclo ./Core/Src/trajectory.d ./Core/Src/trajectory.o ./Core/Src/trajectory.su

.PHONY: clean-Core-2f-Src
