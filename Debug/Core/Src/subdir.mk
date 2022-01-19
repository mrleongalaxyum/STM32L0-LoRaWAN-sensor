################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/aes.c \
../Core/Src/debug.c \
../Core/Src/hal.c \
../Core/Src/i2c_hal.c \
../Core/Src/lmic.c \
../Core/Src/main.c \
../Core/Src/oslmic.c \
../Core/Src/radio.c \
../Core/Src/sdp800.c \
../Core/Src/stm32l0xx_hal_msp.c \
../Core/Src/stm32l0xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system.c \
../Core/Src/system_stm32l0xx.c 

OBJS += \
./Core/Src/aes.o \
./Core/Src/debug.o \
./Core/Src/hal.o \
./Core/Src/i2c_hal.o \
./Core/Src/lmic.o \
./Core/Src/main.o \
./Core/Src/oslmic.o \
./Core/Src/radio.o \
./Core/Src/sdp800.o \
./Core/Src/stm32l0xx_hal_msp.o \
./Core/Src/stm32l0xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system.o \
./Core/Src/system_stm32l0xx.o 

C_DEPS += \
./Core/Src/aes.d \
./Core/Src/debug.d \
./Core/Src/hal.d \
./Core/Src/i2c_hal.d \
./Core/Src/lmic.d \
./Core/Src/main.d \
./Core/Src/oslmic.d \
./Core/Src/radio.d \
./Core/Src/sdp800.d \
./Core/Src/stm32l0xx_hal_msp.d \
./Core/Src/stm32l0xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system.d \
./Core/Src/system_stm32l0xx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32L073xx -DDEBUG -DCFG_eu868 -DCFG_sx1276_radio -c -I../Core/Inc -I../Drivers/STM32L0xx_HAL_Driver/Inc -I../Drivers/STM32L0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L0xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

