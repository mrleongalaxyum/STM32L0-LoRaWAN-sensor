################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/BMP280/bmp280.c 

OBJS += \
./Core/Src/BMP280/bmp280.o 

C_DEPS += \
./Core/Src/BMP280/bmp280.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/BMP280/bmp280.o: ../Core/Src/BMP280/bmp280.c Core/Src/BMP280/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32L073xx -DDEBUG -DCFG_eu868 -DCFG_sx1276_radio -c -I../Core/Inc -I../Drivers/STM32L0xx_HAL_Driver/Inc -I../Drivers/STM32L0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L0xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/BMP280/bmp280.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

