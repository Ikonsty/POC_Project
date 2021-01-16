################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/crc.c \
../Core/Src/dma.c \
../Core/Src/gpio.c \
../Core/Src/i2c.c \
../Core/Src/i2s.c \
../Core/Src/main.c \
../Core/Src/spi.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/system_stm32f4xx.c 

OBJS += \
./Core/Src/crc.o \
./Core/Src/dma.o \
./Core/Src/gpio.o \
./Core/Src/i2c.o \
./Core/Src/i2s.o \
./Core/Src/main.o \
./Core/Src/spi.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/system_stm32f4xx.o 

C_DEPS += \
./Core/Src/crc.d \
./Core/Src/dma.d \
./Core/Src/gpio.d \
./Core/Src/i2c.d \
./Core/Src/i2s.d \
./Core/Src/main.d \
./Core/Src/spi.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/system_stm32f4xx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o: ../Core/Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -DUSE_HAL_DRIVER -DARM_MATH_CM4 -D__FPU_PRESENT -DSTM32F411xE -I"/home/ilya/UCU/Grade2/POC/labs/STM32_Tune/Core/Inc" -I"/home/ilya/UCU/Grade2/POC/labs/STM32_Tune/PDM2PCM/App" -I"/home/ilya/UCU/Grade2/POC/labs/STM32_Tune/USB_DEVICE/App" -I"/home/ilya/UCU/Grade2/POC/labs/STM32_Tune/USB_DEVICE/Target" -I"/home/ilya/UCU/Grade2/POC/labs/STM32_Tune/Drivers/STM32F4xx_HAL_Driver/Inc" -I"/home/ilya/UCU/Grade2/POC/labs/STM32_Tune/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"/home/ilya/UCU/Grade2/POC/labs/STM32_Tune/Middlewares/ST/STM32_Audio/Addons/PDM/Inc" -I"/home/ilya/UCU/Grade2/POC/labs/STM32_Tune/Middlewares/ST/STM32_USB_Device_Library/Core/Inc" -I"/home/ilya/UCU/Grade2/POC/labs/STM32_Tune/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc" -I"/home/ilya/UCU/Grade2/POC/labs/STM32_Tune/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"/home/ilya/UCU/Grade2/POC/labs/STM32_Tune/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


