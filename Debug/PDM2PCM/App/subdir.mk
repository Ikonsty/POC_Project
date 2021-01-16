################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../PDM2PCM/App/pdm2pcm.c 

OBJS += \
./PDM2PCM/App/pdm2pcm.o 

C_DEPS += \
./PDM2PCM/App/pdm2pcm.d 


# Each subdirectory must supply rules for building sources it contributes
PDM2PCM/App/%.o: ../PDM2PCM/App/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -DUSE_HAL_DRIVER -DARM_MATH_CM4 -D__FPU_PRESENT -DSTM32F411xE -I"/home/ilya/UCU/Grade2/POC/labs/STM32_Tune/Core/Inc" -I"/home/ilya/UCU/Grade2/POC/labs/STM32_Tune/PDM2PCM/App" -I"/home/ilya/UCU/Grade2/POC/labs/STM32_Tune/USB_DEVICE/App" -I"/home/ilya/UCU/Grade2/POC/labs/STM32_Tune/USB_DEVICE/Target" -I"/home/ilya/UCU/Grade2/POC/labs/STM32_Tune/Drivers/STM32F4xx_HAL_Driver/Inc" -I"/home/ilya/UCU/Grade2/POC/labs/STM32_Tune/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"/home/ilya/UCU/Grade2/POC/labs/STM32_Tune/Middlewares/ST/STM32_Audio/Addons/PDM/Inc" -I"/home/ilya/UCU/Grade2/POC/labs/STM32_Tune/Middlewares/ST/STM32_USB_Device_Library/Core/Inc" -I"/home/ilya/UCU/Grade2/POC/labs/STM32_Tune/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc" -I"/home/ilya/UCU/Grade2/POC/labs/STM32_Tune/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"/home/ilya/UCU/Grade2/POC/labs/STM32_Tune/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


