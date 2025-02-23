################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../User_functions/Encoder.c \
../User_functions/format_out.c \
../User_functions/lcd.c 

OBJS += \
./User_functions/Encoder.o \
./User_functions/format_out.o \
./User_functions/lcd.o 

C_DEPS += \
./User_functions/Encoder.d \
./User_functions/format_out.d \
./User_functions/lcd.d 


# Each subdirectory must supply rules for building sources it contributes
User_functions/%.o User_functions/%.su User_functions/%.cyclo: ../User_functions/%.c User_functions/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"D:/Narodstream/1/Disco_F407_Link_2211_blue/User_functions" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-User_functions

clean-User_functions:
	-$(RM) ./User_functions/Encoder.cyclo ./User_functions/Encoder.d ./User_functions/Encoder.o ./User_functions/Encoder.su ./User_functions/format_out.cyclo ./User_functions/format_out.d ./User_functions/format_out.o ./User_functions/format_out.su ./User_functions/lcd.cyclo ./User_functions/lcd.d ./User_functions/lcd.o ./User_functions/lcd.su

.PHONY: clean-User_functions

