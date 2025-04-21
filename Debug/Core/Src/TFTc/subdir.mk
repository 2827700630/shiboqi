################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/TFTc/TFT_CAD.c \
../Core/Src/TFTc/TFT_UI.c \
../Core/Src/TFTc/TFT_init.c \
../Core/Src/TFTc/TFT_io.c \
../Core/Src/TFTc/TFT_text.c \
../Core/Src/TFTc/font.c 

OBJS += \
./Core/Src/TFTc/TFT_CAD.o \
./Core/Src/TFTc/TFT_UI.o \
./Core/Src/TFTc/TFT_init.o \
./Core/Src/TFTc/TFT_io.o \
./Core/Src/TFTc/TFT_text.o \
./Core/Src/TFTc/font.o 

C_DEPS += \
./Core/Src/TFTc/TFT_CAD.d \
./Core/Src/TFTc/TFT_UI.d \
./Core/Src/TFTc/TFT_init.d \
./Core/Src/TFTc/TFT_io.d \
./Core/Src/TFTc/TFT_text.d \
./Core/Src/TFTc/font.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/TFTc/%.o Core/Src/TFTc/%.su Core/Src/TFTc/%.cyclo: ../Core/Src/TFTc/%.c Core/Src/TFTc/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-TFTc

clean-Core-2f-Src-2f-TFTc:
	-$(RM) ./Core/Src/TFTc/TFT_CAD.cyclo ./Core/Src/TFTc/TFT_CAD.d ./Core/Src/TFTc/TFT_CAD.o ./Core/Src/TFTc/TFT_CAD.su ./Core/Src/TFTc/TFT_UI.cyclo ./Core/Src/TFTc/TFT_UI.d ./Core/Src/TFTc/TFT_UI.o ./Core/Src/TFTc/TFT_UI.su ./Core/Src/TFTc/TFT_init.cyclo ./Core/Src/TFTc/TFT_init.d ./Core/Src/TFTc/TFT_init.o ./Core/Src/TFTc/TFT_init.su ./Core/Src/TFTc/TFT_io.cyclo ./Core/Src/TFTc/TFT_io.d ./Core/Src/TFTc/TFT_io.o ./Core/Src/TFTc/TFT_io.su ./Core/Src/TFTc/TFT_text.cyclo ./Core/Src/TFTc/TFT_text.d ./Core/Src/TFTc/TFT_text.o ./Core/Src/TFTc/TFT_text.su ./Core/Src/TFTc/font.cyclo ./Core/Src/TFTc/font.d ./Core/Src/TFTc/font.o ./Core/Src/TFTc/font.su

.PHONY: clean-Core-2f-Src-2f-TFTc

