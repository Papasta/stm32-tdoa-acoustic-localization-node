################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/WIZnet/ioLibrary_Driver/Ethernet/W5500/w5500.c 

OBJS += \
./Middlewares/WIZnet/ioLibrary_Driver/Ethernet/W5500/w5500.o 

C_DEPS += \
./Middlewares/WIZnet/ioLibrary_Driver/Ethernet/W5500/w5500.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/WIZnet/ioLibrary_Driver/Ethernet/W5500/%.o Middlewares/WIZnet/ioLibrary_Driver/Ethernet/W5500/%.su Middlewares/WIZnet/ioLibrary_Driver/Ethernet/W5500/%.cyclo: ../Middlewares/WIZnet/ioLibrary_Driver/Ethernet/W5500/%.c Middlewares/WIZnet/ioLibrary_Driver/Ethernet/W5500/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Diploma/MASTER_MODULE/Middlewares/WIZnet/ioLibrary_Driver/Ethernet" -I"C:/Diploma/MASTER_MODULE/Middlewares/WIZnet/ioLibrary_Driver/Ethernet/W5500" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Middlewares-2f-WIZnet-2f-ioLibrary_Driver-2f-Ethernet-2f-W5500

clean-Middlewares-2f-WIZnet-2f-ioLibrary_Driver-2f-Ethernet-2f-W5500:
	-$(RM) ./Middlewares/WIZnet/ioLibrary_Driver/Ethernet/W5500/w5500.cyclo ./Middlewares/WIZnet/ioLibrary_Driver/Ethernet/W5500/w5500.d ./Middlewares/WIZnet/ioLibrary_Driver/Ethernet/W5500/w5500.o ./Middlewares/WIZnet/ioLibrary_Driver/Ethernet/W5500/w5500.su

.PHONY: clean-Middlewares-2f-WIZnet-2f-ioLibrary_Driver-2f-Ethernet-2f-W5500

