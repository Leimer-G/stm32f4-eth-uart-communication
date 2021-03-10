################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../ioLibrary_Driver/Internet/FTPClient/ftpc.c 

OBJS += \
./ioLibrary_Driver/Internet/FTPClient/ftpc.o 

C_DEPS += \
./ioLibrary_Driver/Internet/FTPClient/ftpc.d 


# Each subdirectory must supply rules for building sources it contributes
ioLibrary_Driver/Internet/FTPClient/ftpc.o: ../ioLibrary_Driver/Internet/FTPClient/ftpc.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32F407xx -c -I"C:/Users/taco_/Desktop/curso/EthSerial/ioLibrary_Driver/Internet/DNS" -I"C:/Users/taco_/Desktop/curso/EthSerial/ioLibrary_Driver/Application/loopback" -I../Drivers/CMSIS/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Core/Inc -I"C:/Users/taco_/Desktop/curso/EthSerial/ioLibrary_Driver/Ethernet" -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I"C:/Users/taco_/Desktop/curso/EthSerial/ioLibrary_Driver/Internet/DHCP" -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I"C:/Users/taco_/Desktop/curso/EthSerial/ioLibrary_Driver/Ethernet/W5500" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"ioLibrary_Driver/Internet/FTPClient/ftpc.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

