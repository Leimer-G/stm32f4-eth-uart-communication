################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../ioLibrary_Driver/Internet/MQTT/MQTTPacket/src/MQTTConnectClient.c \
../ioLibrary_Driver/Internet/MQTT/MQTTPacket/src/MQTTConnectServer.c \
../ioLibrary_Driver/Internet/MQTT/MQTTPacket/src/MQTTDeserializePublish.c \
../ioLibrary_Driver/Internet/MQTT/MQTTPacket/src/MQTTFormat.c \
../ioLibrary_Driver/Internet/MQTT/MQTTPacket/src/MQTTPacket.c \
../ioLibrary_Driver/Internet/MQTT/MQTTPacket/src/MQTTSerializePublish.c \
../ioLibrary_Driver/Internet/MQTT/MQTTPacket/src/MQTTSubscribeClient.c \
../ioLibrary_Driver/Internet/MQTT/MQTTPacket/src/MQTTSubscribeServer.c \
../ioLibrary_Driver/Internet/MQTT/MQTTPacket/src/MQTTUnsubscribeClient.c \
../ioLibrary_Driver/Internet/MQTT/MQTTPacket/src/MQTTUnsubscribeServer.c 

OBJS += \
./ioLibrary_Driver/Internet/MQTT/MQTTPacket/src/MQTTConnectClient.o \
./ioLibrary_Driver/Internet/MQTT/MQTTPacket/src/MQTTConnectServer.o \
./ioLibrary_Driver/Internet/MQTT/MQTTPacket/src/MQTTDeserializePublish.o \
./ioLibrary_Driver/Internet/MQTT/MQTTPacket/src/MQTTFormat.o \
./ioLibrary_Driver/Internet/MQTT/MQTTPacket/src/MQTTPacket.o \
./ioLibrary_Driver/Internet/MQTT/MQTTPacket/src/MQTTSerializePublish.o \
./ioLibrary_Driver/Internet/MQTT/MQTTPacket/src/MQTTSubscribeClient.o \
./ioLibrary_Driver/Internet/MQTT/MQTTPacket/src/MQTTSubscribeServer.o \
./ioLibrary_Driver/Internet/MQTT/MQTTPacket/src/MQTTUnsubscribeClient.o \
./ioLibrary_Driver/Internet/MQTT/MQTTPacket/src/MQTTUnsubscribeServer.o 

C_DEPS += \
./ioLibrary_Driver/Internet/MQTT/MQTTPacket/src/MQTTConnectClient.d \
./ioLibrary_Driver/Internet/MQTT/MQTTPacket/src/MQTTConnectServer.d \
./ioLibrary_Driver/Internet/MQTT/MQTTPacket/src/MQTTDeserializePublish.d \
./ioLibrary_Driver/Internet/MQTT/MQTTPacket/src/MQTTFormat.d \
./ioLibrary_Driver/Internet/MQTT/MQTTPacket/src/MQTTPacket.d \
./ioLibrary_Driver/Internet/MQTT/MQTTPacket/src/MQTTSerializePublish.d \
./ioLibrary_Driver/Internet/MQTT/MQTTPacket/src/MQTTSubscribeClient.d \
./ioLibrary_Driver/Internet/MQTT/MQTTPacket/src/MQTTSubscribeServer.d \
./ioLibrary_Driver/Internet/MQTT/MQTTPacket/src/MQTTUnsubscribeClient.d \
./ioLibrary_Driver/Internet/MQTT/MQTTPacket/src/MQTTUnsubscribeServer.d 


# Each subdirectory must supply rules for building sources it contributes
ioLibrary_Driver/Internet/MQTT/MQTTPacket/src/MQTTConnectClient.o: ../ioLibrary_Driver/Internet/MQTT/MQTTPacket/src/MQTTConnectClient.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32F407xx -c -I"C:/Users/taco_/Desktop/curso/EthSerial/ioLibrary_Driver/Internet/DNS" -I"C:/Users/taco_/Desktop/curso/EthSerial/ioLibrary_Driver/Application/loopback" -I../Drivers/CMSIS/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Core/Inc -I"C:/Users/taco_/Desktop/curso/EthSerial/ioLibrary_Driver/Ethernet" -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I"C:/Users/taco_/Desktop/curso/EthSerial/ioLibrary_Driver/Internet/DHCP" -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I"C:/Users/taco_/Desktop/curso/EthSerial/ioLibrary_Driver/Ethernet/W5500" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"ioLibrary_Driver/Internet/MQTT/MQTTPacket/src/MQTTConnectClient.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
ioLibrary_Driver/Internet/MQTT/MQTTPacket/src/MQTTConnectServer.o: ../ioLibrary_Driver/Internet/MQTT/MQTTPacket/src/MQTTConnectServer.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32F407xx -c -I"C:/Users/taco_/Desktop/curso/EthSerial/ioLibrary_Driver/Internet/DNS" -I"C:/Users/taco_/Desktop/curso/EthSerial/ioLibrary_Driver/Application/loopback" -I../Drivers/CMSIS/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Core/Inc -I"C:/Users/taco_/Desktop/curso/EthSerial/ioLibrary_Driver/Ethernet" -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I"C:/Users/taco_/Desktop/curso/EthSerial/ioLibrary_Driver/Internet/DHCP" -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I"C:/Users/taco_/Desktop/curso/EthSerial/ioLibrary_Driver/Ethernet/W5500" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"ioLibrary_Driver/Internet/MQTT/MQTTPacket/src/MQTTConnectServer.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
ioLibrary_Driver/Internet/MQTT/MQTTPacket/src/MQTTDeserializePublish.o: ../ioLibrary_Driver/Internet/MQTT/MQTTPacket/src/MQTTDeserializePublish.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32F407xx -c -I"C:/Users/taco_/Desktop/curso/EthSerial/ioLibrary_Driver/Internet/DNS" -I"C:/Users/taco_/Desktop/curso/EthSerial/ioLibrary_Driver/Application/loopback" -I../Drivers/CMSIS/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Core/Inc -I"C:/Users/taco_/Desktop/curso/EthSerial/ioLibrary_Driver/Ethernet" -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I"C:/Users/taco_/Desktop/curso/EthSerial/ioLibrary_Driver/Internet/DHCP" -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I"C:/Users/taco_/Desktop/curso/EthSerial/ioLibrary_Driver/Ethernet/W5500" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"ioLibrary_Driver/Internet/MQTT/MQTTPacket/src/MQTTDeserializePublish.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
ioLibrary_Driver/Internet/MQTT/MQTTPacket/src/MQTTFormat.o: ../ioLibrary_Driver/Internet/MQTT/MQTTPacket/src/MQTTFormat.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32F407xx -c -I"C:/Users/taco_/Desktop/curso/EthSerial/ioLibrary_Driver/Internet/DNS" -I"C:/Users/taco_/Desktop/curso/EthSerial/ioLibrary_Driver/Application/loopback" -I../Drivers/CMSIS/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Core/Inc -I"C:/Users/taco_/Desktop/curso/EthSerial/ioLibrary_Driver/Ethernet" -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I"C:/Users/taco_/Desktop/curso/EthSerial/ioLibrary_Driver/Internet/DHCP" -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I"C:/Users/taco_/Desktop/curso/EthSerial/ioLibrary_Driver/Ethernet/W5500" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"ioLibrary_Driver/Internet/MQTT/MQTTPacket/src/MQTTFormat.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
ioLibrary_Driver/Internet/MQTT/MQTTPacket/src/MQTTPacket.o: ../ioLibrary_Driver/Internet/MQTT/MQTTPacket/src/MQTTPacket.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32F407xx -c -I"C:/Users/taco_/Desktop/curso/EthSerial/ioLibrary_Driver/Internet/DNS" -I"C:/Users/taco_/Desktop/curso/EthSerial/ioLibrary_Driver/Application/loopback" -I../Drivers/CMSIS/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Core/Inc -I"C:/Users/taco_/Desktop/curso/EthSerial/ioLibrary_Driver/Ethernet" -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I"C:/Users/taco_/Desktop/curso/EthSerial/ioLibrary_Driver/Internet/DHCP" -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I"C:/Users/taco_/Desktop/curso/EthSerial/ioLibrary_Driver/Ethernet/W5500" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"ioLibrary_Driver/Internet/MQTT/MQTTPacket/src/MQTTPacket.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
ioLibrary_Driver/Internet/MQTT/MQTTPacket/src/MQTTSerializePublish.o: ../ioLibrary_Driver/Internet/MQTT/MQTTPacket/src/MQTTSerializePublish.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32F407xx -c -I"C:/Users/taco_/Desktop/curso/EthSerial/ioLibrary_Driver/Internet/DNS" -I"C:/Users/taco_/Desktop/curso/EthSerial/ioLibrary_Driver/Application/loopback" -I../Drivers/CMSIS/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Core/Inc -I"C:/Users/taco_/Desktop/curso/EthSerial/ioLibrary_Driver/Ethernet" -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I"C:/Users/taco_/Desktop/curso/EthSerial/ioLibrary_Driver/Internet/DHCP" -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I"C:/Users/taco_/Desktop/curso/EthSerial/ioLibrary_Driver/Ethernet/W5500" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"ioLibrary_Driver/Internet/MQTT/MQTTPacket/src/MQTTSerializePublish.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
ioLibrary_Driver/Internet/MQTT/MQTTPacket/src/MQTTSubscribeClient.o: ../ioLibrary_Driver/Internet/MQTT/MQTTPacket/src/MQTTSubscribeClient.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32F407xx -c -I"C:/Users/taco_/Desktop/curso/EthSerial/ioLibrary_Driver/Internet/DNS" -I"C:/Users/taco_/Desktop/curso/EthSerial/ioLibrary_Driver/Application/loopback" -I../Drivers/CMSIS/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Core/Inc -I"C:/Users/taco_/Desktop/curso/EthSerial/ioLibrary_Driver/Ethernet" -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I"C:/Users/taco_/Desktop/curso/EthSerial/ioLibrary_Driver/Internet/DHCP" -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I"C:/Users/taco_/Desktop/curso/EthSerial/ioLibrary_Driver/Ethernet/W5500" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"ioLibrary_Driver/Internet/MQTT/MQTTPacket/src/MQTTSubscribeClient.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
ioLibrary_Driver/Internet/MQTT/MQTTPacket/src/MQTTSubscribeServer.o: ../ioLibrary_Driver/Internet/MQTT/MQTTPacket/src/MQTTSubscribeServer.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32F407xx -c -I"C:/Users/taco_/Desktop/curso/EthSerial/ioLibrary_Driver/Internet/DNS" -I"C:/Users/taco_/Desktop/curso/EthSerial/ioLibrary_Driver/Application/loopback" -I../Drivers/CMSIS/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Core/Inc -I"C:/Users/taco_/Desktop/curso/EthSerial/ioLibrary_Driver/Ethernet" -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I"C:/Users/taco_/Desktop/curso/EthSerial/ioLibrary_Driver/Internet/DHCP" -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I"C:/Users/taco_/Desktop/curso/EthSerial/ioLibrary_Driver/Ethernet/W5500" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"ioLibrary_Driver/Internet/MQTT/MQTTPacket/src/MQTTSubscribeServer.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
ioLibrary_Driver/Internet/MQTT/MQTTPacket/src/MQTTUnsubscribeClient.o: ../ioLibrary_Driver/Internet/MQTT/MQTTPacket/src/MQTTUnsubscribeClient.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32F407xx -c -I"C:/Users/taco_/Desktop/curso/EthSerial/ioLibrary_Driver/Internet/DNS" -I"C:/Users/taco_/Desktop/curso/EthSerial/ioLibrary_Driver/Application/loopback" -I../Drivers/CMSIS/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Core/Inc -I"C:/Users/taco_/Desktop/curso/EthSerial/ioLibrary_Driver/Ethernet" -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I"C:/Users/taco_/Desktop/curso/EthSerial/ioLibrary_Driver/Internet/DHCP" -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I"C:/Users/taco_/Desktop/curso/EthSerial/ioLibrary_Driver/Ethernet/W5500" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"ioLibrary_Driver/Internet/MQTT/MQTTPacket/src/MQTTUnsubscribeClient.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
ioLibrary_Driver/Internet/MQTT/MQTTPacket/src/MQTTUnsubscribeServer.o: ../ioLibrary_Driver/Internet/MQTT/MQTTPacket/src/MQTTUnsubscribeServer.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32F407xx -c -I"C:/Users/taco_/Desktop/curso/EthSerial/ioLibrary_Driver/Internet/DNS" -I"C:/Users/taco_/Desktop/curso/EthSerial/ioLibrary_Driver/Application/loopback" -I../Drivers/CMSIS/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Core/Inc -I"C:/Users/taco_/Desktop/curso/EthSerial/ioLibrary_Driver/Ethernet" -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I"C:/Users/taco_/Desktop/curso/EthSerial/ioLibrary_Driver/Internet/DHCP" -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I"C:/Users/taco_/Desktop/curso/EthSerial/ioLibrary_Driver/Ethernet/W5500" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"ioLibrary_Driver/Internet/MQTT/MQTTPacket/src/MQTTUnsubscribeServer.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

