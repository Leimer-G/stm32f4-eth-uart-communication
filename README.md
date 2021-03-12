# stm32f4-eth-uart-communication
## Objetivo 
Implementar 2 interfaces de Comunicación: 
- Una conexión UDP o TCP (Ethernet) desde una PC al STM32F407.
- Una conexión UART desde USB-UART de STM32F4 DISCOVERY SHIELD hacia PC.<br/>

Integraremos [ioLibrary_Driver](https://github.com/Leimer-G/ioLibrary_Driver) en el Entorno de desarrollo integrado para STM32 -> STM32Cube IDE.
### Esquema 
<image src="https://user-images.githubusercontent.com/74838411/110870112-d4661880-8299-11eb-8f5a-a1a436cfd3ed.PNG" width=50%>

## Hadware utilizado

- Placa de entrenamiento [STM32F4 DISCOVERY](https://www.st.com/resource/en/user_manual/dm00039084-discovery-kit-with-stm32f407vg-mcu-stmicroelectronics.pdf)

<image src="https://user-images.githubusercontent.com/74838411/110871350-40498080-829c-11eb-8ffa-ae58799b5899.PNG" width= 200>
	
- [STM32F4 DISCOVERY SHIEL](https://download.mikroe.com/documents/add-on-boards/click-shields/stm32f4-discovery/stm32f4-discovery-mikrobus-shield-user-manual-v100.pdf)

<image src="https://user-images.githubusercontent.com/74838411/110641034-8230d480-817f-11eb-8f35-020663f27b80.png">
	
- [Modulo ETH Wiz click](https://download.mikroe.com/documents/add-on-boards/click/eth-wiz/eth-wiz-click-manual-v100.pdf)

<image src="https://user-images.githubusercontent.com/74838411/110870222-06777a80-829a-11eb-82ae-f1fbf1b3215a.png" width= 200> <br/>

Es posible que USB-UART de STM32F4 DISCOVERY SHIELD requiera la instalacion de drives, los puede encontrar [aqui](https://www.usb-drivers.org/ft232r-usb-uart-driver.html)

## Software utilizado
- Terminal Serial [Tera Term](https://ttssh2.osdn.jp/index.html.en) es un útil terminal de puerto serie.
- [Hercules SETUP](https://www.hw-group.com/software/hercules-setup-utility) es un útil terminal de puerto serie (terminal RS-485 o RS-232), terminal UDP/IP y terminal TCP/IP Client Server. 
## Creando proyecto CubeMx

### Modulo ETH Wiz 5500
ETH Wiz 5500 trabaja con una comunicación SPI<br/>
Activamos el Periferico de comunicacion SPI. <br/>
Para la STM32F4 DISCOVERY SHIELD usar MicroBus 3.

- Activamos el modo Full-Duplex.
- La velocidad menor a 80MHz.
- Re-asignamos pines SPI1 para compatibilidad de MicroBus 3: PB3, PB4, PB5.
- PE5 como salida para CS/SPI1, asignar label ETH_CS.

<div align="center">
  <image src="https://user-images.githubusercontent.com/74838411/110643368-05ebc080-8182-11eb-8812-278266628ae5.JPG" width=45%>
</div>
  
### UART Integrado STM32F4 DISCOVERY SHIELD
Activamos el Periferico de comunicacion UART.
- Activamos el modo Asynchronous
- Velocidad 9600
- En NVIC Settings activamos las interrupciones (UARTX global interrup)
<div align="center">
  <image src="https://user-images.githubusercontent.com/74838411/110643369-071ced80-8182-11eb-9a48-51cd5460ac38.JPG" width=50%>
</div>

Generamos el codigo desde nuestra herramienta STM32 CubeMx y estaria listo la configuracion de perifericos.

## Integrar ioLibrary_Driver en STM32 Cube IDE
  - Descargamos la Carpeta ioLibrary_Driver de https://github.com/Leimer-G/ioLibrary_Driver y lo copiamos en la carpeta de nuestro proyecto<br/>
  <div align="center"> <image src="https://user-images.githubusercontent.com/74838411/110650768-d3919180-8188-11eb-9982-1509b72379cf.PNG" width=40%> </div>
  
  - En STM32 Cube IDE seleccionamos nuestro proyecto / click derecho/ Propiedades >> C/C++ General/ Path and Symbols/ Includes/ Add<br/>
   
  <div align="center"> <image src="https://user-images.githubusercontent.com/74838411/110650321-71389100-8188-11eb-9694-7bd564597660.PNG" width=40%> </div>
  
  - En File System agragamos los recursos que vamos a utilizar en este caso las carpetas:
    - ioLibrary_Driver/Application/loopback
    - ioLibrary_Driver/Ethernet/W5500
    - ioLibrary_Driver/Ethernet<br/>
   <div align="center"> 
    <image src="https://user-images.githubusercontent.com/74838411/110650325-71389100-8188-11eb-9c16-84cc93032c0b.PNG" width=40%> 
    <image src="https://user-images.githubusercontent.com/74838411/110650327-71d12780-8188-11eb-8a85-5af16d40add9.PNG" width=40%> 
   </div>
       
  - Nos dirigimos a Source Location
    - En Add Folder agreagamos nuestra carpeta ioLibrary_Driver
    - Si no se encuentra la ubicaremos manualmente con Link Folder/ Seleccionamos la casilla Link to folder in the file system/ la buscamos con browse<br/>
   <div align="center"> 
    <image src="https://user-images.githubusercontent.com/74838411/110650330-71d12780-8188-11eb-8a61-ae941c624ada.PNG" width=40%>
    <image src="https://user-images.githubusercontent.com/74838411/110655180-d2626380-818c-11eb-82c0-be695b2aecd1.PNG" width=40%>
  </div>
 
 Con esto ya integramos la libreria ioLibrary_Driver en nuestro proyecto STM32 Cube IDE
 
## Descripcion del ejemplo
  - En el archivo wizchip_conf.h definimos el chip con el que estamos trabajando en la linea 75
  ```C++
  #define _WIZCHIP_                      W5500   // W5100, W5100S, W5200, W5300, W5500
  ```
  - Incluimos las librerias necesarias en nuestro main.c
  ```C++
    #include <string.h>
    #include "stdio.h"
    #include "socket.h"
    #include "wizchip_conf.h"
 ```
  - Funciones necesarias para la comunicacion SPI y UART
    - En las funciones UART_Printf modificar valor &huart2 por su UART_HandleTypeDef
    - En las funciones write_Buff, read_Buff modificar valor &hspi1 por su SPI_HandleTypeDef
 ```C++
  void UART_Printf(const char* fmt){
      HAL_UART_Transmit(&huart2, (uint8_t*)fmt, strlen(fmt), HAL_MAX_DELAY);
  }
  void select_CS(void) { /*Seleccion Cs para Spi*/
      HAL_GPIO_WritePin(ETH_CS_GPIO_Port, ETH_CS_Pin, GPIO_PIN_RESET);
  }

  void unselect_CS(void) { /*Desseleccion Cs para Spi*/
      HAL_GPIO_WritePin(ETH_CS_GPIO_Port, ETH_CS_Pin, GPIO_PIN_SET);
  }
  void read_Buff( uint8_t* buff, uint16_t len) { /*Lee Buffer SPi*/
      HAL_SPI_Receive(&hspi1, buff, len, HAL_MAX_DELAY);
  }
  void write_Buff(uint8_t* buff, uint16_t len) { /*Escribe Buffer SPi*/
      HAL_SPI_Transmit(&hspi1, buff, len, HAL_MAX_DELAY);
  }
  uint8_t read_Byte(void) {  /*Lee Byte SPi*/
      uint8_t byte;
      read_Buff(&byte, sizeof(byte));
      return byte;
  }
  void write_Byte(uint8_t byte) { /*Escribe Byte SPi*/
      write_Buff(&byte, sizeof(byte));
  }
  ```
  
  #### Configuracion WIZCHIP 
  
  - Registramos la función de devolución de llamada para seleccionar y desseleccionar WIZCHIP.
 ```C++
 reg_wizchip_cs_cbfunc(select_CS, unselect_CS);
 ```
  - Registramos la función de devolución de llamada para la interfaz SPI.
 ```C++
  reg_wizchip_spi_cbfunc(read_Byte, write_Byte);
  reg_wizchip_spiburst_cbfunc(read_Buff, write_Buff);
 ```
  - Inicializamos la funcion wizchip_init con un tamaño de buffer
 ```C++
  uint8_t rx_tx_buff_sizes[] = {2, 2, 2, 2, 2, 2, 2, 2};
  wizchip_init(rx_tx_buff_sizes, rx_tx_buff_sizes);
 ```
  - Creamos la variable wiz_NetInfo Informacion para WIZCHIP y la cargamos
 ```C++
 /*La mac es una aleatoria para pruebas*/
  wiz_NetInfo net_info = { .mac 	= {0x00, 0x08, 0xdc, 0xab, 0xcd, 0xef},	// Mac address
                           .ip 	= {192, 168, 0, 20},	// IP address
                           .sn 	= {255, 255, 255, 0},	// Subnet mask
                           .gw 	= {192, 168, 0, 1}	// Gateway address
                         };
                           
  wizchip_setnetinfo(&net_info);
 ```
 - Descripcion de funciones 
 ```C++
 /*uint8_t sn -> numero del socket para el caso del chip W5500 va de 0~4*/
 int8_t socket(uint8_t sn, uint8_t protocol, uint16_t port, uint8_t flag)/*Crea el socket ejemplo-> socket(0, Sn_MR_TCP, 5000, 0))-> Abra el socket 0 como TCP_SOCKET con el   puerto 5000 */
 int8_t listen(uint8_t sn) /* Ponga el Socket en modo ESCUCHAR. Esto significa que estamos creando un servidor TCP. */
 int8_t  getsockopt(uint8_t sn, sockopt_type sotype, void* arg); /*Obtenga la opción de socket como FLAG, TTL, MSS, etc. Consulte @ref sockopt_type*/
 uint16_t getSn_RX_RSR(uint8_t sn) /*Retorna el registro de tamaño de datos recibidos (R)*/
 getSn_DIPR(sn, dipr) /*Configura o indica la dirección IP de destino del Socket n. Es válido cuando se utiliza Socket n en modo TCP / UDP.*/
 getSn_DPORT(sn) /*Configura o indica el número de puerto de destino del Socket n. Es válido cuando se utiliza Socket n en modo TCP / UDP.*/
 getSn_SR(uint8_t sn) /*Retorna el registro de estado del socket (R) , ejemplo -> Return : SOCK_LISTEN ->Modo Escucha, SOCK_ESTABLISHED -> Conexion Establecida, */ 
 getSn_IR(uint8_t sn) /*Retorna el registro de interrupción de socket (R) ejem -> Evento cuando recibio datos -> Return: Sn_IR_RECV*/
 disconnec(uint8_t sn) /*Desconectar socket de conexión.*/
 close(uint8_t sn) /*Cerrar socket.*/
 ```
 - TCP funciones para envio y recepcion
 ```C++
 int32_t send(uint8_t sn, uint8_t * buf, uint16_t len); /*Envias datos al par conectado. Solo modo cliente o servidor TCP.*/
 int32_t recv(uint8_t sn, uint8_t * buf, uint16_t len) /*Reciba datos del par conectado. Solo modo cliente o servidor TCP.*/
 ```
 - UDP funciones para envio y recepcion
 ```C++
 int32_t sendto(uint8_t sn, uint8_t * buf, uint16_t len, uint8_t * addr, uint16_t port); /*Envía un datagrama de UDP o MACRAW*/
 int32_t recvfrom(uint8_t sn, uint8_t * buf, uint16_t len, uint8_t * addr, uint16_t *port) /*Reciba datagrama de UDP o MACRAW*/
 ```
 - Funcion para conexion UDP
 ```C++
 void conectar_Socket_UDP(uint8_t socketNum, uint16_t portLocal){
 /*Abra el socket 0 como TCP_SOCKET con el   puerto 5000*/
	if((retVal = socket(socketNum, Sn_MR_UDP, portLocal, 0)) == 0){
		uint8_t ipDestination[4];
		uint16_t portDestination;
		uint16_t len;
		uint8_t sockInterrupt;
		uint32_t revDatagram;
		okReceptionUart = 1;
		while(1){
			/*Comprobamos si el estatus del socket esta en UPD*/
			if((sockStatus = getSn_SR(socketNum)) == SOCK_UDP) {
				/*Comprueba el servicio de interrupcion y verifica si existen datos entrantes*/
				if((sockInterrupt=getSn_IR(socketNum)) == Sn_IR_RECV){//verifica si llego datos verificar con o  Sn_IR values en w5500.h
					setSn_IR(socketNum, Sn_IR_RECV); /*Set el servicio de interruciones con la recepcion de datos*/
					/*Verificamos la longitud de buffer recibido*/
					if((len=getSn_RX_RSR(socketNum)) > 0){
						/*Recibimos los datos*/
						if((revDatagram=recvfrom(socketNum, recepcionSocket, len, ipDestination, &portDestination)) > 0){
							sprintf(buffer, "Dato recibido: * %s * de la IP Remota: %d.%d.%d.%d: %d\r\n",
									recepcionSocket,
									ipDestination[0], ipDestination[1], ipDestination[2], ipDestination[3], portDestination);
							UART_Printf(buffer);
							for( int i = 0; i < sizeof(recepcionSocket);  ++i )
								recepcionSocket[i] = (char)0;
						}else {
							sprintf(buffer, "Algo salio mal; Receive datagram : %ld\r\n", revDatagram);
							UART_Printf(buffer);
						}
					}
				}
        /*Compruebo si exiten datos recibidos por UART (Interrupcion)*/
				if (!okReceptionUart) {
					okReceptionUart = 1;
					/*Enviamos datos enviados de Uart por socket UDP*/
					retVal = sendto(socketNum, recepcionUart, sizeof(recepcionUart), ipDestination, portDestination);
					/*Verificamos envio correcto*/
					if (retVal == (int16_t) sizeof(recepcionUart)){
						sprintf(buffer, "Mensaje Enviando a la IP Remota: %d.%d.%d.%d: %d\r\n",
								ipDestination[0], ipDestination[1], ipDestination[2], ipDestination[3], portDestination);
						UART_Printf(buffer);
					}else{
						sprintf(buffer, "Algo salio mal; Return Value: %d\r\n", retVal);
						UART_Printf(buffer);
					}
				}
			}else if(sockStatus == SOCK_CLOSED) break;
			HAL_Delay(100);
		}
	}
}
 ```
 - Funcion de interrupcion UART
 ```C++
 void HAL_UART_RxCpltCallback ( UART_HandleTypeDef *huart ){
	if(huart->Instance == USART2)
		okReceptionUart = HAL_UART_Receive_IT(&huart2, recepcionUart, RX_UART_BUFFER_RECEPTION);
	else
		HAL_UART_Receive_IT(&huart2, recepcionUart, RX_UART_BUFFER_RECEPTION);
}
 ```
## Resultados

![ezgif com-gif-maker](https://user-images.githubusercontent.com/74838411/110888340-f66f9300-82b9-11eb-9be1-ea49f4bf7c1a.gif)

