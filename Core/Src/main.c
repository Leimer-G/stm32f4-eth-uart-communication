/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  * @autor          : Leimer Guambaña
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************\
  * Link Driver FT232R: https://www.usb-drivers.org/ft232r-usb-uart-driver.html
  * Link Pin out STM32F4 Shield: https://download.mikroe.com/documents/add-on-boards/click-shields/stm32f4-discovery/stm32f4-discovery-mikrobus-shield-user-manual-v100.pdf
  * Link Example 1: https://github.com/cnoviello/stm32-nucleof4/blob/master/stm32-nucleof4-ethshield1/src/main.c
  * Link Example 2: https://github.com/afiskon/stm32-w5500
  * Link Libreria ioLibrary_Driver:  https://github.com/Leimer-G/ioLibrary_Driver
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdarg.h>
#include "stdio.h"

#include "socket.h"
#include "wizchip_conf.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define PORT_NUMBER   5000
#define SOCKET_NUMBER   0 //0-4
#define RX_UART_BUFFER_RECEPTION     15
#define RX_SOCKET_BUFFER_RECEPTION   15
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t retVal, sockStatus;
uint8_t okReceptionUart = 1;

unsigned char recepcionSocket[RX_SOCKET_BUFFER_RECEPTION+1];
unsigned char recepcionUart[RX_UART_BUFFER_RECEPTION+1];

char buffer[256];
int lenBuffer;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

void UART_Printf(const char *fmt);
void select_CS(void);
void unselect_CS(void);
void read_Buff(uint8_t *buff, uint16_t len);
void write_Buff(uint8_t *buff, uint16_t len);
uint8_t read_Byte(void);
void write_Byte(uint8_t byte);

void param_Connection(void);
void conectar_Socket_TCP(uint8_t socketNum, uint16_t portLocal);
void conectar_Socket_UDP(uint8_t socketNum, uint16_t portLocal);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void param_Connection() {
    UART_Printf("\r\ninit () llamado!\r\n");

    UART_Printf("Registro de devoluciones de llamada del W5500...\r\n");
    reg_wizchip_cs_cbfunc(select_CS, unselect_CS);
    reg_wizchip_spi_cbfunc(read_Byte, write_Byte);
    reg_wizchip_spiburst_cbfunc(read_Buff, write_Buff);

    UART_Printf("Iniciando funcion wizchip_init ()\r\n");
    uint8_t rx_tx_buff_sizes[] = {2, 2, 2, 2, 2, 2, 2, 2};
    wizchip_init(rx_tx_buff_sizes, rx_tx_buff_sizes);

    wiz_NetInfo net_info = { .mac 	= {0x00, 0x08, 0xdc, 0xab, 0xcd, 0xef},	// Mac address
                             .ip 	= {192, 168, 0, 20},					// IP address
                             .sn 	= {255, 255, 255, 0},					// Subnet mask
                             .gw 	= {192, 168, 0, 1},					// Gateway address
    						 .dns 	= {8, 8, 8, 8}};					// DNS


     UART_Printf("Iniciando funcion wizchip_setnetinfo ()\r\n");
     wizchip_setnetinfo(&net_info);
     wizchip_getnetinfo(&net_info);

    sprintf(buffer, "IP:  %d.%d.%d.%d\r\nGW:  %d.%d.%d.%d\r\nNet: %d.%d.%d.%d\r\nDNS: %d.%d.%d.%d\r\n",
            net_info.ip[0], net_info.ip[1], net_info.ip[2], net_info.ip[3],
            net_info.gw[0], net_info.gw[1], net_info.gw[2], net_info.gw[3],
            net_info.sn[0], net_info.sn[1], net_info.sn[2], net_info.sn[3],
    		net_info.dns[0], net_info.dns[1], net_info.dns[2], net_info.dns[3]);
    UART_Printf(buffer);
}

void conectar_Socket_UDP(uint8_t socketNum, uint16_t portLocal){
	/**Es necesario enviar un dato por socket debido:
	 * A que es necesario una IP de destino
	 * Al recibir un mensaje por socket este obtiene la IP y la guarda
	 * Se podria omitir este paso asignando una IP fija para ip Destination
	 * 		ejm ->uint8_t ipDestination[4] = {192.168.0.50}; // IP PC
	 */
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

void conectar_Socket_TCP(uint8_t socketNum, uint16_t portLocal){
	/* Abra el socket 0 como TCP_SOCKET con el puerto 5000*/
	if((retVal = socket(socketNum, Sn_MR_TCP, 5000, 0)) == 0) {
		/* Ponga el Socket en modo ESCUCHAR. Esto significa que estamos creando un servidor TCP. */
		if((retVal = listen(socketNum)) == SOCK_OK) {
			UART_Printf("Esperando conexion con el Socket...\r\n");
			/* Mientras el socket está en modo ESCUCHAR, esperamos una conexión remota */
			while((sockStatus = getSn_SR(socketNum)) == SOCK_LISTEN) HAL_Delay(500);

			okReceptionUart = 1;
			uint16_t len;
			uint8_t sockInterrupt;
			uint8_t ipDestination[4];
			uint16_t portDestination;
			uint32_t revData;
			/* Si la conexión está ESTABLECIDA con peer remoto*/
			while(1) {
				/* If connection is ESTABLISHED with remote peer */
				if((sockStatus = getSn_SR(socketNum)) == SOCK_ESTABLISHED) {
					/* Enviamos mensaje recibido de Uart */
					if(!okReceptionUart){
						okReceptionUart = 1;
						/* Recuperación del número de puerto y la IP del par remoto */
						getsockopt(socketNum, SO_DESTIP, ipDestination);
						getsockopt(socketNum, SO_DESTPORT, (uint8_t*)&portDestination);
						retVal = send(socketNum, recepcionUart, sizeof(recepcionUart));
						if( retVal == (int16_t)sizeof(recepcionUart)){
							sprintf(buffer, "Mensaje Enviando a la IP Remota: %d.%d.%d.%d: %d\r\n",
									ipDestination[0], ipDestination[1], ipDestination[2], ipDestination[3], portDestination);
							UART_Printf(buffer);
						}else{
							sprintf(buffer, "Algo salio mal; Return Value: %d\r\n", retVal);
							UART_Printf(buffer);
						}
					}

					/* Enviamos mensaje recibido de Socket */
					if((sockInterrupt = getSn_IR(socketNum)) == Sn_IR_CON){
						sprintf(buffer,"Conexion con el Socket Exitosa!, Socket Interrupt Value: %d\r\n", sockInterrupt);
						UART_Printf(buffer);
						setSn_IR(socketNum, Sn_IR_CON);
					}else if(sockInterrupt == Sn_IR_RECV || sockInterrupt == (Sn_IR_RECV|Sn_IR_SENDOK) ){
						setSn_IR(socketNum, Sn_IR_RECV);
						if ((len = getSn_RX_RSR(socketNum)) > 0){
							if((revData=recv(0,recepcionSocket, len)) > 0){
								getSn_DIPR(socketNum, ipDestination);
								portDestination = getSn_DPORT(socketNum);
								sprintf(buffer, "Dato recibido: * %s * de la IP Remota: %d.%d.%d.%d: %d\r\n",
										recepcionSocket,
										ipDestination[0], ipDestination[1], ipDestination[2], ipDestination[3], portDestination);
								UART_Printf(buffer);
								for( int i = 0; i < sizeof(recepcionSocket);  ++i )
									recepcionSocket[i] = (char)0;
							}else{
								sprintf(buffer, "Algo salio mal; Receive data : %ld\r\n", revData);
								UART_Printf(buffer);
							}
						}
					}
					HAL_Delay(500);
				}else{
					if(sockStatus == SOCK_CLOSE_WAIT) UART_Printf( "Socket cerrado\r\n");
					else{
						sprintf(buffer, "Algo salio mal; Socket Status: %d\r\n", sockStatus); /* Algo salió mal con el par remoto, tal vez la conexión se cerró inesperadamente */
						UART_Printf(buffer);
					}
					break;
				}
			}
		}else UART_Printf("LISTEN¡ Error!\r\n"); /* Ops: el socket no está en modo ESCUCHAR. Algo salió mal */
	}else{
		sprintf(buffer, "Algo salió mal; Return Value:%d\r\n", retVal); /* No se puede abrir el Socket. Esto significa que algo está mal con la configuración del W5100: ¿tal vez un problema de SPI? */
		UART_Printf(buffer);
	}

	/* Cerramos el socket y volvemos a iniciar una conexión. */
	disconnect(socketNum);
	close(socketNum);

}

void HAL_UART_RxCpltCallback ( UART_HandleTypeDef *huart ){
	if(huart->Instance == USART2)
		okReceptionUart = HAL_UART_Receive_IT(&huart2, recepcionUart, RX_UART_BUFFER_RECEPTION);
	else
		HAL_UART_Receive_IT(&huart2, recepcionUart, RX_UART_BUFFER_RECEPTION);
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart2, recepcionUart, RX_UART_BUFFER_RECEPTION);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  /*Configura parametros iniciales para W5500*/
  param_Connection();

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  /*Descomentar para conexion UDP*/
	  conectar_Socket_TCP(SOCKET_NUMBER, PORT_NUMBER);
	  /*Descomentar para conexion TCP*/
//	  conectar_Socket_UDP(SOCKET_NUMBER, PORT_NUMBER);
	  /*Nota. No descomentar los ambos*/
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ETH_CS_GPIO_Port, ETH_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : ETH_CS_Pin */
  GPIO_InitStruct.Pin = ETH_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ETH_CS_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
//UART_Printf("Texto : %d\r\n", datoAadjuntar);
//void UART_Printf(const char* fmt, ...) {
//      char buff[256];
//      va_list args;
//      va_start(args, fmt);
//      vsnprintf(buff, sizeof(buff), fmt, args);
//      HAL_UART_Transmit(&huart2, (uint8_t*)buff, strlen(buff), HAL_MAX_DELAY);
//      va_end(args);
//}
  void UART_Printf(const char* fmt){
      HAL_UART_Transmit(&huart2, (uint8_t*)fmt, strlen(fmt), HAL_MAX_DELAY);
  }

  void select_CS(void) {
      HAL_GPIO_WritePin(ETH_CS_GPIO_Port, ETH_CS_Pin, GPIO_PIN_RESET);
  }

  void unselect_CS(void) {
      HAL_GPIO_WritePin(ETH_CS_GPIO_Port, ETH_CS_Pin, GPIO_PIN_SET);
  }

  void read_Buff( uint8_t* buff, uint16_t len) {
      HAL_SPI_Receive(&hspi1, buff, len, HAL_MAX_DELAY);
  }

  void write_Buff(uint8_t* buff, uint16_t len) {
      HAL_SPI_Transmit(&hspi1, buff, len, HAL_MAX_DELAY);
  }

  uint8_t read_Byte(void) {
      uint8_t byte;
      read_Buff(&byte, sizeof(byte));
      return byte;
  }

  void write_Byte(uint8_t byte) {
      write_Buff(&byte, sizeof(byte));
  }

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
