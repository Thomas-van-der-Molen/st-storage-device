/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdlib.h>
#include <math.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

COM_InitTypeDef BspCOMInit;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

const uint8_t flash_write_stat_reg    = 0x01;
const uint8_t flash_byte_program      = 0x02;
const uint8_t flash_read              = 0x03;
const uint8_t flash_write_disable     = 0x04;
const uint8_t flash_read_stat_reg     = 0x05;
const uint8_t flash_write_enable      = 0x06;
const uint8_t flash_hs_read           = 0x0b;
const uint8_t flash_4k_s_erase        = 0x20;
const uint8_t flash_en_write_stat_reg = 0x50;
const uint8_t flash_32k_b_erase       = 0x52;
const uint8_t flash_chip_erase        = 0x60;
const uint8_t flash_read_id           = 0x90;
const uint8_t flash_aai_word_program  = 0xad;
const uint8_t flash_64k_b_erase       = 0xd8;

int buf_len;
//uart_tx_buf has to be pretty big to accomodate the main menu text
char uart_tx_buf[200];
uint8_t uart_rx_buf[20];

uint8_t spi_rx_buf[100];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

void get_flash_status();
void write_enable();
void write_disable();
void read_id();
void read_data();
void write_data();
void unlock_blocks();
void print_menu();
void erase4k();
void poll_chip();
uint8_t get_flash_status_register();
void write_aai();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//HAL_UART_Receive_IT(&hcom_uart, rx_buf, 10, 100);
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
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  
  /* USER CODE END 2 */

  /* Initialize COM1 port (115200, 8 bits (7-bit data + 1 stop bit), no parity */
  BspCOMInit.BaudRate   = 115200;
  BspCOMInit.WordLength = COM_WORDLENGTH_8B;
  BspCOMInit.StopBits   = COM_STOPBITS_1;
  BspCOMInit.Parity     = COM_PARITY_NONE;
  BspCOMInit.HwFlowCtl  = COM_HWCONTROL_NONE;
  if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE)
  {
    Error_Handler();
  }

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  //pull hold and write protect high, they are active low
  //pull chip select high just in case
  HAL_GPIO_WritePin(HOLD_GPIO_Port, HOLD_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(write_protect_GPIO_Port, write_protect_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(chip_select_GPIO_Port, chip_select_Pin, GPIO_PIN_SET);

  //write_enable();
  //write_data();
  //unlock_blocks();
  //get_flash_status();
  //read_data();
  //write_disable();
  //read_id();

  //buf_len = sprintf(uart_tx_buf, "%02x \n", spi_rx_buf[0]);
  //HAL_UART_Transmit(&hcom_uart, (uint8_t*)uart_tx_buf, buf_len, 1000);
  
  //print menu
  print_menu();

  while (1)
  {
    
    if (HAL_UART_Receive(&hcom_uart, uart_rx_buf, 2, 1000) == HAL_OK){
      switch (uart_rx_buf[0])
      {
      case '1':
        // print status
        get_flash_status();
        break;
      case '2':
        //read from chip
        read_data();
        break;
      case '3':
        //write to chip
        write_data();
        break;
      case '4':
        //unlock blocks
        unlock_blocks();
        break;
      case '5':
        //enable write
        write_enable();
        break;
      case '6':
        //disable write
        write_disable();
        break;
      case '7':
        //erase 4k
        erase4k();
        break;
      case '8':
        //write aai
        write_aai();
        break;
      default:
        break;
      }

      // print menu
      print_menu();
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, HOLD_Pin|chip_select_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(write_protect_GPIO_Port, write_protect_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : HOLD_Pin chip_select_Pin */
  GPIO_InitStruct.Pin = HOLD_Pin|chip_select_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : write_protect_Pin */
  GPIO_InitStruct.Pin = write_protect_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(write_protect_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void print_menu(){
  //buf_len = sprintf(uart_tx_buf, "--MENU--\n1.status\t2.read\n3.write\t4.unlock blocks\n5.enable write\n6.disable write\n7.erase4k\n");
  buf_len = sprintf(uart_tx_buf, "--MENU--\n%-14s\t2.read\n%-14s\t4.unlock blocks\n%-14s\t6.disable write\n%-14s\t8.write aai\n","1.status","3.write", "5.enable write", "7.erase4k");
  HAL_UART_Transmit(&hcom_uart, (uint8_t*)uart_tx_buf, buf_len, 1000);
}

void unlock_blocks(){
  uint8_t bytes[] = {flash_write_stat_reg, 0x00};

  HAL_GPIO_WritePin(chip_select_GPIO_Port, chip_select_Pin, GPIO_PIN_RESET);
  HAL_StatusTypeDef txstat = HAL_SPI_Transmit(&hspi1, &flash_write_enable, 1, 1000);
  HAL_GPIO_WritePin(chip_select_GPIO_Port, chip_select_Pin, GPIO_PIN_SET);
  
  HAL_GPIO_WritePin(chip_select_GPIO_Port, chip_select_Pin, GPIO_PIN_RESET);
  txstat = HAL_SPI_Transmit(&hspi1, bytes, 2, 1000);
  HAL_GPIO_WritePin(chip_select_GPIO_Port, chip_select_Pin, GPIO_PIN_SET);

  if (txstat != HAL_OK){
    __NOP();
  }
}

void erase4k(){
  uint8_t bytes[] = {flash_4k_s_erase, 0x00, 0x00, 0x00};
  HAL_GPIO_WritePin(chip_select_GPIO_Port, chip_select_Pin, GPIO_PIN_RESET);
  HAL_StatusTypeDef txstat = HAL_SPI_Transmit(&hspi1, bytes, 4, 1000);
  HAL_GPIO_WritePin(chip_select_GPIO_Port, chip_select_Pin, GPIO_PIN_SET);
}

void write_data(){
  uint8_t bytes[] = {flash_byte_program, 0x00, 0x00, 0x00, 0xad};
  HAL_GPIO_WritePin(chip_select_GPIO_Port, chip_select_Pin, GPIO_PIN_RESET);
  HAL_StatusTypeDef txstat = HAL_SPI_Transmit(&hspi1, bytes, 5, 1000);
  HAL_GPIO_WritePin(chip_select_GPIO_Port, chip_select_Pin, GPIO_PIN_SET);
  if (txstat != HAL_OK){
    __NOP();
  }
}

void write_aai(){
  //write aai, 3 bytes starting address, 2 bytes data
  //polling
  //aai, 2 bytes
  //polling
  //uint8_t bytes[] = {flash_aai_word_program, 0x00, 0x00, 0x00, 0xde, 0xad};
  uint8_t data[] = {
    flash_aai_word_program,
    0x00, 0x00, 0x00,
    0xde,
    0xad,
    0xbe,
    0xef,
    0x13,
    0x37,
    0xca,
    0xfe,
    0xb0,
    0xba,
    0xf0,
    0x0d,
  };
  //enable write
  write_enable();

  //send 6 bytes. Then, enter a loop sending three bytes and polling
  HAL_GPIO_WritePin(chip_select_GPIO_Port, chip_select_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, data, 6, 1000);
  HAL_GPIO_WritePin(chip_select_GPIO_Port, chip_select_Pin, GPIO_PIN_SET);

  for(int x=6; x < (sizeof(data)/sizeof(uint8_t)); x+=2){
    //polling first
    poll_chip();
    HAL_GPIO_WritePin(chip_select_GPIO_Port, chip_select_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, (uint8_t[]){flash_aai_word_program, data[x], data[x+1]}, 3, 1000);
    HAL_GPIO_WritePin(chip_select_GPIO_Port, chip_select_Pin, GPIO_PIN_SET);
  }

  //writing fininshed, need to disable write
  write_disable();
}

void poll_chip(){
  uint8_t chip_status;
  uint8_t chip_status_mask = 0x01;
  do{
    chip_status = get_flash_status_register();
  }while(chip_status & chip_status_mask);
}

void read_data(){

  //prompt the user, ask how many bytes they want to read
  buf_len = sprintf(uart_tx_buf, "How many bytes to read?(2 digits):\n");
  HAL_UART_Transmit(&hcom_uart, (uint8_t*)uart_tx_buf, buf_len, 1000);

  //receive 3 bytes, 2 for digits, plus \n from putty
  //need to make a blocking call
  HAL_StatusTypeDef uart_rx_stat;
  do{
    uart_rx_stat = HAL_UART_Receive(&hcom_uart, uart_rx_buf, 3, 1000);
  }while(uart_rx_stat != HAL_OK);

  //extract just the two digits from the received input
  char temp[] = {uart_rx_buf[0], uart_rx_buf[1]};
  float bytes_to_read = atoi(temp);

  uint8_t bytes[] = {flash_read, 0x00, 0x00, 0x00};
  HAL_GPIO_WritePin(chip_select_GPIO_Port, chip_select_Pin, GPIO_PIN_RESET);
  HAL_StatusTypeDef txstat = HAL_SPI_Transmit(&hspi1, bytes, 4, 1000);
  HAL_StatusTypeDef rxstat = HAL_SPI_Receive(&hspi1, spi_rx_buf, (int)bytes_to_read, 1000);
  HAL_GPIO_WritePin(chip_select_GPIO_Port, chip_select_Pin, GPIO_PIN_SET);

  buf_len = sprintf(uart_tx_buf, "Data:\n");
  HAL_UART_Transmit(&hcom_uart, (uint8_t*)uart_tx_buf, buf_len, 1000);

  //transmit the read data via uart
  //print in rows of four bytes. If a row is not full ex: read one byte - fill the rest with 0's
  //needs to be refactored, works, but not the way that you said it would
  for (int x=0; x < ceil(bytes_to_read/4.0); x++){
    buf_len = sprintf(uart_tx_buf, "%02x %02x %02x %02x\n", spi_rx_buf[(4*x)],spi_rx_buf[(4*x)+1],spi_rx_buf[(4*x)+2],spi_rx_buf[(4*x)+3]);
    HAL_UART_Transmit(&hcom_uart, (uint8_t*)uart_tx_buf, buf_len, 1000);
  }
  if (txstat != HAL_OK || rxstat != HAL_OK){
    __NOP();
  }
}

void get_flash_status(){
  //get status from chip
  HAL_GPIO_WritePin(chip_select_GPIO_Port, chip_select_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, &flash_read_stat_reg, 1, 1000);
  HAL_SPI_Receive(&hspi1, spi_rx_buf, 1, 1000);
  HAL_GPIO_WritePin(chip_select_GPIO_Port, chip_select_Pin, GPIO_PIN_SET);

  //transmit status via uart
  buf_len = sprintf(uart_tx_buf, "chip status: %02x\n", spi_rx_buf[0]);
  HAL_UART_Transmit(&hcom_uart, (uint8_t *)uart_tx_buf, buf_len, 1000);
}

uint8_t get_flash_status_register(){
  HAL_GPIO_WritePin(chip_select_GPIO_Port, chip_select_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, &flash_read_stat_reg, 1, 1000);
  HAL_SPI_Receive(&hspi1, spi_rx_buf, 1, 1000);
  HAL_GPIO_WritePin(chip_select_GPIO_Port, chip_select_Pin, GPIO_PIN_SET);
  return spi_rx_buf[0];
}

void write_enable(){
  HAL_GPIO_WritePin(chip_select_GPIO_Port, chip_select_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, &flash_write_enable, 1, 1000);
  HAL_GPIO_WritePin(chip_select_GPIO_Port, chip_select_Pin, GPIO_PIN_SET);
}

void write_disable(){
  HAL_GPIO_WritePin(chip_select_GPIO_Port, chip_select_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, &flash_write_disable, 1, 1000);
  HAL_GPIO_WritePin(chip_select_GPIO_Port, chip_select_Pin, GPIO_PIN_SET);
}

void read_id(){
  uint8_t bytes[] = {flash_read_id, 0x00, 0x00, 0x01};
  HAL_GPIO_WritePin(chip_select_GPIO_Port, chip_select_Pin, GPIO_PIN_RESET);
  HAL_StatusTypeDef txstat = HAL_SPI_Transmit(&hspi1, bytes, 4, 1000);
  HAL_StatusTypeDef rxstat = HAL_SPI_Receive(&hspi1, spi_rx_buf, 2, 1000);
  HAL_GPIO_WritePin(chip_select_GPIO_Port, chip_select_Pin, GPIO_PIN_SET);
  if (txstat != HAL_OK || rxstat != HAL_OK){
    __NOP();
  }
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
