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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// CAN message structures
FDCAN_TxHeaderTypeDef TxHeader;
FDCAN_RxHeaderTypeDef RxHeader;
FDCAN_FilterTypeDef sFilterConfig;

uint8_t TxData[8];
uint8_t RxData[8];

// Status variables
uint8_t can_tx_complete = 0;
uint8_t can_rx_complete = 0;
uint32_t can_error_count = 0;


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

uint32_t message_counter = 0;

// CAN Statistics
typedef struct {
    uint32_t total_rx_messages;
    uint32_t total_tx_messages;
    uint32_t total_errors;
    uint32_t bus_off_count;
} CAN_Stats_t;

CAN_Stats_t can_stats = {0};

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

COM_InitTypeDef BspCOMInit;
FDCAN_HandleTypeDef hfdcan1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

// Function prototypes
void CAN_Analyzer_Init(void);
void CAN_Config_Filter(void);
void CAN_Send_Message(uint32_t id, uint8_t *data, uint8_t length);
void CAN_Process_Received_Message(void);
void Print_CAN_Message(const char* direction, uint32_t id, uint8_t *data, uint8_t length);
void Print_CAN_Statistics(void);
const char* Get_CAN_Error_String(uint32_t error_code);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * @brief Initialize CAN Analyzer
 * @details This function sets up the CAN peripheral for analyzer operation:
 *          - Configures message filters to accept all CAN frames
 *          - Enables necessary interrupts for RX, TX, and error handling
 *          - Starts the CAN peripheral
 * @param None
 * @retval None
 */
void CAN_Analyzer_Init(void)
{
    printf("=== STM32 CAN Analyzer Initializing ===\r\n");

    // Configure CAN filters to accept all messages
    CAN_Config_Filter();

    // Start CAN peripheral
    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
    {
        printf("ERROR: Failed to start FDCAN peripheral!\r\n");
        Error_Handler();
    }

    printf("CAN Analyzer Ready!\r\n");
    printf("Configuration:\r\n");
    printf("  - Baud Rate: 500 kbit/s\r\n");
    printf("  - Mode: Classic CAN\r\n");
    printf("  - Filters: Accept All IDs\r\n");
    printf("  - UART Output: 115200 baud\r\n");
    printf("=== Listening for CAN messages ===\r\n\r\n");
}

/**
 * @brief Configure CAN filter to accept all messages
 * @param None
 * @retval None
 */

void CAN_Config_Filter(void)
{
	sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex = 0;
    sFilterConfig.FilterType = FDCAN_FILTER_MASK;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1 = 0x000;      // Filter ID
    sFilterConfig.FilterID2 = 0x000;      // Filter Mask (0x000 = accept all)

    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
    {
    	Error_Handler();
    }

    // Configure filter to accept all extended IDs
    sFilterConfig.IdType = FDCAN_EXTENDED_ID;
    sFilterConfig.FilterIndex = 0;
    sFilterConfig.FilterType = FDCAN_FILTER_MASK;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1 = 0x00000000; // Filter ID
    sFilterConfig.FilterID2 = 0x00000000; // Filter Mask (0x00000000 = accept all)

    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
    {
        Error_Handler();
    }

    // Enable notifications
    if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_TX_COMPLETE, 0) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_BUS_OFF | FDCAN_IT_ERROR_WARNING | FDCAN_IT_ERROR_PASSIVE, 0) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief Send a CAN message
 * @param id: CAN message ID
 * @param data: Pointer to data buffer
 * @param length: Data length (0-8 bytes)
 * @retval None
 */

void CAN_Send_Message(uint32_t id, uint8_t *data, uint8_t length)
{
    // Configure transmission header
    if (id <= 0x7FF) {
        TxHeader.IdType = FDCAN_STANDARD_ID;
        TxHeader.Identifier = id;
    } else {
        TxHeader.IdType = FDCAN_EXTENDED_ID;
        TxHeader.Identifier = id;
    }

    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = length << 16; // Convert to FDCAN format
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;

    // Send the message
    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, data) != HAL_OK)
    {
        can_error_count++;
        printf("CAN TX Error! Error count: %lu\r\n", can_error_count);
    }
    else
    {
        Print_CAN_Message("TX", TxHeader.Identifier, data, length);
    }
}

/**
 * @brief Process received CAN message
 * @param None
 * @retval None
 */
void CAN_Process_Received_Message(void)
{
    if (can_rx_complete)
    {
        can_rx_complete = 0;

        // Extract data length from FDCAN format
        uint8_t data_length = (RxHeader.DataLength >> 16) & 0x0F;

        Print_CAN_Message("RX", RxHeader.Identifier, RxData, data_length);

        // Example: Echo back received message with ID incremented by 1
        // You can modify this behavior based on your analyzer requirements
        if (RxHeader.Identifier < 0x7FE) // Avoid overflow for standard IDs
        {
            CAN_Send_Message(RxHeader.Identifier + 1, RxData, data_length);
        }
    }
}

/**
 * @brief Print CAN message details via UART
 * @param direction: "TX" or "RX"
 * @param id: CAN message ID
 * @param data: Pointer to data buffer
 * @param length: Data length
 * @retval None
 */
void Print_CAN_Message(const char* direction, uint32_t id, uint8_t *data, uint8_t length)
{
    printf("[%s] ID: 0x%03lX, DLC: %d, Data: ", direction, id, length);

    for (uint8_t i = 0; i < length; i++)
    {
        printf("0x%02X ", data[i]);
    }
    printf("\r\n");
}

/**
 * @brief Print CAN bus statistics
 * @details Displays current statistics including message counts and errors.
 * @param None
 * @retval None
 */
void Print_CAN_Statistics(void)
{
    printf("\r\n=== CAN Statistics ===\r\n");
    printf("RX Messages: %lu\r\n", can_stats.total_rx_messages);
    printf("TX Messages: %lu\r\n", can_stats.total_tx_messages);
    printf("Total Errors: %lu\r\n", can_stats.total_errors);
    printf("Bus Off Events: %lu\r\n", can_stats.bus_off_count);
    printf("======================\r\n\r\n");
}

/**
 * @brief Convert error code to readable string
 * @param error_code: HAL error code
 * @retval Pointer to error description string
 */

const char* Get_CAN_Error_String(uint32_t error_code)
{
    switch (error_code)
    {
        case HAL_FDCAN_ERROR_NONE: return "No Error";
        case FDCAN_FLAG_ERROR_WARNING: return "Error Warning";
        case FDCAN_ESI_PASSIVE: return "Error Passive";
        case FDCAN_FLAG_BUS_OFF: return "Bus Off";
        case FDCAN_PROTOCOL_ERROR_STUFF: return "Stuff Error";
        case FDCAN_PROTOCOL_ERROR_FORM: return "Form Error";
        case FDCAN_PROTOCOL_ERROR_ACK: return "ACK Error";
        case FDCAN_PROTOCOL_ERROR_BIT1: return "Bit1 Error";
        case FDCAN_PROTOCOL_ERROR_BIT0: return "Bit0 Error";
        case FDCAN_PROTOCOL_ERROR_CRC: return "CRC Error";
        default: return "Unknown Error";
    }
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
  MX_FDCAN1_Init();
  MX_USART1_UART_Init();
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

  CAN_Analyzer_Init();

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */
	  CAN_Process_Received_Message();

	  static uint32_t led_tick = 0;

	  if (HAL_GetTick() - led_tick >= 500)
	  {
		  led_tick = HAL_GetTick();
	      HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	  }

	  // Small delay to prevent excessive CPU usage
	  HAL_Delay(1);

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
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = ENABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 20;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 13;
  hfdcan1.Init.NominalTimeSeg2 = 3;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.StdFiltersNbr = 0;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
 * @brief CAN RX FIFO 0 callback
 * @details Called automatically when a new message is received in RX FIFO 0.
 *          This function is called by HAL_FDCAN_IRQHandler().
 * @param hfdcan: Pointer to FDCAN handle
 * @param RxFifo0ITs: Interrupt flags
 * @retval None
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
    {
        // Retrieve the received message
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
        {
            can_rx_complete = 1;  // Set flag for main loop processing
        }
    }
}

/**
 * @brief CAN TX Event FIFO callback
 * @details Called when a transmission is completed successfully.
 * @param hfdcan: Pointer to FDCAN handle
 * @param TxEventFifoITs: Interrupt flags
 * @retval None
 */
void HAL_FDCAN_TxEventFifoCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t TxEventFifoITs)
{
    can_tx_complete = 1;  // Set flag indicating transmission complete
}

/**
 * @brief CAN Error callback
 * @details Called when CAN errors occur (bus-off, error warning, etc.).
 *          Provides detailed error information for debugging.
 * @param hfdcan: Pointer to FDCAN handle
 * @retval None
 */
void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef *hfdcan)
{
    can_error_count++;
    can_stats.total_errors++;

    // Get detailed error information
    uint32_t error_code = HAL_FDCAN_GetError(hfdcan);

    printf("CAN Error Detected!\r\n");
    printf("  Error Code: 0x%08lX (%s)\r\n", error_code, Get_CAN_Error_String(error_code));
    printf("  Total Errors: %lu\r\n", can_error_count);

    // Handle specific error conditions
    if (error_code & FDCAN_FLAG_BUS_OFF)
    {
        can_stats.bus_off_count++;
        printf("  Bus-Off condition detected! Attempting restart...\r\n");

        // Attempt to restart CAN after bus-off
        HAL_FDCAN_Stop(hfdcan);
        HAL_Delay(100);  // Wait before restart
        if (HAL_FDCAN_Start(hfdcan) == HAL_OK)
        {
            printf("  CAN restarted successfully.\r\n");
        }
    }
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
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
