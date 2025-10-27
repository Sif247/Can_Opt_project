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

FDCAN_HandleTypeDef hfdcan1; // CAN1
FDCAN_HandleTypeDef hfdcan2; // CAN2

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_FDCAN2_Init(void);

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_FDCAN1_Init();
    MX_FDCAN2_Init();

    HAL_FDCAN_Start(&hfdcan1);
    HAL_FDCAN_Start(&hfdcan2);

    // Attiva notifiche RX su entrambe le CAN
    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

    // Header e dati CAN1
    FDCAN_TxHeaderTypeDef txHeader1;
    uint8_t txData1[8] = {0x11,0x22,0,0,0,0,0,0};
    txHeader1.Identifier = 0x100;
    txHeader1.IdType = FDCAN_STANDARD_ID;
    txHeader1.TxFrameType = FDCAN_DATA_FRAME;
    txHeader1.DataLength = FDCAN_DLC_BYTES_8;
    txHeader1.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    txHeader1.BitRateSwitch = FDCAN_BRS_OFF;
    txHeader1.FDFormat = FDCAN_CLASSIC_CAN;
    txHeader1.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    txHeader1.MessageMarker = 0;

    // Header e dati CAN2
    FDCAN_TxHeaderTypeDef txHeader2;
    uint8_t txData2[8] = {0xAA,0xBB,0,0,0,0,0,0};
    txHeader2.Identifier = 0x200;
    txHeader2.IdType = FDCAN_STANDARD_ID;
    txHeader2.TxFrameType = FDCAN_DATA_FRAME;
    txHeader2.DataLength = FDCAN_DLC_BYTES_8;
    txHeader2.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    txHeader2.BitRateSwitch = FDCAN_BRS_OFF;
    txHeader2.FDFormat = FDCAN_CLASSIC_CAN;
    txHeader2.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    txHeader2.MessageMarker = 0;

    while(1)
    {
        // Trasmissione su CAN1 e CAN2
        HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&txHeader1,txData1);
        HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2,&txHeader2,txData2);

        // Lampeggia LED come attività
        HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_5);
        HAL_Delay(1000);

        // Cambia leggermente i dati per esempio
        txData1[0]++;
        txData2[0]++;
    }
}

// --------------------------------------------------------------------------
// Callback ricezione per entrambe le CAN
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan,uint32_t RxFifo0ITs)
{
    FDCAN_RxHeaderTypeDef rxHeader;
    uint8_t rxData[8];

    HAL_FDCAN_GetRxMessage(hfdcan,FDCAN_RX_FIFO0,&rxHeader,rxData);

    if(hfdcan->Instance==FDCAN1)
    {
        // Messaggio ricevuto da CAN1
        // Esempio: accendi LED se primo byte = 0x55
        if(rxData[0]==0x55) HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_SET);
    }

    if(hfdcan->Instance==FDCAN2)
    {
        // Messaggio ricevuto da CAN2
        if(rxData[0]==0xAA) HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_SET);
    }
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
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
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Led_GPIO_Port, Led_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Led_Pin */
  GPIO_InitStruct.Pin = Led_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Led_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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