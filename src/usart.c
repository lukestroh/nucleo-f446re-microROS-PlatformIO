#include "usart.h"


/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
void MX_USART2_UART_Init(UART_HandleTypeDef* huart2) {

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2->Instance = USART2;
  huart2->Init.BaudRate = 115200;
  huart2->Init.WordLength = UART_WORDLENGTH_8B;
  huart2->Init.StopBits = UART_STOPBITS_1;
  huart2->Init.Parity = UART_PARITY_NONE;
  huart2->Init.Mode = UART_MODE_TX_RX;
  huart2->Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2->Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
void MX_USB_OTG_FS_PCD_Init(PCD_HandleTypeDef* hpcd_USB_OTG_FS)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS->Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS->Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS->Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS->Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS->Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS->Init.Sof_enable = ENABLE;
  hpcd_USB_OTG_FS->Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS->Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS->Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS->Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

}