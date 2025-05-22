/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    opamp.c
  * @brief   This file provides code for the configuration
  *          of the OPAMP instances.
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
#include "opamp.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

OPAMP_HandleTypeDef hopamp4;
OPAMP_HandleTypeDef hopamp5;

/* OPAMP4 init function */
void MX_OPAMP4_Init(void)
{

  /* USER CODE BEGIN OPAMP4_Init 0 */

  /* USER CODE END OPAMP4_Init 0 */

  /* USER CODE BEGIN OPAMP4_Init 1 */

  /* USER CODE END OPAMP4_Init 1 */
  hopamp4.Instance = OPAMP4;
  hopamp4.Init.PowerMode = OPAMP_POWERMODE_NORMALSPEED;
  hopamp4.Init.Mode = OPAMP_FOLLOWER_MODE;
  hopamp4.Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_DAC;
  hopamp4.Init.InternalOutput = DISABLE;
  hopamp4.Init.TimerControlledMuxmode = OPAMP_TIMERCONTROLLEDMUXMODE_DISABLE;
  hopamp4.Init.UserTrimming = OPAMP_TRIMMING_FACTORY;
  if (HAL_OPAMP_Init(&hopamp4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN OPAMP4_Init 2 */

  /* USER CODE END OPAMP4_Init 2 */

}
/* OPAMP5 init function */
void MX_OPAMP5_Init(void)
{

  /* USER CODE BEGIN OPAMP5_Init 0 */

  /* USER CODE END OPAMP5_Init 0 */

  /* USER CODE BEGIN OPAMP5_Init 1 */

  /* USER CODE END OPAMP5_Init 1 */
  hopamp5.Instance = OPAMP5;
  hopamp5.Init.PowerMode = OPAMP_POWERMODE_HIGHSPEED;
  hopamp5.Init.Mode = OPAMP_FOLLOWER_MODE;
  hopamp5.Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_DAC;
  hopamp5.Init.InternalOutput = DISABLE;
  hopamp5.Init.TimerControlledMuxmode = OPAMP_TIMERCONTROLLEDMUXMODE_DISABLE;
  hopamp5.Init.UserTrimming = OPAMP_TRIMMING_FACTORY;
  if (HAL_OPAMP_Init(&hopamp5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN OPAMP5_Init 2 */

  /* USER CODE END OPAMP5_Init 2 */

}

void HAL_OPAMP_MspInit(OPAMP_HandleTypeDef* opampHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(opampHandle->Instance==OPAMP4)
  {
  /* USER CODE BEGIN OPAMP4_MspInit 0 */

  /* USER CODE END OPAMP4_MspInit 0 */

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**OPAMP4 GPIO Configuration
    PB12     ------> OPAMP4_VOUT
    */
    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN OPAMP4_MspInit 1 */

  /* USER CODE END OPAMP4_MspInit 1 */
  }
  else if(opampHandle->Instance==OPAMP5)
  {
  /* USER CODE BEGIN OPAMP5_MspInit 0 */

  /* USER CODE END OPAMP5_MspInit 0 */

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**OPAMP5 GPIO Configuration
    PA8     ------> OPAMP5_VOUT
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN OPAMP5_MspInit 1 */

  /* USER CODE END OPAMP5_MspInit 1 */
  }
}

void HAL_OPAMP_MspDeInit(OPAMP_HandleTypeDef* opampHandle)
{

  if(opampHandle->Instance==OPAMP4)
  {
  /* USER CODE BEGIN OPAMP4_MspDeInit 0 */

  /* USER CODE END OPAMP4_MspDeInit 0 */

    /**OPAMP4 GPIO Configuration
    PB12     ------> OPAMP4_VOUT
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_12);

  /* USER CODE BEGIN OPAMP4_MspDeInit 1 */

  /* USER CODE END OPAMP4_MspDeInit 1 */
  }
  else if(opampHandle->Instance==OPAMP5)
  {
  /* USER CODE BEGIN OPAMP5_MspDeInit 0 */

  /* USER CODE END OPAMP5_MspDeInit 0 */

    /**OPAMP5 GPIO Configuration
    PA8     ------> OPAMP5_VOUT
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_8);

  /* USER CODE BEGIN OPAMP5_MspDeInit 1 */

  /* USER CODE END OPAMP5_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
