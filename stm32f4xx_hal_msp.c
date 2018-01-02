/**
  ******************************************************************************
  * File Name          : stm32f4xx_hal_msp.c
  * Description        : This file provides code for the MSP Initialization 
  *                      and de-Initialization codes.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"


extern DMA_HandleTypeDef udma_tx;
extern DMA_HandleTypeDef udma_rx;


/**
  * Initializes the Global MSP.
  */
void HAL_MspInit(void)
{

  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /* System interrupt init*/
  /* MemoryManagement_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(MemoryManagement_IRQn, 0, 0);
  /* BusFault_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(BusFault_IRQn, 0, 0);
  /* UsageFault_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(UsageFault_IRQn, 0, 0);
  /* SVCall_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SVCall_IRQn, 0, 0);
  /* DebugMonitor_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DebugMonitor_IRQn, 0, 0);
  /* PendSV_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(PendSV_IRQn, 0, 0);
  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);

}

void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{

  /* uart3 = usb, uart6 = gpio */
  GPIO_InitTypeDef GPIO_InitStruct;

  if(huart->Instance==USART6)
  {

    /* GPIOG clock enable */
    __HAL_RCC_GPIOG_CLK_ENABLE();

    /* Peripheral clock enable */
    __HAL_RCC_USART6_CLK_ENABLE();

    /* Enable DMA clock */
    __HAL_RCC_DMA2_CLK_ENABLE();
  
    /**USART6 GPIO Configuration    
    PG14     ------> USART6_TX
    PG9     ------> USART6_RX 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_USART6;

    HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

    /* Configure the DMA handler for Transmission process */
    udma_tx.Instance                 = DMA2_Stream6;
    udma_tx.Init.Channel             = DMA_CHANNEL_5;
    udma_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    udma_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
    udma_tx.Init.MemInc              = DMA_MINC_ENABLE;
    udma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    udma_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    udma_tx.Init.Mode                = DMA_NORMAL;
    udma_tx.Init.Priority            = DMA_PRIORITY_LOW;
    udma_tx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    udma_tx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
    udma_tx.Init.MemBurst            = DMA_MBURST_INC4;
    udma_tx.Init.PeriphBurst         = DMA_PBURST_INC4;

    HAL_DMA_Init(&udma_tx);

    /* Associate the initialized DMA handle to the UART handle */
    __HAL_LINKDMA(huart, hdmatx, udma_tx);

    /* Configure the DMA handler for reception process */
    udma_rx.Instance                 = DMA2_Stream2;
    udma_rx.Init.Channel             = DMA_CHANNEL_5;
    udma_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    udma_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
    udma_rx.Init.MemInc              = DMA_MINC_ENABLE;
    udma_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    udma_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    udma_rx.Init.Mode                = DMA_NORMAL;
    udma_rx.Init.Priority            = DMA_PRIORITY_HIGH;
    udma_rx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    udma_rx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
    udma_rx.Init.MemBurst            = DMA_MBURST_INC4;
    udma_rx.Init.PeriphBurst         = DMA_PBURST_INC4;

    HAL_DMA_Init(&udma_rx);

    /* Associate the initialized DMA handle to the the UART handle */
    __HAL_LINKDMA(huart, hdmarx, udma_rx);

    /* NVIC configuration for DMA transfer complete interrupt (USART6_TX) */
    HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 0, 1);
    HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);

    /* NVIC configuration for DMA transfer complete interrupt (USART6_RX) */
    HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

    /* USART6 interrupt Init */
    HAL_NVIC_SetPriority(USART6_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART6_IRQn);

  }
  else if(huart->Instance==USART3)
  {

    /* GPIOB clock enable */
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* Peripheral clock enable */
    __HAL_RCC_USART3_CLK_ENABLE();

    /* Enable DMA clock */
    __HAL_RCC_DMA1_CLK_ENABLE();

    /**USART3 GPIO Configuration
    PB10     ------> USART3_TX
    PB11     ------> USART3_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;

    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Configure the DMA handler for Transmission process */
    udma_tx.Instance                 = DMA1_Stream3;
    udma_tx.Init.Channel             = DMA_CHANNEL_4;
    udma_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    udma_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
    udma_tx.Init.MemInc              = DMA_MINC_ENABLE;
    udma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    udma_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    udma_tx.Init.Mode                = DMA_NORMAL;
    udma_tx.Init.Priority            = DMA_PRIORITY_LOW;
    udma_tx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    udma_tx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
    udma_tx.Init.MemBurst            = DMA_MBURST_INC4;
    udma_tx.Init.PeriphBurst         = DMA_PBURST_INC4;

    HAL_DMA_Init(&udma_tx);

    /* Associate the initialized DMA handle to the UART handle */
    __HAL_LINKDMA(huart, hdmatx, udma_tx);

    /* Configure the DMA handler for reception process */
    udma_rx.Instance                 = DMA1_Stream1;
    udma_rx.Init.Channel             = DMA_CHANNEL_4;
    udma_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    udma_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
    udma_rx.Init.MemInc              = DMA_MINC_ENABLE;
    udma_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    udma_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    udma_rx.Init.Mode                = DMA_NORMAL;
    udma_rx.Init.Priority            = DMA_PRIORITY_HIGH;
    udma_rx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    udma_rx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
    udma_rx.Init.MemBurst            = DMA_MBURST_INC4;
    udma_rx.Init.PeriphBurst         = DMA_PBURST_INC4;

    HAL_DMA_Init(&udma_rx);

    /* Associate the initialized DMA handle to the the UART handle */
    __HAL_LINKDMA(huart, hdmarx, udma_rx);

    /* NVIC configuration for DMA transfer complete interrupt (USART3_TX) */
    HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 1);
    HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);

    /* NVIC configuration for DMA transfer complete interrupt (USART3_RX) */
    HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);

    /* USART3 interrupt Init */
    HAL_NVIC_SetPriority(USART3_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART3_IRQn);

  }

}

void HAL_UART_MspDeInit(UART_HandleTypeDef* huart)
{

  /* uart3 = usb, uart6 = gpio */

  if(huart->Instance==USART6)
  {

    /* Peripheral clock disable */
    __HAL_RCC_USART6_CLK_DISABLE();
  
    /**USART6 GPIO Configuration    
    PG14     ------> USART6_TX
    PG9     ------> USART6_RX 
    */
    HAL_GPIO_DeInit(GPIOG, GPIO_PIN_14|GPIO_PIN_9);

    /* USART6 DMA DeInit */
    HAL_DMA_DeInit(huart->hdmatx);
    HAL_DMA_DeInit(huart->hdmarx);

    /* Disable the NVIC for DMA */
    HAL_NVIC_DisableIRQ(DMA2_Stream6_IRQn);
    HAL_NVIC_DisableIRQ(DMA2_Stream2_IRQn);

    /* USART6 interrupt DeInit */
    HAL_NVIC_DisableIRQ(USART6_IRQn);

  }
  else if(huart->Instance==USART3)
  {

    /* Peripheral clock disable */
    __HAL_RCC_USART3_CLK_DISABLE();

    /**USART3 GPIO Configuration
    PB10     ------> USART3_TX
    PB11     ------> USART3_RX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10|GPIO_PIN_11);

    /* USART3 DMA DeInit */
    HAL_DMA_DeInit(huart->hdmatx);
    HAL_DMA_DeInit(huart->hdmarx);

    /* Disable the NVIC for DMA */
    HAL_NVIC_DisableIRQ(DMA1_Stream3_IRQn);
    HAL_NVIC_DisableIRQ(DMA1_Stream1_IRQn);

    /* USART3 interrupt DeInit */
    HAL_NVIC_DisableIRQ(USART3_IRQn);
  }
  else
    Error_Handler();

}

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
