/*
 * Minimal interrupt service routine implementation required by the
 * LoRaWAN_End_Node STM32CubeIDE example.  Drop this file into
 * Core/Src/ to satisfy the linker error for DMA1_Channel5_IRQHandler
 * when the default HAL generated file has been removed.
 */
#include "main.h"
#include "stm32wlxx_it.h"

extern DMA_HandleTypeDef hdma_usart2_tx;

void DMA1_Channel5_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_usart2_tx);
}
