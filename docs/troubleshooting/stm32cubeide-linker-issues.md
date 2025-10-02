# STM32CubeIDE Linker Errors for LoRaWAN End Node

When porting the LoRaWAN End Node example from STM32CubeWL into STM32CubeIDE, you may encounter linker errors similar to the following:

```
./Application/User/Startup/startup_stm32wl55jcix.o:(.isr_vector+0x7c): undefined reference to `DMA1_Channel5_IRQHandler'
./Middlewares/SubGHz_Phy/radio_driver.o: in function `SUBGRF_SetTxParams':
/Users/<user>/STM32Cube/Repository/STM32Cube_FW_WL_V1.3.1/Middlewares/Third_Party/SubGHz_Phy/stm32_radio_driver/radio_driver.c:658: undefined reference to `RBI_GetRFOMaxPowerConfig'
```

## Root cause

Both symbols belong to optional board-support layers that are not pulled in automatically when you start from a clean project template:

* `DMA1_Channel5_IRQHandler` must be provided by the user application in `stm32wlxx_it.c` so that the DMA interrupt calls through to the HAL.
* `RBI_GetRFOMaxPowerConfig` is part of the Radio Board Interface (RBI). If you do not include a board-specific `radio_board_if.c`, the radio middleware expects you to implement the call-out.

## Fix

1. **DMA interrupt handler** – add the missing handler to your interrupt file. A minimal implementation that forwards to the HAL looks like this:

   ```c
   void DMA1_Channel5_IRQHandler(void)
   {
       HAL_DMA_IRQHandler(&hdma_usart2_tx);
   }
   ```

   Adjust the handle (`hdma_usart2_tx` in the stock LoRaWAN End Node example) to match the peripheral that uses DMA1 Channel 5 in your project. A drop-in version of the function can be found in [`docs/troubleshooting/examples/stm32cubeide/stm32wlxx_it.c`](examples/stm32cubeide/stm32wlxx_it.c).

2. **Radio Board Interface hook** – extend your board support file (for example, `radio_board_if.c`) with:

   ```c
   int32_t RBI_GetRFOMaxPowerConfig(RBI_RFOMaxPowerConfig_TypeDef config)
   {
       switch (config)
       {
       case RBI_RFO_LP_MAXPOWER:
           return 15; /* dBm for the low-power PA on the Nucleo reference design */
       case RBI_RFO_HP_MAXPOWER:
           return 22; /* dBm for the high-power PA on the Nucleo reference design */
       default:
           return 0;
       }
   }
   ```

   Return the correct limits for your hardware variant. The constants above match the STM32WL Nucleo reference design. A ready-made helper is available in [`docs/troubleshooting/examples/stm32cubeide/radio_board_if.c`](examples/stm32cubeide/radio_board_if.c).

After adding these definitions, rebuild the project. The linker should resolve both symbols, allowing the firmware image to be produced successfully.
