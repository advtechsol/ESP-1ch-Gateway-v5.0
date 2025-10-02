/*
 * Minimal Radio Board Interface implementation that is sufficient to
 * satisfy the LoRaWAN_End_Node example's call to RBI_GetRFOMaxPowerConfig.
 */
#include "radio_board_if.h"

int32_t RBI_GetRFOMaxPowerConfig(RBI_RFOMaxPowerConfig_TypeDef config)
{
  switch (config)
  {
  case RBI_RFO_LP_MAXPOWER:
    return 15; /* dBm */
  case RBI_RFO_HP_MAXPOWER:
    return 22; /* dBm */
  default:
    return 0;
  }
}
