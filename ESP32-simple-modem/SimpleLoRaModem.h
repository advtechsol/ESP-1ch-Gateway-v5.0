#ifndef SIMPLE_LORA_MODEM_H
#define SIMPLE_LORA_MODEM_H

#include <Arduino.h>
#include <SPI.h>

struct LoRaPins {
    int8_t ss;
    int8_t rst;
    int8_t dio0;
    int8_t sck;
    int8_t miso;
    int8_t mosi;
};

class SimpleLoRaModem {
  public:
    explicit SimpleLoRaModem(const LoRaPins &pins);

    bool begin(uint32_t frequencyHz,
               uint8_t spreadingFactor,
               uint16_t bandwidthKHz,
               uint8_t codingRate,
               int8_t txPowerDbm);

    void setFrequency(uint32_t frequencyHz);
    void setSpreadingFactor(uint8_t spreadingFactor);
    void setBandwidth(uint16_t bandwidthKHz);
    void setCodingRate(uint8_t codingRate);
    void setTxPower(int8_t txPowerDbm);

    uint32_t frequency() const { return frequencyHz_; }
    uint8_t spreadingFactor() const { return spreadingFactor_; }
    uint16_t bandwidth() const { return bandwidthKHz_; }
    uint8_t codingRate() const { return codingRate_; }
    int8_t txPower() const { return txPowerDbm_; }
    bool ready() const { return ready_; }

    void dumpStatus(Stream &out);
    void dumpRegisters(Stream &out, uint8_t start = 0x00, uint8_t end = 0x70);

  private:
    static constexpr uint8_t REG_FIFO = 0x00;
    static constexpr uint8_t REG_OP_MODE = 0x01;
    static constexpr uint8_t REG_FRF_MSB = 0x06;
    static constexpr uint8_t REG_FRF_MID = 0x07;
    static constexpr uint8_t REG_FRF_LSB = 0x08;
    static constexpr uint8_t REG_PA_CONFIG = 0x09;
    static constexpr uint8_t REG_LNA = 0x0C;
    static constexpr uint8_t REG_FIFO_ADDR_PTR = 0x0D;
    static constexpr uint8_t REG_FIFO_TX_BASE_ADDR = 0x0E;
    static constexpr uint8_t REG_FIFO_RX_BASE_ADDR = 0x0F;
    static constexpr uint8_t REG_MODEM_CONFIG1 = 0x1D;
    static constexpr uint8_t REG_MODEM_CONFIG2 = 0x1E;
    static constexpr uint8_t REG_SYMB_TIMEOUT_LSB = 0x1F;
    static constexpr uint8_t REG_PREAMBLE_MSB = 0x20;
    static constexpr uint8_t REG_PREAMBLE_LSB = 0x21;
    static constexpr uint8_t REG_PAYLOAD_LENGTH = 0x22;
    static constexpr uint8_t REG_MODEM_CONFIG3 = 0x26;
    static constexpr uint8_t REG_DETECTION_OPTIMIZE = 0x31;
    static constexpr uint8_t REG_DETECTION_THRESHOLD = 0x37;
    static constexpr uint8_t REG_SYNC_WORD = 0x39;
    static constexpr uint8_t REG_DIO_MAPPING_1 = 0x40;
    static constexpr uint8_t REG_VERSION = 0x42;

    static constexpr uint8_t MODE_LONG_RANGE_MODE = 0x80;
    static constexpr uint8_t MODE_SLEEP = 0x00;
    static constexpr uint8_t MODE_STDBY = 0x01;

    LoRaPins pins_{};
    uint32_t frequencyHz_ = 0;
    uint16_t bandwidthKHz_ = 0;
    uint8_t spreadingFactor_ = 0;
    uint8_t codingRate_ = 0;
    int8_t txPowerDbm_ = 0;
    bool ready_ = false;

    void resetChip();
    void applyLowDataRateOpt();
    void standby();
    void sleep();
    void setOpMode(uint8_t mode);
    uint8_t readRegister(uint8_t addr);
    void writeRegister(uint8_t addr, uint8_t value);
};

#endif // SIMPLE_LORA_MODEM_H
