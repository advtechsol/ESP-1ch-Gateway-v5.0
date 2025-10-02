#include "SimpleLoRaModem.h"

namespace {
constexpr uint32_t RADIO_XTAL_HZ = 32000000UL;

uint8_t bandwidthCode(uint16_t bandwidthKHz) {
    switch (bandwidthKHz) {
        case 125: return 0x70; // BW 125 kHz
        case 250: return 0x80; // BW 250 kHz
        case 500: return 0x90; // BW 500 kHz
        default: return 0xFF;
    }
}

} // namespace

SimpleLoRaModem::SimpleLoRaModem(const LoRaPins &pins) : pins_(pins) {}

bool SimpleLoRaModem::begin(uint32_t frequencyHz,
                            uint8_t spreadingFactor,
                            uint16_t bandwidthKHz,
                            uint8_t codingRate,
                            int8_t txPowerDbm) {
    ready_ = false;

    pinMode(pins_.ss, OUTPUT);
    digitalWrite(pins_.ss, HIGH);

    if (pins_.rst >= 0) {
        pinMode(pins_.rst, OUTPUT);
    }
    if (pins_.dio0 >= 0) {
        pinMode(pins_.dio0, INPUT);
    }

    SPI.begin(pins_.sck, pins_.miso, pins_.mosi, pins_.ss);

    resetChip();

    sleep();
    writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);
    standby();

    if (readRegister(REG_VERSION) != 0x12) {
        return false;
    }

    writeRegister(REG_FIFO_TX_BASE_ADDR, 0x80);
    writeRegister(REG_FIFO_RX_BASE_ADDR, 0x00);
    writeRegister(REG_LNA, readRegister(REG_LNA) | 0x03);
    writeRegister(REG_MODEM_CONFIG3, 0x04); // LNA gain set by the AGC loop

    setFrequency(frequencyHz);
    setBandwidth(bandwidthKHz);
    setCodingRate(codingRate);
    setSpreadingFactor(spreadingFactor);
    setTxPower(txPowerDbm);

    writeRegister(REG_SYNC_WORD, 0x34); // LoRaWAN public network sync word
    writeRegister(REG_PREAMBLE_MSB, 0x00);
    writeRegister(REG_PREAMBLE_LSB, 0x08);
    writeRegister(REG_PAYLOAD_LENGTH, 0x00);
    writeRegister(REG_SYMB_TIMEOUT_LSB, 0x64);

    ready_ = true;
    return true;
}

void SimpleLoRaModem::setFrequency(uint32_t frequencyHz) {
    standby();
    frequencyHz_ = frequencyHz;

    uint64_t frf = ((uint64_t)frequencyHz << 19) / RADIO_XTAL_HZ;
    writeRegister(REG_FRF_MSB, (uint8_t)(frf >> 16));
    writeRegister(REG_FRF_MID, (uint8_t)(frf >> 8));
    writeRegister(REG_FRF_LSB, (uint8_t)(frf));
}

void SimpleLoRaModem::setSpreadingFactor(uint8_t spreadingFactor) {
    standby();
    spreadingFactor_ = constrain(spreadingFactor, (uint8_t)6, (uint8_t)12);

    if (spreadingFactor_ == 6) {
        writeRegister(REG_DETECTION_OPTIMIZE, 0xC5);
        writeRegister(REG_DETECTION_THRESHOLD, 0x0C);
    } else {
        writeRegister(REG_DETECTION_OPTIMIZE, 0xC3);
        writeRegister(REG_DETECTION_THRESHOLD, 0x0A);
    }

    uint8_t config2 = readRegister(REG_MODEM_CONFIG2);
    config2 = (config2 & 0x0F) | ((spreadingFactor_ << 4) & 0xF0);
    writeRegister(REG_MODEM_CONFIG2, config2);

    applyLowDataRateOpt();
}

void SimpleLoRaModem::setBandwidth(uint16_t bandwidthKHz) {
    uint8_t code = bandwidthCode(bandwidthKHz);
    if (code == 0xFF) {
        return;
    }

    standby();
    bandwidthKHz_ = bandwidthKHz;
    uint8_t config1 = readRegister(REG_MODEM_CONFIG1);
    config1 = (config1 & 0x0F) | code;
    writeRegister(REG_MODEM_CONFIG1, config1);

    applyLowDataRateOpt();
}

void SimpleLoRaModem::setCodingRate(uint8_t codingRate) {
    standby();
    codingRate_ = constrain(codingRate, (uint8_t)5, (uint8_t)8);
    uint8_t cr = (codingRate_ - 4) & 0x07;
    uint8_t config1 = readRegister(REG_MODEM_CONFIG1);
    config1 = (config1 & 0xF1) | (cr << 1);
    writeRegister(REG_MODEM_CONFIG1, config1);
}

void SimpleLoRaModem::setTxPower(int8_t txPowerDbm) {
    standby();
    txPowerDbm_ = constrain(txPowerDbm, (int8_t)2, (int8_t)17);
    writeRegister(REG_PA_CONFIG, 0x80 | (txPowerDbm_ - 2));
}

void SimpleLoRaModem::dumpStatus(Stream &out) {
    out.println(F("LoRa modem status:"));
    out.print(F("  Ready: "));
    out.println(ready_ ? F("yes") : F("no"));
    out.print(F("  Frequency: "));
    out.print(static_cast<double>(frequencyHz_) / 1e6, 3);
    out.println(F(" MHz"));
    out.print(F("  Spreading factor: SF"));
    out.println(spreadingFactor_);
    out.print(F("  Bandwidth: "));
    out.print(bandwidthKHz_);
    out.println(F(" kHz"));
    out.print(F("  Coding rate: 4/"));
    out.println(codingRate_);
    out.print(F("  TX power: "));
    out.print(txPowerDbm_);
    out.println(F(" dBm"));
}

void SimpleLoRaModem::dumpRegisters(Stream &out, uint8_t start, uint8_t end) {
    if (start > end) {
        uint8_t tmp = start;
        start = end;
        end = tmp;
    }

    for (uint8_t addr = start; addr <= end; ++addr) {
        if ((addr - start) % 8 == 0) {
            out.println();
            out.print(F("0x"));
            if (addr < 16) {
                out.print('0');
            }
            out.print(addr, HEX);
            out.print(F(": "));
        }
        uint8_t value = readRegister(addr);
        if (value < 16) {
            out.print('0');
        }
        out.print(value, HEX);
        out.print(' ');
    }
    out.println();
}

void SimpleLoRaModem::resetChip() {
    if (pins_.rst < 0) {
        return;
    }

    digitalWrite(pins_.rst, LOW);
    delay(10);
    digitalWrite(pins_.rst, HIGH);
    delay(10);
}

void SimpleLoRaModem::applyLowDataRateOpt() {
    uint8_t config3 = readRegister(REG_MODEM_CONFIG3);
    if (spreadingFactor_ >= 11 && bandwidthKHz_ <= 125) {
        config3 |= 0x08;
    } else {
        config3 &= ~0x08;
    }
    writeRegister(REG_MODEM_CONFIG3, config3 | 0x04); // ensure AGC is enabled
}

void SimpleLoRaModem::standby() {
    setOpMode(MODE_STDBY);
}

void SimpleLoRaModem::sleep() {
    setOpMode(MODE_SLEEP);
}

void SimpleLoRaModem::setOpMode(uint8_t mode) {
    writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | mode);
}

uint8_t SimpleLoRaModem::readRegister(uint8_t addr) {
    SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
    digitalWrite(pins_.ss, LOW);
    SPI.transfer(addr & 0x7F);
    uint8_t value = SPI.transfer(0x00);
    digitalWrite(pins_.ss, HIGH);
    SPI.endTransaction();
    return value;
}

void SimpleLoRaModem::writeRegister(uint8_t addr, uint8_t value) {
    SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
    digitalWrite(pins_.ss, LOW);
    SPI.transfer(addr | 0x80);
    SPI.transfer(value);
    digitalWrite(pins_.ss, HIGH);
    SPI.endTransaction();
}
