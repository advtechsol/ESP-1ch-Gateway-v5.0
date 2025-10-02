#include <Arduino.h>

#include "SimpleLoRaModem.h"

// Update the pin mapping to match your ESP32 + LoRa module wiring.
constexpr LoRaPins LORA_PINS = {
    .ss = 18,
    .rst = 14,
    .dio0 = 26,
    .sck = 5,
    .miso = 19,
    .mosi = 27,
};

constexpr uint32_t DEFAULT_FREQUENCY_HZ = 868100000UL;
constexpr uint8_t DEFAULT_SPREADING_FACTOR = 7;  // SF7
constexpr uint16_t DEFAULT_BANDWIDTH_KHZ = 125;  // 125 kHz
constexpr uint8_t DEFAULT_CODING_RATE = 5;       // 4/5
constexpr int8_t DEFAULT_TX_POWER_DBM = 14;      // 14 dBm

SimpleLoRaModem modem(LORA_PINS);

String readCommand();
void handleCommand(const String &command);
void printHelp();
void printStatus();
void applyDefaults(bool force = false);

void setup() {
    Serial.begin(115200);
    while (!Serial) {
        delay(10);
    }

    Serial.println(F("Simple ESP32 LoRa modem control console"));
    Serial.println(F("----------------------------------------"));

    applyDefaults();
    printHelp();
}

void loop() {
    String command = readCommand();
    if (command.length() > 0) {
        handleCommand(command);
    }
}

String readCommand() {
    if (!Serial.available()) {
        return String();
    }

    String line = Serial.readStringUntil('\n');
    line.trim();
    return line;
}

void handleCommand(const String &command) {
    if (command.isEmpty()) {
        return;
    }

    int spaceIndex = command.indexOf(' ');
    String verb = (spaceIndex == -1) ? command : command.substring(0, spaceIndex);
    String arg = (spaceIndex == -1) ? String() : command.substring(spaceIndex + 1);
    verb.toLowerCase();
    arg.trim();

    if (verb == F("help")) {
        printHelp();
        return;
    }

    if (verb == F("status")) {
        printStatus();
        return;
    }

    if (verb == F("freq")) {
        double value = arg.toDouble();
        if (value <= 0.0) {
            Serial.println(F("Usage: freq <MHz or Hz>"));
            return;
        }
        uint32_t hz = value < 1000.0 ? static_cast<uint32_t>(value * 1e6) : static_cast<uint32_t>(value);
        modem.setFrequency(hz);
        double displayMHz = (value < 1000.0) ? value : hz / 1e6.0;
        Serial.print(F("Frequency set to "));
        Serial.print(displayMHz, 3);
        Serial.println(F(" MHz"));
        return;
    }

    if (verb == F("sf")) {
        int sf = arg.toInt();
        if (sf < 6 || sf > 12) {
            Serial.println(F("Usage: sf <6-12>"));
            return;
        }
        modem.setSpreadingFactor(static_cast<uint8_t>(sf));
        Serial.print(F("Spreading factor set to SF"));
        Serial.println(sf);
        return;
    }

    if (verb == F("bw")) {
        int bw = arg.toInt();
        if (bw != 125 && bw != 250 && bw != 500) {
            Serial.println(F("Usage: bw <125|250|500>"));
            return;
        }
        modem.setBandwidth(static_cast<uint16_t>(bw));
        Serial.print(F("Bandwidth set to "));
        Serial.print(bw);
        Serial.println(F(" kHz"));
        return;
    }

    if (verb == F("cr")) {
        int cr = arg.toInt();
        if (cr < 5 || cr > 8) {
            Serial.println(F("Usage: cr <5-8> (meaning coding rate 4/x)"));
            return;
        }
        modem.setCodingRate(static_cast<uint8_t>(cr));
        Serial.print(F("Coding rate set to 4/"));
        Serial.println(cr);
        return;
    }

    if (verb == F("txp")) {
        int level = arg.toInt();
        if (level < 2 || level > 17) {
            Serial.println(F("Usage: txp <2-17> (dBm)"));
            return;
        }
        modem.setTxPower(static_cast<int8_t>(level));
        Serial.print(F("TX power set to "));
        Serial.print(level);
        Serial.println(F(" dBm"));
        return;
    }

    if (verb == F("dump")) {
        int start = 0;
        int end = 0x70;
        if (!arg.isEmpty()) {
            int comma = arg.indexOf(' ');
            if (comma == -1) {
                start = strtol(arg.c_str(), nullptr, 0);
                end = start + 0x10;
            } else {
                String startStr = arg.substring(0, comma);
                String endStr = arg.substring(comma + 1);
                start = strtol(startStr.c_str(), nullptr, 0);
                end = strtol(endStr.c_str(), nullptr, 0);
            }
        }
        start = constrain(start, 0, 0x7F);
        end = constrain(end, 0, 0x7F);
        modem.dumpRegisters(Serial, static_cast<uint8_t>(start), static_cast<uint8_t>(end));
        return;
    }

    if (verb == F("reset")) {
        applyDefaults(true);
        Serial.println(F("Modem reinitialised with default parameters."));
        return;
    }

    Serial.println(F("Unknown command. Type 'help' for a list of commands."));
}

void printHelp() {
    Serial.println();
    Serial.println(F("Available commands:"));
    Serial.println(F("  help           - Show this message"));
    Serial.println(F("  status         - Print current modem settings"));
    Serial.println(F("  freq <value>   - Set frequency (MHz or Hz)"));
    Serial.println(F("  sf <6-12>      - Set spreading factor"));
    Serial.println(F("  bw <125|250|500>- Set bandwidth in kHz"));
    Serial.println(F("  cr <5-8>       - Set coding rate (4/x)"));
    Serial.println(F("  txp <2-17>     - Set TX power in dBm"));
    Serial.println(F("  dump [start end]- Dump registers (optional range)"));
    Serial.println(F("  reset          - Reapply default settings"));
    Serial.println();
}

void printStatus() {
    modem.dumpStatus(Serial);
}

void applyDefaults(bool force) {
    if (force || !modem.ready()) {
        if (!modem.begin(DEFAULT_FREQUENCY_HZ,
                         DEFAULT_SPREADING_FACTOR,
                         DEFAULT_BANDWIDTH_KHZ,
                         DEFAULT_CODING_RATE,
                         DEFAULT_TX_POWER_DBM)) {
            Serial.println(F("Failed to initialise LoRa modem. Check wiring."));
            return;
        }
    }

    modem.setFrequency(DEFAULT_FREQUENCY_HZ);
    modem.setSpreadingFactor(DEFAULT_SPREADING_FACTOR);
    modem.setBandwidth(DEFAULT_BANDWIDTH_KHZ);
    modem.setCodingRate(DEFAULT_CODING_RATE);
    modem.setTxPower(DEFAULT_TX_POWER_DBM);
    printStatus();
}
