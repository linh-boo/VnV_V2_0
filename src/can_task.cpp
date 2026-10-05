#include "can_task.h"

#include <Arduino.h>
#include <LittleFS.h>
#include <driver/twai.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "pin_mapping.h"

namespace {

constexpr char DBC_PATH[] = "/vnv.dbc";
constexpr uint32_t CAN_BITRATE = 500000;
constexpr size_t MAX_DBC_MESSAGES = 64;
constexpr size_t MAX_DBC_SIGNALS = 16;
constexpr size_t MAX_NAME_LENGTH = 31;
constexpr size_t MAX_UNIT_LENGTH = 15;

struct DbcSignal {
  char name[MAX_NAME_LENGTH + 1] = {};
  uint8_t startBit = 0;
  uint8_t bitLength = 0;
  bool littleEndian = true;
  bool signedValue = false;
  double factor = 1.0;
  double offset = 0.0;
  char unit[MAX_UNIT_LENGTH + 1] = {};
};

struct DbcMessage {
  uint32_t id = 0;
  char name[MAX_NAME_LENGTH + 1] = {};
  uint8_t dlc = 0;
  uint8_t signalCount = 0;
  DbcSignal signals[MAX_DBC_SIGNALS];
};

class DbcDatabase {
public:
  bool load(File &file) {
    _messageCount = 0;
    DbcMessage *current = nullptr;
    char line[256];

    while (file.available()) {
      size_t length = file.readBytesUntil('\n', line, sizeof(line) - 1);
      line[length] = '\0';
      if (length > 0 && line[length - 1] == '\r') {
        line[length - 1] = '\0';
      }

      char *text = line;
      while (*text == ' ' || *text == '\t') {
        ++text;
      }
      if (*text == '\0' || *text == '/' || *text == '\n') {
        continue;
      }

      if (strncmp(text, "BO_ ", 4) == 0) {
        if (_messageCount >= MAX_DBC_MESSAGES) {
          continue;
        }

        unsigned long id = 0;
        unsigned int dlc = 0;
        char name[MAX_NAME_LENGTH + 1] = {};
        if (sscanf(text, "BO_ %lu %31[^:]: %u", &id, name, &dlc) == 3) {
          current = &_messages[_messageCount++];
          current->id = static_cast<uint32_t>(id) & 0x1FFFFFFF;
          strncpy(current->name, name, MAX_NAME_LENGTH);
          current->name[MAX_NAME_LENGTH] = '\0';
          current->dlc = static_cast<uint8_t>(dlc > 8 ? 8 : dlc);
          current->signalCount = 0;
        } else {
          current = nullptr;
        }
        continue;
      }

      if (strncmp(text, "SG_ ", 4) == 0 && current != nullptr) {
        if (current->signalCount >= MAX_DBC_SIGNALS) {
          continue;
        }

        DbcSignal signal;
        char byteOrder = '1';
        char sign = '+';
        unsigned int startBit = 0;
        unsigned int bitLength = 0;
        double minimum = 0.0;
        double maximum = 0.0;
        char unit[MAX_UNIT_LENGTH + 1] = {};

        const int fields = sscanf(
            text,
            "SG_ %31s : %u|%u@%c%c (%lf,%lf) [%lf|%lf] \"%15[^\"]\"",
            signal.name, &startBit, &bitLength, &byteOrder, &sign,
            &signal.factor, &signal.offset, &minimum, &maximum, unit);

        if (fields >= 8 && bitLength > 0 && bitLength <= 64 &&
            startBit < 64) {
          signal.startBit = static_cast<uint8_t>(startBit);
          signal.bitLength = static_cast<uint8_t>(bitLength);
          signal.littleEndian = (byteOrder == '1');
          signal.signedValue = (sign == '-');
          strncpy(signal.unit, unit, MAX_UNIT_LENGTH);
          signal.unit[MAX_UNIT_LENGTH] = '\0';
          current->signals[current->signalCount++] = signal;
        }
      }
    }

    return _messageCount > 0;
  }

  const DbcMessage *find(uint32_t id) const {
    const uint32_t normalizedId = id & 0x1FFFFFFF;
    for (size_t i = 0; i < _messageCount; ++i) {
      if (_messages[i].id == normalizedId) {
        return &_messages[i];
      }
    }
    return nullptr;
  }

  size_t messageCount() const { return _messageCount; }

  void printDecoded(uint32_t id, uint8_t dlc, const uint8_t *data) const {
    const DbcMessage *message = find(id);
    if (message == nullptr) {
      Serial.printf("CAN RX 0x%lX DLC=%u DATA=", static_cast<unsigned long>(id),
                    dlc);
      for (uint8_t i = 0; i < dlc && i < 8; ++i) {
        Serial.printf("%02X ", data[i]);
      }
      Serial.println();
      return;
    }

    Serial.printf("CAN RX 0x%lX %s", static_cast<unsigned long>(id),
                  message->name);
    for (uint8_t i = 0; i < message->signalCount; ++i) {
      const DbcSignal &signal = message->signals[i];
      const uint64_t raw = extract(signal, data, dlc);
      double physical = static_cast<double>(raw);

      if (signal.signedValue && signal.bitLength < 64 &&
          (raw & (1ULL << (signal.bitLength - 1))) != 0) {
        const uint64_t signExtended = raw | (~0ULL << signal.bitLength);
        physical = static_cast<double>(static_cast<int64_t>(signExtended));
      }
      physical = physical * signal.factor + signal.offset;

      Serial.printf(" %s=%.6g", signal.name, physical);
      if (signal.unit[0] != '\0') {
        Serial.printf("[%s]", signal.unit);
      }
    }
    Serial.println();
  }

private:
  static uint64_t extract(const DbcSignal &signal, const uint8_t *data,
                          uint8_t dlc) {
    uint64_t value = 0;
    if (signal.bitLength == 0 || signal.bitLength > 64) {
      return 0;
    }

    if (signal.littleEndian) {
      for (uint8_t bit = 0; bit < signal.bitLength; ++bit) {
        const uint16_t bitIndex = signal.startBit + bit;
        const uint8_t byteIndex = bitIndex / 8;
        if (byteIndex >= dlc) {
          break;
        }
        if ((data[byteIndex] & (1U << (bitIndex % 8))) != 0) {
          value |= (1ULL << bit);
        }
      }
    } else {
      int bitIndex = signal.startBit;
      for (uint8_t bit = 0; bit < signal.bitLength; ++bit) {
        const uint8_t byteIndex = bitIndex / 8;
        if (byteIndex >= dlc) {
          break;
        }
        value <<= 1;
        if ((data[byteIndex] & (1U << (bitIndex % 8))) != 0) {
          value |= 1;
        }
        bitIndex = (bitIndex % 8 == 0) ? bitIndex + 15 : bitIndex - 1;
      }
    }
    return value;
  }

  DbcMessage _messages[MAX_DBC_MESSAGES];
  size_t _messageCount = 0;
};

bool startTwai() {
  twai_general_config_t general = TWAI_GENERAL_CONFIG_DEFAULT(
      static_cast<gpio_num_t>(PIN_CAN_TX), static_cast<gpio_num_t>(PIN_CAN_RX),
      TWAI_MODE_NORMAL);
  general.tx_queue_len = 8;
  general.rx_queue_len = 32;
  general.alerts_enabled = TWAI_ALERT_BUS_OFF | TWAI_ALERT_BUS_RECOVERED |
                           TWAI_ALERT_RX_QUEUE_FULL;

  twai_timing_config_t timing = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t filter = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  esp_err_t error = twai_driver_install(&general, &timing, &filter);
  if (error != ESP_OK) {
    Serial.printf("CAN driver install failed: %s\n", esp_err_to_name(error));
    return false;
  }

  error = twai_start();
  if (error != ESP_OK) {
    Serial.printf("CAN driver start failed: %s\n", esp_err_to_name(error));
    twai_driver_uninstall();
    return false;
  }

  Serial.printf("CAN started on core %d, TX=%d RX=%d bitrate=%lu\n",
                xPortGetCoreID(), PIN_CAN_TX, PIN_CAN_RX,
                static_cast<unsigned long>(CAN_BITRATE));
  return true;
}

void canTask(void *) {
  // The message/signal table is intentionally static; putting it on the
  // FreeRTOS task stack would consume far more than the 8 KB task stack.
  static DbcDatabase database;

  if (!LittleFS.begin(false)) {
    Serial.println("CAN: LittleFS mount failed; DBC decoder disabled");
  } else {
    File dbcFile = LittleFS.open(DBC_PATH, FILE_READ);
    if (!dbcFile) {
      Serial.printf("CAN: DBC file not found: %s\n", DBC_PATH);
    } else if (!database.load(dbcFile)) {
      Serial.printf("CAN: no DBC messages loaded from %s\n", DBC_PATH);
      dbcFile.close();
    } else {
      Serial.printf("CAN: loaded %u DBC messages from %s\n",
                    static_cast<unsigned>(database.messageCount()), DBC_PATH);
      dbcFile.close();
    }
  }

  if (!startTwai()) {
    vTaskDelete(nullptr);
    return;
  }

  for (;;) {
    twai_message_t message = {};
    const esp_err_t receiveResult = twai_receive(&message, pdMS_TO_TICKS(100));
    if (receiveResult == ESP_OK && !message.rtr) {
      database.printDecoded(message.identifier, message.data_length_code,
                             message.data);
    }

    uint32_t alerts = 0;
    if (twai_read_alerts(&alerts, 0) == ESP_OK &&
        (alerts & TWAI_ALERT_BUS_OFF) != 0) {
      Serial.println("CAN bus-off; starting recovery");
      twai_initiate_recovery();
    }
    if ((alerts & TWAI_ALERT_BUS_RECOVERED) != 0) {
      Serial.println("CAN bus recovered");
      twai_start();
    }
  }
}

} // namespace

void startCanTask() {
  TaskHandle_t handle = nullptr;
  const BaseType_t result = xTaskCreatePinnedToCore(
      canTask, "CAN_DBC", 8192, nullptr, 4, &handle, 1);
  if (result != pdPASS) {
    Serial.println("CAN task creation failed");
  }
}
