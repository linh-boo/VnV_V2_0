#include <Arduino.h>
#include <Adafruit_MCP4728.h>
#include <Wire.h>
#include <SPI.h>
#include <VnV_PCF8574.h>
#include "pin_mapping.h"
#include "can_task.h"
#include <VnV_MCP41100.h>
#include <math.h>

#define MCP4728_I2CADDR1 0x60
#define MCP4728_I2CADDR2 0x61
#define PCF8574_I2CADDR1 0x20 // CS1-CS4: P0-P3
#define PCF8574_I2CADDR2 0x22 // RELAY1-RELAY8
#define PCF8574_I2CADDR3 0x21 // PWM input signals
#define PCF8574_I2CADDR4 0x24 // PWM outputs

Adafruit_MCP4728 mcp1;
Adafruit_MCP4728 mcp2;
VnV_PCF8574 pcf1(Wire, PCF8574_I2CADDR1);
VnV_PCF8574 pcf2(Wire, PCF8574_I2CADDR2);
VnV_PCF8574 pcf3(Wire, PCF8574_I2CADDR3);
VnV_PCF8574 pcf4(Wire, PCF8574_I2CADDR4);
VnV_MCP41100 rpot1(SPI, pcf1, 0); // CS1 = PCF1 P0
VnV_MCP41100 rpot2(SPI, pcf1, 1); // CS2 = PCF1 P1
VnV_MCP41100 rpot3(SPI, pcf1, 2); // CS3 = PCF1 P2
VnV_MCP41100 rpot4(SPI, pcf1, 3); // CS4 = PCF1 P3


bool dac1_ok = false;
bool dac2_ok = false;
bool pcf1_ok = false;
bool pcf2_ok = false;
bool pcf3_ok = false;
bool pcf4_ok = false;
bool rpot1_ok = false;
bool rpot2_ok = false;
bool rpot3_ok = false;
bool rpot4_ok = false;
uint8_t eloadDacCode = 0;

uint16_t dacValues[8] = {0};// gia tri M1-M8 da ghi thanh cong -- cho lenh S:? doc lai
uint8_t x9c_values[4] = {0};
bool isUpdateScucessful = false;

// E-load shunt rating: 50 A at 75 mV => 1.5 mOhm.
constexpr float ELOAD_SHUNT_OHMS = 0.075f / 50.0f;
constexpr float ELOAD_DAC_TO_SHUNT = 0.17544f;
constexpr uint32_t ELOAD_MAX_MA = 50000;


bool setMCPValue(String command);
bool setRelayValue(String command);
bool setRpotValue(String command);
bool setELoadValue(String command);
String processCommand(String command);
bool initPCF(VnV_PCF8574 &pcf, uint8_t number, uint8_t address,
             bool output = true, uint8_t initialState = 0x00);
// void reportInputs();
// void reportState();

void setup() {
  Serial.begin(115200);
  Wire.begin(PIN_SDA, PIN_SCL);
  SPI.begin(PIN_SCK, -1, PIN_SIO, -1);
  delay(2000);

  // Initialize the MCP4728 DAC
  if (!mcp1.begin(MCP4728_I2CADDR1)) {
    Serial.println("Failed to find MCP4728 chip at address 1. Check your wiring!");
  }
  else {
    dac1_ok = true;
    // Set the DAC output to mid-scale (0 for a 12-bit DAC)
    mcp1.setChannelValue(MCP4728_CHANNEL_A, 0);
    mcp1.setChannelValue(MCP4728_CHANNEL_B, 0);
    mcp1.setChannelValue(MCP4728_CHANNEL_C, 0);
    mcp1.setChannelValue(MCP4728_CHANNEL_D, 0);
  }
  if( !mcp2.begin(MCP4728_I2CADDR2)) {
    Serial.println("Failed to find MCP4728 chip at address 2. Check your wiring!");
  }
  else {
    dac2_ok = true;
    mcp2.setChannelValue(MCP4728_CHANNEL_A, 0);
    mcp2.setChannelValue(MCP4728_CHANNEL_B, 0);
    mcp2.setChannelValue(MCP4728_CHANNEL_C, 0);
    mcp2.setChannelValue(MCP4728_CHANNEL_D, 0);
  }
  // PCF1 P0-P3 are MCP41100 CS lines and must idle HIGH.
  pcf1_ok = initPCF(pcf1, 1, PCF8574_I2CADDR1, true, 0x0F);
  pcf2_ok = initPCF(pcf2, 2, PCF8574_I2CADDR2, true, 0x00);
  pcf3_ok = initPCF(pcf3, 3, PCF8574_I2CADDR3, false, 0xFF);
  pcf4_ok = initPCF(pcf4, 4, PCF8574_I2CADDR4, true, 0x00);

  // MCP41100 wiper code: 0 = minimum, 255 = maximum.
  rpot1_ok = rpot1.begin(0x00);
  rpot2_ok = rpot2.begin(0x00);
  rpot3_ok = rpot3.begin(0x00);
  rpot4_ok = rpot4.begin(0x00);
  Serial.printf("MCP41100: R1=%s R2=%s R3=%s R4=%s\n",
                rpot1_ok ? "OK" : "FAIL", rpot2_ok ? "OK" : "FAIL",
                rpot3_ok ? "OK" : "FAIL", rpot4_ok ? "OK" : "FAIL");

  pinMode(PIN_DAC, OUTPUT);
  // E-load disabled at boot: GPIO25 DAC output is 0 V.
  dacWrite(PIN_DAC, 0);
  analogReadResolution(12);
  analogSetPinAttenuation(PIN_ADC_VIN, ADC_11db);
  analogSetPinAttenuation(PIN_ADC_IOUT, ADC_11db);
  pinMode(PIN_PWM1, OUTPUT);
  pinMode(PIN_PWM2, OUTPUT);
  pinMode(PIN_PWM3, OUTPUT);
  pinMode(PIN_PWM4, OUTPUT);
  pinMode(PIN_PWM5, OUTPUT);
  pinMode(PIN_PWM6, OUTPUT);

  // Arduino loopTask is pinned to core 0 by platformio.ini. CAN is isolated
  // on core 1; it only receives/decodes frames and does not touch I2C outputs.
  startCanTask();

  Serial.println("VnV 2.0 ready. Enter commands:");
}

bool initPCF(VnV_PCF8574 &pcf, uint8_t number, uint8_t address,
             bool output, uint8_t initialState) {
  // Preserve the original startup state: all pins LOW.
  // For active-low relays, use 0xFF for all-relays-OFF startup.
  if (!pcf.begin(initialState)) {
    Serial.printf("Failed to find PCF8574 #%u at 0x%02X\n", number, address);
    return false;
  }

  for (uint8_t pin = 0; pin < 8; pin++) {
    if (!pcf.pinMode(pin, output? OUTPUT : INPUT)) {
      Serial.printf("PCF8574 #%u pin %u configuration failed\n", number, pin);
      return false;
    }
  }

  Serial.printf("PCF8574 #%u ready at 0x%02X\n", number, address);
  return true;
}

void loop() {
  if(Serial.available())
  {
    String serialRead = Serial.readStringUntil('\n');
    String response = processCommand(serialRead);
    Serial.println(response);   // in ket qua lenh ve cho nguoi dung
  }
  delay(1);
}

String processCommand(String command) {
  command.trim();
  // < 3 (khong phai <= 3): lenh 3 ky tu hop le ton tai -- "F:0" (dung PWM), "A:1"/"A:0"/"A:?"
  if (command.length() < 3) {
    return "Command too short";
  }
  // [2026-07-20] LENH CHUNG (dung format Clamp board) PHAI xet TRUOC dispatch 1-ky-tu:
  // "ACC_ON_1" bat dau bang 'A' -> neu xet sau se roi nham vao nhanh ADC ('A').
  // if (setClampCommand(command)) {
  //   return "Clamp command processed";
  // }
  char type = command.charAt(0);
  if (type == 'M') {
    return setMCPValue(command) ? "M command processed" : "M command failed";
  }
  if (type == 'R') {
    return setRelayValue(command) ? "R command processed" : "R command failed";
  }
  if (type == 'P') {
    return setRpotValue(command) ? "P command processed" : "P command failed";
  }
  if (type == 'E') {
    return setELoadValue(command) ? "E command processed" : "E command failed";
  }
  // // S: "S:?" -- dump trang thai output dang giu (DAC/Relay/X9C/PWM) de doi chieu voi lenh da gui
  // if (type == 'S') {
  //   reportState();
  //   return "State reported";
  // }
  return "Invalid command prefix";
}

bool setRpotValue(String command) {
  if (command.length() < 4 || command.charAt(2) != ':') {
    Serial.println("Invalid command. Usage: Pn:VALUE where n=1-4 and VALUE=0-255");
    return false;
  }

  const char channel = command.charAt(1);
  if (channel < '1' || channel > '4') {
    Serial.println("Invalid command. Usage: Pn:VALUE where n=1-4 and VALUE=0-255");
    return false;
  }

  const String valueText = command.substring(3);
  for (size_t i = 0; i < valueText.length(); ++i) {
    if (valueText.charAt(i) < '0' || valueText.charAt(i) > '9') {
      Serial.println("Invalid value. VALUE must be between 0 and 255.");
      return false;
    }
  }
  const long value = valueText.toInt();
  if (value < 0 || value > 255) {
    Serial.println("Invalid value. VALUE must be between 0 and 255.");
    return false;
  }

  VnV_MCP41100 *pot = nullptr;
  bool available = false;
  switch (channel) {
    case '1': pot = &rpot1; available = rpot1_ok; break;
    case '2': pot = &rpot2; available = rpot2_ok; break;
    case '3': pot = &rpot3; available = rpot3_ok; break;
    case '4': pot = &rpot4; available = rpot4_ok; break;
  }

  if (!available || pot == nullptr || !pot->setWiper(static_cast<uint8_t>(value))) {
    Serial.printf("P%c ERR: MCP41100 write failed\n", channel);
    return false;
  }

  Serial.printf("Set resistor %c to wiper code %ld/255\n", channel, value);
  return true;
}

bool setELoadValue(String command) {
  if (command == "E:?") {
    const uint16_t vinRaw = analogRead(PIN_ADC_VIN);
    const uint16_t ioutRaw = analogRead(PIN_ADC_IOUT);
    const float adcVoltage = (static_cast<float>(vinRaw) * 3.3f) / 4095.0f;
    const float inputVoltage = adcVoltage * 12.0f;
    const float dacVoltage =
        (static_cast<float>(eloadDacCode) * 3.3f) / 255.0f;
    const float setCurrent =
        (ELOAD_DAC_TO_SHUNT * dacVoltage) / ELOAD_SHUNT_OHMS;

    Serial.printf("ELOAD DAC=%u VDAC=%.3fV ISET=%.3fA VIN=%.3fV "
                  "ADC_VIN=%u ADC_IOUT=%u\n",
                  eloadDacCode, dacVoltage, setCurrent, inputVoltage, vinRaw,
                  ioutRaw);
    return true;
  }

  if (command.length() < 3 || command.charAt(1) != ':') {
    Serial.println("Invalid command. Usage: E:CURRENT_mA or E:?");
    return false;
  }

  const String currentText = command.substring(2);
  if (currentText.length() == 0) {
    Serial.println("Invalid current. Use 0 to 50000 mA.");
    return false;
  }
  for (size_t i = 0; i < currentText.length(); ++i) {
    if (currentText.charAt(i) < '0' || currentText.charAt(i) > '9') {
      Serial.println("Invalid current. Use 0 to 50000 mA.");
      return false;
    }
  }

  const long currentMilliamp = currentText.toInt();
  if (currentMilliamp > static_cast<long>(ELOAD_MAX_MA)) {
    Serial.println("Invalid current. Shunt limit is 50000 mA.");
    return false;
  }

  // VSHUNT = 0.17544 * VDAC and I = VSHUNT / 0.0015 ohm.
  const float dacVoltage = (static_cast<float>(currentMilliamp) / 1000.0f) *
                           ELOAD_SHUNT_OHMS / ELOAD_DAC_TO_SHUNT;
  const long dacCode = lroundf((dacVoltage / 3.3f) * 255.0f);
  if (dacCode < 0 || dacCode > 255) {
    Serial.println("Current exceeds the DAC range.");
    return false;
  }

  dacWrite(PIN_DAC, static_cast<uint8_t>(dacCode));
  eloadDacCode = static_cast<uint8_t>(dacCode);
  Serial.printf("ELOAD set to %ld mA, DAC code %u\n", currentMilliamp,
                eloadDacCode);
  return true;
}

bool setRelayValue(String command) {
  char channel = command.charAt(1);
  String value = command.substring(3);
  if (value.length() == 0) {
    Serial.println("Invalid command. Usage: Rn:ON or Rn:OFF where n=1-8");
    return false;
  }
  if (value != "ON" && value != "OFF") {
    Serial.println("Invalid command. Usage: Rn:ON or Rn:OFF where n=1-8");
    return false;
  }
  if (!pcf2_ok) {
    Serial.println("Invalid command. Usage: Rn:ON or Rn:OFF where n=1-8");
    return false;
  }
  if (channel < '1' || channel > '8') {
    Serial.println("Invalid command. Usage: Rn:ON or Rn:OFF where n=1-8");  
    return false;
  }
  uint8_t idx = channel - '1'; // 0-7 for R1
  bool state = value.equalsIgnoreCase("ON");
  // Active-high relay: HIGH = ON, LOW = OFF.
  if (!pcf2.digitalWrite(idx, state ? HIGH : LOW)) {
    Serial.println("Relay I2C write failed");
    return false;
  }
  Serial.print("Set Relay ");
  Serial.print((int)(idx + 1));
  Serial.print(" to ");
  Serial.println(state ? "ON" : "OFF");
  return true;
}

bool setMCPValue(String command) {
  char channel = command.charAt(1);
  uint16_t value = command.substring(3).toInt();
  uint8_t idx;
  bool ok = false;
  if (channel < '1' || channel > '8') {
    Serial.println("Invalid command. Usage: Mn:VALUE where n=1-8 and VALUE=0-4095");
    return false;
  }
  if (value > 4095) {
    Serial.println("Invalid value. VALUE must be between 0 and 4095.");
    return false;
  }
  idx = channel - '1';                                   // 0-7: M1-M4 -> mcp1, M5-M8 -> mcp2
  if ((idx < 4 && !dac1_ok) || (idx >= 4 && !dac2_ok)) {
    Serial.printf("M%c ERR: MCP4728 chip %d not available\n", channel, (idx < 4) ? 1 : 2);
    return false;
  }
  {
    Adafruit_MCP4728 &chip = (idx < 4) ? mcp2 : mcp1;
    MCP4728_channel_t ch = (MCP4728_channel_t)(3- idx % 4);  // M1/M5=A, then B/C/D
    for (uint8_t attempt = 0; attempt < 3 && !ok; attempt++) {
      ok = chip.setChannelValue(ch, value);
      if (!ok) delayMicroseconds(300);
    }
  }
  if (ok) {
    dacValues[idx] = value;
    Serial.print("Set MCP4728 Channel ");
    Serial.print((int)(idx + 1));
    Serial.print(" to ");
    Serial.println(value);
    return true;
  } else {
    Serial.printf("M%c ERR: I2C write failed after 3 retries\n", channel);
    return false;
  }
}
