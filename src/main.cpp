#include <Arduino.h>
#include <Adafruit_MCP4728.h>
#include <Wire.h>
#include <VnV_PCF8574.h>
#include "pin_mapping.h"
#include "can_task.h"

#define MCP4728_I2CADDR1 0x60
#define MCP4728_I2CADDR2 0x61
#define PCF8574_I2CADDR1 0x20
#define PCF8574_I2CADDR2 0x22
#define PCF8574_I2CADDR3 0x21
#define PCF8574_I2CADDR4 0x24

Adafruit_MCP4728 mcp1;
Adafruit_MCP4728 mcp2;
VnV_PCF8574 pcf1(Wire, PCF8574_I2CADDR1);
VnV_PCF8574 pcf2(Wire, PCF8574_I2CADDR2);
VnV_PCF8574 pcf3(Wire, PCF8574_I2CADDR3);
VnV_PCF8574 pcf4(Wire, PCF8574_I2CADDR4);


bool dac1_ok = false;
bool dac2_ok = false;
bool pcf1_ok = false;
bool pcf2_ok = false;
bool pcf3_ok = false;
bool pcf4_ok = false;

uint16_t dacValues[8] = {0};// gia tri M1-M8 da ghi thanh cong -- cho lenh S:? doc lai
uint8_t x9c_values[4] = {0};
bool isUpdateScucessful = false;


bool setMCPValue(String command);
bool setRelayValue(String command);
String processCommand(String command);
bool initPCF(VnV_PCF8574 &pcf, uint8_t number, uint8_t address, bool output = true);
// void reportInputs();
// void reportState();

void setup() {
  Serial.begin(115200);
  Wire.begin(PIN_SDA, PIN_SCL);
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
  pcf1_ok = initPCF(pcf1, 1, PCF8574_I2CADDR1, true);
  pcf2_ok = initPCF(pcf2, 2, PCF8574_I2CADDR2, true);
  pcf3_ok = initPCF(pcf3, 3, PCF8574_I2CADDR3, false);
  pcf4_ok = initPCF(pcf4, 4, PCF8574_I2CADDR4, false);

  pinMode(PIN_DAC, OUTPUT);
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

bool initPCF(VnV_PCF8574 &pcf, uint8_t number, uint8_t address, bool output) {
  // Preserve the original startup state: all pins LOW.
  // For active-low relays, use 0xFF for all-relays-OFF startup.
  if (!pcf.begin(0x00)) {
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
  // if (command.startsWith("X")) {
  //   serialx9c(command);
  //   return "X command processed";
  // }
  char type = command.charAt(0);
  if (type == 'M') {
    return setMCPValue(command) ? "M command processed" : "M command failed";
  }
  if (type == 'R') {
    return setRelayValue(command) ? "R command processed" : "R command failed";
  }
  // // S: "S:?" -- dump trang thai output dang giu (DAC/Relay/X9C/PWM) de doi chieu voi lenh da gui
  // if (type == 'S') {
  //   reportState();
  //   return "State reported";
  // }
  return "Invalid command prefix";
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
