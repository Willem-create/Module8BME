#include <dummy.h>

// MPU-6050 Short Example Sketch
// By Arduino User JohnChi
// August 17, 2014
// Public Domain
#include<Wire.h>
#include<BluetoothSerial.h>
#include "Esp.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "driver\rtc_io.h"

#define PAIRING_TIME 30000

#define PAIR_PIN 26
#define POWER_OFF_PIN 32
#define POWER_OFF_PIN_GPIO GPIO_NUM_32
#define WIRED_MODE_PIN 33
#define WIRELESS_MODE_PIN 25

BluetoothSerial SerialBT;

const int MPU_addr = 0x68; // I2C address of the MPU-6050
byte output[12];

TaskHandle_t readIMU, bluetoothHandler, pairingTask;


void setup() {
  Serial.begin(115200);

  //Set pinmodes for slider en button
  pinMode(PAIR_PIN, INPUT_PULLUP);
  pinMode(POWER_OFF_PIN, INPUT_PULLUP);
  pinMode(WIRED_MODE_PIN, INPUT_PULLUP);
  pinMode(WIRELESS_MODE_PIN, INPUT_PULLUP);

  //Wake up MPU-6050 and set accelerometer and gyroscope to correct ranges
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x1B);
  Wire.write(0b00011000);
  Wire.endTransmission(false);
  Wire.beginTransmission(MPU_addr);
  Wire.write(0b00011000);
  Wire.endTransmission(false);
  Wire.endTransmission(true);

  //Enable bluetooth
  startBluetoothStack();
  SerialBT.begin("WirelessIMU-"+getBluetoothAddress());
  discoverable(false);
  Serial.println("Not discoverable");

  //Create repeating tasks
  xTaskCreatePinnedToCore(
    readIMUCode,
    "ReadIMU",
    10000,
    NULL,
    0,
    &readIMU,
    0
  );

  xTaskCreatePinnedToCore(
    bluetoothHandlerCode,
    "BluetoothHandler",
    10000,
    NULL,
    1,
    &bluetoothHandler,
    1
  );

  xTaskCreatePinnedToCore(
    pairingCode,
    "ParingTask",
    10000,
    NULL,
    1,
    &pairingTask,
    1
  );
}

void readIMUCode( void * parameter) {
  for (;;) {
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr, 14, true); // request a total of 14 registers
    for (int i = 0; i < 14; i++) {
      if (i < 6) {
        output[i] = Wire.read();
      } else if (i > 7) {
        output[i - 2] = Wire.read();
      }
    }
  }
}

void bluetoothHandlerCode ( void * parameters) {
  for (;;) {

    if (SerialBT.available() > 0) {
      SerialBT.read();
      SerialBT.write(output, 12);
    }
  }
}

void pairingCode ( void * parameters) {
  for (;;) {
    if (digitalRead(PAIR_PIN) == LOW) {
      discoverable(true);
      Serial.println("Discoverable");
      delay(PAIRING_TIME);
      discoverable(false);
      Serial.println("Not discoverable");
    }
  }
}

void startBluetoothStack() {
  btStart();
  esp_bluedroid_init();
  esp_bluedroid_enable();
}

String getBluetoothAddress() {
  const uint8_t* address = esp_bt_dev_get_address();
  String addrString = String();
  for (int i = 0; i < 2; i++) {
    char str[3];
    sprintf(str, "%02X", (int)address[i]);
    addrString = String(addrString + str);
  }
  return addrString;
}

void printWakeupReason() {
  esp_sleep_wakeup_cause_t wakeupReason;
  wakeupReason = esp_sleep_get_wakeup_cause();

  switch (wakeupReason) {
    case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1 : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP : Serial.println("Wakeup caused by ULP program"); break;
    default : Serial.printf("Wakeup was not caused by deep sleep: %d\n", wakeupReason); break;
  }
}

void startSleep() {
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);
  Wire.write(0b01001000);
  Wire.endTransmission(false);
  esp_sleep_enable_ext0_wakeup(POWER_OFF_PIN_GPIO, 1);
  Serial.println("start sleep");
  esp_deep_sleep_start();
}

void discoverable(bool dis) {
  if (dis) {
    esp_bt_gap_set_scan_mode(ESP_BT_SCAN_MODE_CONNECTABLE_DISCOVERABLE);
  } else {
    esp_bt_gap_set_scan_mode(ESP_BT_SCAN_MODE_CONNECTABLE);
  }
}


void loop() {
  if (digitalRead(POWER_OFF_PIN) == LOW) {
    startSleep();
  }
}
