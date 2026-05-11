// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program. If not, see <https://www.gnu.org/licenses/>.

// Copyright (c) 2025 Aman Maheshwari     - Aman@upsidedownlabs.tech
// Copyright (c) 2024-2025 Krishnanshu Mittal - krishnanshu@upsidedownlabs.tech
// Copyright (c) 2024-2025 Deepak Khatri      - deepak@upsidedownlabs.tech
// Copyright (c) 2024-2025 Upside Down Labs   - contact@upsidedownlabs.tech

// At Upside Down Labs, we create open-source DIY neuroscience hardware and software.
// Our mission is to make neuroscience affordable and accessible for everyone.
// By supporting us with your purchase, you help spread innovation and open science.
// Thank you for being part of this journey with us!

/*
 This example is adapted from the client and server code provided by MoThunderz
 Firmware: https://github.com/mo-thunderz/Esp32BlePart2
 YouTube video: https://www.youtube.com/watch?v=s3yoZa6kzus
*/
#include <Arduino.h>
#include "BLEDevice.h"
#include <Preferences.h>   // ESP32 NVS - survives power cycles

// ================================================================
//  MAC PAIRING - Automatic via Serial on first boot
//
//  First boot (or after clearing NVS):
//    1. Flash ble_server_remote.ino to the NPG Lite and open its
//       Serial Monitor. Copy the MAC address it prints.
//    2. Flash this firmware to the car, open its Serial Monitor.
//    3. Paste the MAC when prompted and press Enter.
//    4. The MAC is saved to flash - no re-entry needed on reboot.
//
//  To reset the saved MAC: type "RESET" (without quotes) in the
//  car's Serial Monitor while it is waiting for commands, then
//  reset the board. You will be prompted for a new MAC.
// ================================================================

// BLE UUIDs - must match server
static BLEUUID serviceUUID("4fafc201-1fb5-459e-8fcc-c5c9c331914b");
static BLEUUID charUUID_1 ("beb5483e-36e1-4688-b7f5-ea07361b26a8");
static BLEUUID charUUID_2 ("1c95d5e3-d8f7-413a-bf3d-7a2e5d7be87e");

// Motor pins (MX1508: IN1-IN4)
const int PIN_L_FWD = 0;
const int PIN_L_BWD = 1;
const int PIN_R_FWD = 21;
const int PIN_R_BWD = 2;

// LEDC channels
const int CH_L_FWD = 0;
const int CH_L_BWD = 1;
const int CH_R_FWD = 2;
const int CH_R_BWD = 3;

// PWM config
const int PWM_FREQ = 25000;
const int PWM_RES  = 8;
const int PWM_MAX  = 200;

// Soft-start ramp
const int RAMP_START    = 220;
const int RAMP_STEPS    = 0;
const int RAMP_DELAY_MS = 12;

// Beep config
const int BEEP_FREQ   = 7000;
const int BEEP_DUTY   = 128;
const int BEEP_ON_MS  = 120;
const int BEEP_OFF_MS = 100;

// ---------------------------------------------------------------
//  Preferences (NVS) - stores MAC across power cycles
// ---------------------------------------------------------------
Preferences prefs;
const char* PREFS_NS  = "bci-car";   // namespace
const char* PREFS_KEY = "server_mac"; // key for the MAC string

String targetMac = "";  // populated in setup() from NVS or Serial

// ---------------------------------------------------------------
//  BLE state
// ---------------------------------------------------------------
static boolean doConnect    = false;
static boolean connected    = false;
static boolean doScan       = false;
static boolean justConnected = false;

static BLEAdvertisedDevice*     myDevice      = nullptr;
static BLERemoteCharacteristic* pRemoteChar_1 = nullptr;
static BLERemoteCharacteristic* pRemoteChar_2 = nullptr;
static BLEClient*               pClient       = nullptr;

volatile uint32_t currentCommand = 0;
volatile bool     newCommand     = false;

// ---------------------------------------------------------------
//  PWM helpers
// ---------------------------------------------------------------
inline void pwmWrite(int pin, int duty) { ledcWrite(pin, duty); }

void stopMotors() {
  pwmWrite(PIN_L_FWD, 0); pwmWrite(PIN_L_BWD, 0);
  pwmWrite(PIN_R_FWD, 0); pwmWrite(PIN_R_BWD, 0);
}

void softStartBoth(bool dirL, bool dirR, int dutyL, int dutyR) {
  stopMotors();
  int aPin_L = dirL ? PIN_L_FWD : PIN_L_BWD;
  int iPin_L = dirL ? PIN_L_BWD : PIN_L_FWD;
  int aPin_R = dirR ? PIN_R_FWD : PIN_R_BWD;
  int iPin_R = dirR ? PIN_R_BWD : PIN_R_FWD;
  pwmWrite(iPin_L, 0); pwmWrite(iPin_R, 0);

  // RAMP_STEPS = 0 means instant start - guard against divide-by-zero
  if (RAMP_STEPS > 0) {
    int stepL = max(1, (dutyL - RAMP_START) / RAMP_STEPS);
    int stepR = max(1, (dutyR - RAMP_START) / RAMP_STEPS);
    int curL  = RAMP_START, curR = RAMP_START;
    while (curL < dutyL || curR < dutyR) {
      if (curL < dutyL) curL = min(curL + stepL, dutyL);
      if (curR < dutyR) curR = min(curR + stepR, dutyR);
      pwmWrite(aPin_L, curL);
      pwmWrite(aPin_R, curR);
      delay(RAMP_DELAY_MS);
    }
  }
  pwmWrite(aPin_L, dutyL);
  pwmWrite(aPin_R, dutyR);
}

void softStartOne(int fwdPin, int bwdPin, bool forward, int targetDuty) {
  int aPin = forward ? fwdPin : bwdPin;
  int iPin = forward ? bwdPin : fwdPin;
  pwmWrite(iPin, 0);
  if (RAMP_STEPS > 0) {
    int step = max(1, (targetDuty - RAMP_START) / RAMP_STEPS);
    for (int d = RAMP_START; d < targetDuty; d += step) {
      pwmWrite(aPin, d);
      delay(RAMP_DELAY_MS);
    }
  }
  pwmWrite(aPin, targetDuty);
}

// ---------------------------------------------------------------
//  Beep
// ---------------------------------------------------------------
void beepConnect() {
  ledcAttachChannel(PIN_L_FWD, BEEP_FREQ, PWM_RES, CH_L_FWD);
  ledcAttachChannel(PIN_R_FWD, BEEP_FREQ, PWM_RES, CH_R_FWD);
  pwmWrite(PIN_L_BWD, 0); pwmWrite(PIN_R_BWD, 0);
  for (int b = 0; b < 2; b++) {
    pwmWrite(PIN_L_FWD, BEEP_DUTY); pwmWrite(PIN_R_FWD, BEEP_DUTY);
    delay(BEEP_ON_MS);
    pwmWrite(PIN_L_FWD, 0); pwmWrite(PIN_R_FWD, 0);
    if (b == 0) delay(BEEP_OFF_MS);
  }
  ledcAttachChannel(PIN_L_FWD, PWM_FREQ, PWM_RES, CH_L_FWD);
  ledcAttachChannel(PIN_R_FWD, PWM_FREQ, PWM_RES, CH_R_FWD);
  stopMotors();
}

// ---------------------------------------------------------------
//  Movement
// ---------------------------------------------------------------
void driveForward()  { Serial.println("fwd");   softStartBoth(true,  true,  PWM_MAX, PWM_MAX); }
void driveBackward() { Serial.println("bwd");   softStartBoth(false, false, PWM_MAX, PWM_MAX); }
void turnLeft()      { Serial.println("left");  stopMotors(); softStartOne(PIN_R_FWD, PIN_R_BWD, true, PWM_MAX); }
void turnRight()     { Serial.println("right"); stopMotors(); softStartOne(PIN_L_FWD, PIN_L_BWD, true, PWM_MAX); }

uint32_t activeCommand = 255;

void applyCommand(uint32_t cmd) {
  if (cmd == activeCommand) return;
  activeCommand = cmd;
  switch (cmd) {
    case 1:  turnLeft();      break;
    case 2:  turnRight();     break;
    case 3:  driveForward();  break;
    case 4:  driveBackward(); break;
    default: stopMotors();    break;
  }
}

// ---------------------------------------------------------------
//  BLE notify callback
// ---------------------------------------------------------------
static void notifyCallback(BLERemoteCharacteristic* pChar,
                           uint8_t* pData, size_t length, bool isNotify) {
  if (pChar->getUUID().toString() != charUUID_1.toString()) return;
  uint32_t cmd = 0;
  for (int i = 0; i < (int)length && i < 4; i++)
    cmd |= ((uint32_t)pData[i]) << (i * 8);
  currentCommand = cmd;
  newCommand     = true;
}

class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient* pclient) {}
  void onDisconnect(BLEClient* pclient) {
    connected     = false;
    doScan        = true;
    activeCommand = 255;
    stopMotors();
    Serial.println("BLE disconnected");
  }
};

bool connectCharacteristic(BLERemoteService* svc,
                           BLERemoteCharacteristic*& out, BLEUUID uuid) {
  out = svc->getCharacteristic(uuid);
  if (!out) return false;
  if (out->canNotify()) out->registerForNotify(notifyCallback);
  return true;
}

bool connectToServer() {
  if (pClient) {
    if (pClient->isConnected()) pClient->disconnect();
    delete pClient; pClient = nullptr;
  }
  pClient = BLEDevice::createClient();
  pClient->setClientCallbacks(new MyClientCallback());
  if (!pClient->connect(myDevice)) return false;

  BLERemoteService* svc = pClient->getService(serviceUUID);
  if (!svc) { pClient->disconnect(); return false; }

  bool ok = connectCharacteristic(svc, pRemoteChar_1, charUUID_1)
          & connectCharacteristic(svc, pRemoteChar_2, charUUID_2);
  if (!ok) { pClient->disconnect(); return false; }

  connected     = true;
  justConnected = true;
  return true;
}

class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice dev) {
    if (!dev.haveServiceUUID() || !dev.isAdvertisingService(serviceUUID)) return;

    String found  = dev.getAddress().toString().c_str();
    String target = targetMac;
    found.toLowerCase(); target.toLowerCase();

    // If targetMac is empty or placeholder, connect to any matching server
    if (target.length() > 0 && target != "xx:xx:xx:xx:xx:xx" && found != target) return;

    BLEDevice::getScan()->stop();
    if (myDevice) delete myDevice;
    myDevice  = new BLEAdvertisedDevice(dev);
    doConnect = true;
    doScan    = false;
    Serial.println("Server found: " + found);
  }
};

// ---------------------------------------------------------------
//  MAC management - load from NVS or prompt user over Serial
// ---------------------------------------------------------------

// Validate that a string looks like a MAC address (xx:xx:xx:xx:xx:xx)
bool isValidMac(const String &mac) {
  if (mac.length() != 17) return false;
  for (int i = 0; i < 17; i++) {
    if (i % 3 == 2) {
      if (mac[i] != ':') return false;
    } else {
      char c = tolower(mac[i]);
      if (!isxdigit(c)) return false;
    }
  }
  return true;
}

// Prompt the user to enter a MAC over Serial and save it to NVS
String promptAndSaveMac() {
  Serial.println("============================================");
  Serial.println("  Car First-Boot MAC Setup                 ");
  Serial.println("============================================");
  Serial.println("Open the NPG Lite Serial Monitor and copy  ");
  Serial.println("the MAC address it printed on startup.     ");
  Serial.println("Paste it here and press Enter:             ");
  Serial.println("  Format: xx:xx:xx:xx:xx:xx (lowercase)    ");
  Serial.println("============================================");

  String input = "";
  while (true) {
    // Wait until something arrives
    while (!Serial.available()) delay(10);

    input = Serial.readStringUntil('\n');
    input.trim();
    input.toLowerCase();

    if (input.length() == 0) {
      Serial.println("Empty input - please try again:");
      continue;
    }
    if (!isValidMac(input)) {
      Serial.println("Invalid format. Expected xx:xx:xx:xx:xx:xx - please try again:");
      continue;
    }
    break;
  }

  // Save to NVS
  prefs.begin(PREFS_NS, false);          // open read-write
  prefs.putString(PREFS_KEY, input);
  prefs.end();

  Serial.println("Saved MAC: " + input);
  Serial.println("This will be used automatically on future boots.");
  Serial.println("To change it, type RESET in Serial Monitor and reset the board.");
  return input;
}

// Load MAC from NVS; return empty string if not set
String loadMac() {
  prefs.begin(PREFS_NS, true);           // open read-only
  String mac = prefs.getString(PREFS_KEY, "");
  prefs.end();
  return mac;
}

// Erase saved MAC from NVS
void clearMac() {
  prefs.begin(PREFS_NS, false);
  prefs.remove(PREFS_KEY);
  prefs.end();
  Serial.println("Saved MAC cleared. Restart the board to enter a new MAC.");
}

// ---------------------------------------------------------------
//  Setup
// ---------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(500);  // give Serial time to settle before printing

  // Motor LEDC setup
  ledcAttachChannel(PIN_L_FWD, PWM_FREQ, PWM_RES, CH_L_FWD);
  ledcAttachChannel(PIN_L_BWD, PWM_FREQ, PWM_RES, CH_L_BWD);
  ledcAttachChannel(PIN_R_FWD, PWM_FREQ, PWM_RES, CH_R_FWD);
  ledcAttachChannel(PIN_R_BWD, PWM_FREQ, PWM_RES, CH_R_BWD);
  stopMotors();

  BLEDevice::init("");

  // --- MAC resolution ---
  String savedMac = loadMac();

  if (savedMac.length() > 0 && isValidMac(savedMac)) {
    // Use the saved MAC
    targetMac = savedMac;
    Serial.println("============================================");
    Serial.println("Loaded saved MAC: " + targetMac);
    Serial.println("Type RESET and press Enter to change it.   ");
    Serial.println("============================================");
  } else {
    // No valid MAC saved - prompt the user
    targetMac = promptAndSaveMac();
  }

  Serial.println("Pairing to: " + targetMac);
  Serial.println("Scanning for NPG Lite...");

  BLEScan* scan = BLEDevice::getScan();
  scan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  scan->setInterval(1349);
  scan->setWindow(449);
  scan->setActiveScan(true);
  scan->start(5, false);
}

// ---------------------------------------------------------------
//  Loop
// ---------------------------------------------------------------
unsigned long lastReconnect = 0;
const unsigned long RECONNECT_INTERVAL = 3000;

void loop() {

  // --- Serial command handler ---
  // Allows "RESET" to clear the stored MAC while running
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.equalsIgnoreCase("RESET")) {
      clearMac();
      // Disconnect if connected
      if (pClient && pClient->isConnected()) pClient->disconnect();
      delay(500);
      ESP.restart();  // restart so promptAndSaveMac() runs on next boot
    }
  }

  // Connect to server when found
  if (doConnect) {
    doConnect = false;
    if (connectToServer()) {
      Serial.println("Connected to NPG Lite");
    } else {
      Serial.println("Connect failed, retrying...");
      doScan = true;
    }
  }

  // Beep once right after connection
  if (justConnected) {
    justConnected = false;
    beepConnect();
  }

  // Re-scan while not connected
  if (!connected && doScan) {
    unsigned long now = millis();
    if (now - lastReconnect >= RECONNECT_INTERVAL) {
      lastReconnect = now;
      BLEDevice::getScan()->start(3, false);
    }
  }

  // Dispatch motor command
  if (newCommand) {
    newCommand = false;
    applyCommand(currentCommand);
  }

  // Watchdog
  if (!connected && !doScan) doScan = true;

  delay(10);
}
