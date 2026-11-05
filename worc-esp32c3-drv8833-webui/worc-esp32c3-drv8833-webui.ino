/*
 * WORC Robot Controller - ESP32-C3 WiFi Robot Controller
 * Copyright (C) 2026 wallieonline (https://github.com/wallieonline)
 *
 * SPDX-License-Identifier: GPL-3.0-only
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 3 as
 * published by the Free Software Foundation.
 *
 * Additional terms under section 7 of the GPL apply (attribution and
 * naming, see the NOTICE file). This copyright notice and the
 * attribution may not be removed.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <https://www.gnu.org/licenses/>.
 */

#include <WiFi.h>
#include <WebServer.h>
#include <WiFiClient.h>
#include <EEPROM.h>
//#include "esp_wifi.h"

// WiFi AP credentials
String ssid;
const char* password = "worc1234";  // Min 8 characters

// Web server
WebServer server(80);

// Minimal WebSocket server on port 81
WiFiServer wsServer(81);
WiFiClient wsClient;
bool wsConnected = false;

// WebSocket magic string for handshake
const char* WS_MAGIC = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";

// Assign DRV8833 pins
#define IN2A_OUT_PIN 6   // Left motor IN2
#define IN1A_OUT_PIN 7   // Left motor IN1
#define STBY_OUT_PIN 5   // Standby pin
#define IN1B_OUT_PIN 9   // Right motor IN1
#define IN2B_OUT_PIN 10  // Right motor IN2
#define FLED_OUT_PIN 8   // Onboard LED

// Assign mixer values
int gThrottle = 0;
int gSteering = 0;
int gMotorLeft = 0;
int gMotorRight = 0;

// Settings stored in EEPROM
bool gInvertLeft = false;
bool gInvertRight = false;
bool gMixerOn = true;

// Timing for connection timeout
uint32_t lastCommandTime = 0;
const uint32_t COMMAND_TIMEOUT = 500; // Stop if no command received

// Timing for telemetry
uint32_t lastTelemetryTime = 0;
const uint32_t TELEMETRY_INTERVAL = 200;

// Forward declarations
void handleRoot();
void handleWebSocket();
void sendWebSocketMessage(const String& msg);
void saveSettings();
String getHTML();

void loadSettings() {
  EEPROM.begin(4);
  uint8_t il = EEPROM.read(0);
  uint8_t ir = EEPROM.read(1);
  uint8_t m  = EEPROM.read(2);
  // 0xFF means "never written" (fresh flash) - fall back to defaults
  gInvertLeft  = (il == 0xFF) ? false : (il != 0);
  gInvertRight = (ir == 0xFF) ? false : (ir != 0);
  gMixerOn     = (m  == 0xFF) ? true  : (m  != 0);
}

void saveSettings() {
  EEPROM.write(0, gInvertLeft ? 1 : 0);
  EEPROM.write(1, gInvertRight ? 1 : 0);
  EEPROM.write(2, gMixerOn ? 1 : 0);
  EEPROM.commit();
}

/*
// Get RSSI (dBm) from first WiFi-client or 0 if none
int8_t getClientRSSI() {
  wifi_sta_list_t staList;
  if (esp_wifi_ap_get_sta_list(&staList) != ESP_OK || staList.num == 0) {
    return 0;
  }
  return staList.sta[0].rssi; // Between -30 (strong) and -90 (weak)
}
*/

void updateMotors() {
  if (gMixerOn) {
    // Mix gThrottle and gSteering
    gMotorLeft = constrain((gThrottle + gSteering), -255, 255);
    gMotorRight = constrain((gThrottle - gSteering), -255, 255);
  } else {
    // Mixer off
    gMotorLeft = constrain(gThrottle, -255, 255);
    gMotorRight = constrain(gSteering, -255, 255);
  }
  // Apply per-channel direction inversion
  if (gInvertLeft) gMotorLeft = -gMotorLeft;
  if (gInvertRight) gMotorRight = -gMotorRight;
  if (gMotorLeft > 0) { // DIRECTION_FORWARD
    ledcWrite(0, LOW);
    ledcWrite(1, abs(gMotorLeft));
  } else if (gMotorLeft < 0) { // DIRECTION_REVERSE
    ledcWrite(0, abs(gMotorLeft));
    ledcWrite(1, LOW);
  } else { // DIRECTION_STOP
    ledcWrite(0, LOW);
    ledcWrite(1, LOW);
  }
  if (gMotorRight > 0) { // DIRECTION_FORWARD
    ledcWrite(2, LOW);
    ledcWrite(3, abs(gMotorRight));
  } else if (gMotorRight < 0) { // DIRECTION_REVERSE
    ledcWrite(2, abs(gMotorRight));
    ledcWrite(3, LOW);
  } else { // DIRECTION_STOP
    ledcWrite(2, LOW);
    ledcWrite(3, LOW);
  }
}

// Disable Nagle/keep-alive so small responses go out immediately
void prepareFastResponse() {
  server.client().setNoDelay(true);
  server.sendHeader("Connection", "close");
}

void handleRoot() {
  prepareFastResponse();
  server.send(200, "text/html", getHTML());
}

void setup() {
  Serial.begin(115200);
  loadSettings();
  ledcSetup(0, 5000, 8); // ESP only freq = 5000 ledChannel = 0 resolution = 8
  ledcSetup(1, 5000, 8); // ESP only freq = 5000 ledChannel = 1 resolution = 8
  ledcSetup(2, 5000, 8); // ESP only freq = 5000 ledChannel = 2 resolution = 8
  ledcSetup(3, 5000, 8); // ESP only freq = 5000 ledChannel = 3 resolution = 8
  ledcAttachPin(IN1A_OUT_PIN, 0); // ESP only ledChannel = 0
  ledcAttachPin(IN2A_OUT_PIN, 1); // ESP only ledChannel = 1
  ledcAttachPin(IN1B_OUT_PIN, 2); // ESP only ledChannel = 2
  ledcAttachPin(IN2B_OUT_PIN, 3); // ESP only ledChannel = 3
  pinMode(STBY_OUT_PIN,OUTPUT);
  pinMode(FLED_OUT_PIN,OUTPUT);
  digitalWrite(STBY_OUT_PIN,LOW);
  delay(2000); // Wait till boot is complete
  digitalWrite(STBY_OUT_PIN,HIGH);
  // Generate ssid from chip-ID
  uint64_t chipid = ESP.getEfuseMac();
  char ssidBuf[32];
  snprintf(ssidBuf, sizeof(ssidBuf), "WORC-%06X", (uint32_t)(chipid >> 24));
  ssid = ssidBuf;
  // Setup WiFi Access Point
  WiFi.mode(WIFI_AP);
  WiFi.setTxPower(WIFI_POWER_13dBm); // Medium power (~20mW)
  WiFi.setSleep(false); // Disable WiFi modem-sleep: much lower & more consistent latency
  WiFi.softAP(ssid.c_str(), password, 6, 0, 4); // Channel 6, not hidden, max 4 connections
  delay(100); // Small delay to ensure AP is broadcasting
  IPAddress IP = WiFi.softAPIP();
  Serial.printf("WiFi AP started\nSSID: %s\nPassword: %s\nIP Address: %s\n", ssid.c_str(), password, IP.toString().c_str());
  // Setup web server
  server.on("/", HTTP_GET, handleRoot);
  server.begin();
  Serial.println("HTTP server started on port 80");
  // Setup WebSocket
  wsServer.begin();
  Serial.println("WebSocket server started on port 81");
}

void loop() {
  // Handle web server and WebSocket
  server.handleClient();
  handleWebSocket();

  // Check for command timeout (safety feature)
  if (millis() - lastCommandTime > COMMAND_TIMEOUT) {
    if (gThrottle != 0 || gSteering != 0) {
      gThrottle = 0;
      gSteering = 0;
      Serial.println("Command timeout - stopping motors");
    }
    digitalWrite(FLED_OUT_PIN, (millis() / 250) % 2); // Failsafe LED blinking
  } else {
    digitalWrite(FLED_OUT_PIN, LOW); // Link ok LED solid on
  }

  /*
  // Send telemetry to the web interface
  if (millis() - lastTelemetryTime > TELEMETRY_INTERVAL) {
    lastTelemetryTime = millis();
    if (wsConnected) {
      sendWebSocketMessage("TLM," + String(gMotorLeft) + "," + gMotorRight + "," + gThrottle + "," + gSteering + "," + String(getClientRSSI()));
    }
  }
  */

  updateMotors();
}