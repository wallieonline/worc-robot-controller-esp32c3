/*
 * WORC Robot Controller - ESP32-C3 ESP-NOW Robot Controller
 * Copyright (C) 2026 wallieonline
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <https://www.gnu.org/licenses/>.
 */

#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include <EEPROM.h>

//Assign DRV8833 pins
#define IN2A_OUT_PIN 6
#define IN1A_OUT_PIN 7
#define STBY_OUT_PIN 5
#define IN1B_OUT_PIN 9
#define IN2B_OUT_PIN 10
#define FLED_OUT_PIN 8

//Assign ESPNOW channel struct
typedef struct rc_data_t {
  uint16_t gAIL;
  uint16_t gELE;
  uint16_t gTHR;
  uint16_t gRUD;
  uint16_t gAUX1;
  uint16_t gAUX2;
  uint16_t gAUX3;
  uint16_t gAUX4;
} rc_data_t;
rc_data_t gEspn;
boolean gGotEspn;

//Assign RC values
#define RC_NEUTRAL 1500
#define RC_MIN 960 //Signals can sometimes be lower than 1000
#define RC_MAX 2040 //Signals can sometime be higer than 2000
#define RC_DEADBAND 40
uint16_t unThrottleIn = 0;
uint16_t unSteeringIn = 0;
uint16_t unThrottleMin = RC_MIN;
uint16_t unThrottleMax = RC_MAX;
uint16_t unThrottleNeu = RC_NEUTRAL;
uint16_t unSteeringMin = RC_MIN;
uint16_t unSteeringMax = RC_MAX;
uint16_t unSteeringNeu = RC_NEUTRAL;

//Assign mixer values
int gThrottle = 0;
int gSteering = 0;
int gMotorLeft = 0;
int gMotorRight = 0;

//Settings stored in EEPROM
bool gInvertLeft = false;
bool gInvertRight = false;
bool gMixerOn = true;

//Timing for connection timeout
uint32_t lastCommandTime = 0;
const uint32_t COMMAND_TIMEOUT = 500; //Stop if no command received

void loadSettings() {
  EEPROM.begin(4);
  uint8_t il = EEPROM.read(0);
  uint8_t ir = EEPROM.read(1);
  uint8_t m  = EEPROM.read(2);
  //0xFF means "never written" (fresh flash) - fall back to defaults
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

void updateMotors() {
  if (gMixerOn) {
    //Mix gThrottle and gSteering
    gMotorLeft = constrain((gThrottle + gSteering), -255, 255); //Invert steering by gThrottle + gSteering
    gMotorRight = constrain((gThrottle - gSteering), -255, 255); //Invert steering by gThrottle - gSteering
  } else {
    //Mixer off
    gMotorLeft = constrain(gThrottle, -255, 255);
    gMotorRight = constrain(gSteering, -255, 255);
  }
  //Apply per-channel direction inversion
  if (gInvertLeft) gMotorLeft = -gMotorLeft;
  if (gInvertRight) gMotorRight = -gMotorRight;
  if (gMotorLeft > 0) { //DIRECTION_FORWARD
    ledcWrite(0, LOW); //IN1A_OUT_PIN
    ledcWrite(1, abs(gMotorLeft)); //IN2A_OUT_PIN
  } else if (gMotorLeft < 0) { //DIRECTION_REVERSE
    ledcWrite(0, abs(gMotorLeft)); //IN1A_OUT_PIN
    ledcWrite(1, LOW); //IN2A_OUT_PIN
  } else { //DIRECTION_STOP
    ledcWrite(0, LOW); //IN1A_OUT_PIN
    ledcWrite(1, LOW); //IN2A_OUT_PIN
  }
  if (gMotorRight > 0) { //DIRECTION_FORWARD
    ledcWrite(2, LOW); //IN1B_OUT_PIN
    ledcWrite(3, abs(gMotorRight)); //IN2B_OUT_PIN
  } else if (gMotorRight < 0) { //DIRECTION_REVERSE
    ledcWrite(2, abs(gMotorRight)); //IN1B_OUT_PIN
    ledcWrite(3, LOW); //IN2B_OUT_PIN
  } else { //DIRECTION_STOP
    ledcWrite(2, LOW); //IN1B_OUT_PIN
    ledcWrite(3, LOW); //IN2B_OUT_PIN
  }
}

//callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  if (len == sizeof(gEspn)) {
    memcpy(&gEspn, incomingData, sizeof(gEspn));
    if (gEspn.gELE > unThrottleMin && gEspn.gELE < unThrottleMax) {
      gThrottle = map(gEspn.gELE,unThrottleMin,unThrottleMax,-255,255);
      gThrottle = constrain(gThrottle,-255,255);
      if (abs(gThrottle) <= RC_DEADBAND) {
        gThrottle = 0;
      }
    }
    if (gEspn.gAIL > unSteeringMin && gEspn.gAIL < unSteeringMax) {
      gSteering = map(gEspn.gAIL,unSteeringMin,unSteeringMax,-255,255);
      gSteering = constrain(gSteering,-255,255);
      if (abs(gSteering) <= RC_DEADBAND) {
        gSteering = 0;
      }
    }
    //Serial.print(gEspn.gTHR); Serial.print("  ");
    //Serial.print(gEspn.gRUD); Serial.println();
    lastCommandTime = millis();
  }
}

void setup() {
  Serial.begin(115200);
  loadSettings();
  ledcSetup(0, 5000, 8); //ESP only freq = 5000 ledChannel = 0 resolution = 8
  ledcSetup(1, 5000, 8); //ESP only freq = 5000 ledChannel = 1 resolution = 8
  ledcSetup(2, 5000, 8); //ESP only freq = 5000 ledChannel = 2 resolution = 8
  ledcSetup(3, 5000, 8); //ESP only freq = 5000 ledChannel = 3 resolution = 8
  ledcAttachPin(IN1A_OUT_PIN, 0); //ESP only ledChannel = 0
  ledcAttachPin(IN2A_OUT_PIN, 1); //ESP only ledChannel = 1
  ledcAttachPin(IN1B_OUT_PIN, 2); //ESP only ledChannel = 2
  ledcAttachPin(IN2B_OUT_PIN, 3); //ESP only ledChannel = 3
  pinMode(STBY_OUT_PIN,OUTPUT);
  pinMode(FLED_OUT_PIN,OUTPUT);
  digitalWrite(STBY_OUT_PIN,LOW); //Keep motor driver in standby during boot
  delay(2000); //Wait till boot is complete
  digitalWrite(STBY_OUT_PIN,HIGH);
  //Set device as a Wi-Fi Station (manual init: disables A-MPDU & power-save for lowest ESP-NOW latency)
  //WiFi.enableLongRange(true); //WiFi.setSleep(false); //WiFi.mode(WIFI_STA); //WiFi.disconnect();
  esp_netif_init();
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  cfg.ampdu_tx_enable = 0;
  cfg.ampdu_rx_enable = 0;
  esp_wifi_init(&cfg);
  esp_wifi_set_mode(WIFI_MODE_STA);
  esp_wifi_set_bandwidth(WIFI_IF_STA, WIFI_BW_HT20);
  esp_wifi_set_storage(WIFI_STORAGE_RAM);
  esp_wifi_set_ps(WIFI_PS_NONE);
  esp_wifi_start();
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE); //Lock the channel (1, 6, or 11 are the non-overlapping ones)
  //Init ESP-NOW
  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW, restarting...");
    delay(1000);
    ESP.restart();
  }
  esp_wifi_config_espnow_rate(WIFI_IF_STA, WIFI_PHY_RATE_24M);
  esp_now_register_recv_cb(OnDataRecv);
}
 
void loop() {
  // Check for command timeout (safety feature)
  if (millis() - lastCommandTime > COMMAND_TIMEOUT) {
    if (gThrottle != 0 || gSteering != 0) {
      gThrottle = 0;
      gSteering = 0;
      Serial.println("Command timeout - stopping motors");
    }
    digitalWrite(FLED_OUT_PIN, (millis() / 250) % 2); //Failsafe LED blinking
  } else {
    digitalWrite(FLED_OUT_PIN, LOW); //Link ok LED solid on
  }

  updateMotors();
}