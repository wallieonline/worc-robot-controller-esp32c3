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

//Assign TB6612FNG pins
#define PWMA_OUT_PIN 6
#define IN2A_OUT_PIN 7
#define IN1A_OUT_PIN 8
#define STBY_OUT_PIN 9
#define IN1B_OUT_PIN 10
#define IN2B_OUT_PIN 20
#define PWMB_OUT_PIN 21
#define FLED_OUT_PIN 2 //Free GPIO for status LED

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
volatile rc_data_t gEspn; //Shared with OnDataRecv (WiFi task context)

//Bind mode settings
#define PAIRING_TIMEOUT_MS 30000 //Listen for BIND signal during first 30 seconds after boot
const uint8_t BIND_MSG[] = "WORC";
volatile uint8_t gBoundMac[6]; //Shared with OnDataRecv
volatile bool gBound = false; //Shared with OnDataRecv

//Assign RC values
#define RC_NEUTRAL 1500
//Wider validation range than the nominal 1000-2000 to tolerate overshoot; values outside are ignored
#define RC_MIN 960 //Signals can sometimes be lower than 1000
#define RC_MAX 2040 //Signals can sometime be higer than 2000
#define RC_DEADBAND 40

//Assign calibrated RC values
uint16_t unThrottleMin = RC_MIN;
uint16_t unThrottleMax = RC_MAX;
uint16_t unThrottleNeu = RC_NEUTRAL; //Reserved for future calibration option
uint16_t unSteeringMin = RC_MIN;
uint16_t unSteeringMax = RC_MAX;
uint16_t unSteeringNeu = RC_NEUTRAL; //Reserved for future calibration option

//Assign mixer values
volatile int gThrottle = 0; //Shared with OnDataRecv
volatile int gSteering = 0; //Shared with OnDataRecv
int gMotorLeft = 0;
int gMotorRight = 0;

//Settings stored in EEPROM
bool gInvertLeft = false;
bool gInvertRight = false;
bool gMixerOn = true;

//Timing for connection timeout
volatile uint32_t lastCommandTime = 0; //Shared with OnDataRecv
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
    digitalWrite(IN1A_OUT_PIN,HIGH);
    digitalWrite(IN2A_OUT_PIN,LOW);
  } else if (gMotorLeft < 0) { //DIRECTION_REVERSE
    digitalWrite(IN1A_OUT_PIN,LOW);
    digitalWrite(IN2A_OUT_PIN,HIGH);
  } else { //DIRECTION_STOP
    digitalWrite(IN1A_OUT_PIN,LOW);
    digitalWrite(IN2A_OUT_PIN,LOW);
  }
  if (gMotorRight > 0) { //DIRECTION_FORWARD
    digitalWrite(IN1B_OUT_PIN,HIGH);
    digitalWrite(IN2B_OUT_PIN,LOW);
  } else if (gMotorRight < 0) { //DIRECTION_REVERSE
    digitalWrite(IN1B_OUT_PIN,LOW);
    digitalWrite(IN2B_OUT_PIN,HIGH);
  } else { //DIRECTION_STOP
    digitalWrite(IN1B_OUT_PIN,LOW);
    digitalWrite(IN2B_OUT_PIN,LOW);
  }
  ledcWrite(0, abs(gMotorLeft)); //PWMA_OUT_PIN
  ledcWrite(1, abs(gMotorRight)); //PWMB_OUT_PIN
}

//Pairing mode: listening for a BIND signal (first PAIRING_TIMEOUT_MS after boot, until bound)
bool isPairing(uint32_t now) {
  return !gBound && now < PAIRING_TIMEOUT_MS;
}

//callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  uint32_t current_millis = millis();
  //Bind mode: capture the sender MAC from the callback arguments when the BIND signal is received
  if (isPairing(current_millis)) {
    if (len == sizeof(BIND_MSG) - 1 && memcmp(incomingData, BIND_MSG, sizeof(BIND_MSG) - 1) == 0) {
      memcpy((void *)gBoundMac, mac, 6);
      gBound = true;
      Serial.print("Bound to transmitter MAC: ");
      Serial.printf("%02X:%02X:%02X:%02X:%02X:%02X\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    }
    return; //Ignore all data packets while in bind mode
  }
  //After bind mode: ignore everything except packets from the bound transmitter
  if (!gBound || memcmp(mac, (const void *)gBoundMac, 6) != 0) {
    return;
  }
  if (len == sizeof(gEspn)) {
    memcpy((void *)&gEspn, incomingData, sizeof(gEspn));
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
    lastCommandTime = current_millis;
  }
}

void setup() {
  Serial.begin(115200);
  //loadSettings(); //Disabled: could pick up stale values already in EEPROM on this board
  ledcSetup(0, 5000, 8); //ESP only freq = 5000 ledChannel = 0 resolution = 8
  ledcSetup(1, 5000, 8); //ESP only freq = 5000 ledChannel = 1 resolution = 8
  ledcAttachPin(PWMA_OUT_PIN, 0); //ESP only ledChannel = 0
  ledcAttachPin(PWMB_OUT_PIN, 1); //ESP only ledChannel = 1
  pinMode(IN2A_OUT_PIN,OUTPUT);
  pinMode(IN1A_OUT_PIN,OUTPUT);
  pinMode(STBY_OUT_PIN,OUTPUT);
  pinMode(IN1B_OUT_PIN,OUTPUT);
  pinMode(IN2B_OUT_PIN,OUTPUT);
  pinMode(FLED_OUT_PIN,OUTPUT);
  digitalWrite(STBY_OUT_PIN,LOW); //Keep motor driver in standby during boot
  delay(2000); //Wait till boot is complete
  digitalWrite(STBY_OUT_PIN,HIGH);
  //Set device as a Wi-Fi Station (manual init: disables A-MPDU & power-save for lowest ESP-NOW latency)
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
  WiFi.setTxPower(WIFI_POWER_17dBm); //Medium power (~50mW)
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE); //Lock the channel (1, 6, or 11 are the non-overlapping ones)
  //Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW, restarting...");
    delay(1000);
    ESP.restart();
  }
  esp_wifi_config_espnow_rate(WIFI_IF_STA, WIFI_PHY_RATE_24M);
  esp_now_register_recv_cb(OnDataRecv);
}

void loop() {
  static uint32_t current_millis;
  current_millis = millis();
  if (isPairing(current_millis)) {
    digitalWrite(FLED_OUT_PIN, (current_millis / 50) % 2); //Blink fast while pairing
  } else if (current_millis - lastCommandTime > COMMAND_TIMEOUT) { //Command timeout (safety feature)
    if (gThrottle != 0 || gSteering != 0) {
      gThrottle = 0;
      gSteering = 0;
      Serial.println("Command timeout - stopping motors");
    }
    digitalWrite(FLED_OUT_PIN, (current_millis / 250) % 2); //Failsafe LED blinking
  } else {
    digitalWrite(FLED_OUT_PIN, LOW); //Link ok LED solid on
  }

  updateMotors();
}