#define CONFIG_ESP32_WIFI_AMPDU_RX_ENABLED 0
#define CONFIG_ESP32_WIFI_AMPDU_TX_ENABLED 0

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
//#define FLED_OUT_PIN 13 //Arduino only FLED is on pin 8
//#define FVCC_OUT_PIN A1 //Arduino only FVCC is on 3.3v pin.

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
uint16_t gRcAil = 0;
uint16_t gRcEle = 0;
uint16_t gRcThr = 0;
uint16_t gRcRud = 0;
uint16_t gRcAux1 = 0;
uint16_t gRcAux2 = 0;
uint16_t gRcAux3 = 0;
uint16_t gRcAux4 = 0;
uint16_t unThrottleIn = 0;
uint16_t unSteeringIn = 0;
uint16_t unThrottleMin = RC_MIN;
uint16_t unThrottleMax = RC_MAX;
uint16_t unThrottleNeu = RC_NEUTRAL;
uint16_t unSteeringMin = RC_MIN;
uint16_t unSteeringMax = RC_MAX;
uint16_t unSteeringNeu = RC_NEUTRAL;

//Assign run or program mode
#define MODE_RUN_WALLIEONLINE 0
#define MODE_PROGRAM_WALLIEONLINE 1
uint8_t gMode = MODE_RUN_WALLIEONLINE;

//Assign mixer values
int gThrottle = 0;
int gSteering = 0;
int gMotorLeft = 0;
int gMotorRight = 0;

//callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&gEspn, incomingData, sizeof(gEspn));
  gGotEspn = true;
  //Serial.print(gEspn.gAIL); Serial.print("  ");
  //Serial.print(gEspn.gELE); Serial.print("  ");
  //Serial.print(gEspn.gTHR); Serial.print("  ");
  //Serial.print(gEspn.gRUD); Serial.println();
}

void setup() {
  Serial.begin(115200);
  ledcSetup(0, 5000, 8); //ESP only freq = 5000 ledChannel = 0 resolution = 8
  ledcSetup(1, 5000, 8); //ESP only freq = 5000 ledChannel = 1 resolution = 8
  ledcAttachPin(PWMA_OUT_PIN, 0); //ESP only ledChannel = 0
  ledcAttachPin(PWMB_OUT_PIN, 1); //ESP only ledChannel = 1
  pinMode(IN2A_OUT_PIN,OUTPUT);
  pinMode(IN1A_OUT_PIN,OUTPUT);
  pinMode(STBY_OUT_PIN,OUTPUT);
  pinMode(IN1B_OUT_PIN,OUTPUT);
  pinMode(IN2B_OUT_PIN,OUTPUT);
  delay(2000); //Wait till boot is complete
  digitalWrite(STBY_OUT_PIN,HIGH); //STBY is a boot selector pin on ESP32 C3 on boot
  
  //Set device as a Wi-Fi Station
  //old //WiFi.enableLongRange(true);
  //old //WiFi.setSleep(false);
  //old //WiFi.mode(WIFI_STA);
  //old //WiFi.disconnect();
  esp_netif_init();
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  esp_wifi_init(&cfg);
  esp_wifi_set_mode(WIFI_MODE_STA);
  esp_wifi_set_bandwidth(WIFI_IF_STA, WIFI_BW_HT20);
  esp_wifi_set_storage(WIFI_STORAGE_RAM);
  esp_wifi_set_ps(WIFI_PS_NONE);
  esp_wifi_start();
  
  //Init ESP-NOW
  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_wifi_config_espnow_rate(WIFI_IF_STA, WIFI_PHY_RATE_24M);
  esp_now_register_recv_cb(OnDataRecv);
}
 
void loop() {
  static uint32_t current_millis;
  static uint32_t last_millis = 0;
  current_millis = millis();
  if (gGotEspn) {
    gGotEspn = false;
    last_millis = current_millis;
    unThrottleIn = gEspn.gELE;
    unSteeringIn = gEspn.gAIL;
    if (gEspn.gAIL) gRcAil = gEspn.gAIL;
    if (gEspn.gELE) gRcEle = gEspn.gELE;
    if (gEspn.gTHR) gRcThr = gEspn.gTHR;
    if (gEspn.gRUD) gRcRud = gEspn.gRUD;
    if (gEspn.gAUX1) gRcAux1 = gEspn.gAUX1;
    if (gEspn.gAUX2) gRcAux2 = gEspn.gAUX2;
    if (gEspn.gAUX3) gRcAux3 = gEspn.gAUX3;
    if (gEspn.gAUX4) gRcAux4 = gEspn.gAUX4;
  } else if (current_millis >= last_millis + 200) {
    unThrottleIn = RC_NEUTRAL;
    unSteeringIn = RC_NEUTRAL;
    gRcAil = RC_NEUTRAL;
    gRcEle = RC_NEUTRAL;
    gRcThr = RC_MIN;
    gRcRud = RC_NEUTRAL;
    gRcAux1 = RC_MIN;
    gRcAux2 = RC_MIN;
    gRcAux3 = RC_MIN;
    gRcAux4 = RC_MIN;
  }
  
  if (gMode == MODE_PROGRAM_WALLIEONLINE) {
    Serial.println("MODE_PROGRAM_WALLIEONLINE");
  }
  
  if (gMode == MODE_RUN_WALLIEONLINE) {
    if (unThrottleIn > unThrottleMin && unThrottleIn < unThrottleMax) {
      gThrottle = map(unThrottleIn,unThrottleMin,unThrottleMax,-255,255);
      gThrottle = constrain(gThrottle,-255,255);
      if (abs(gThrottle) <= RC_DEADBAND) {
        gThrottle = 0;
      }
    }
    if (unSteeringIn > unSteeringMin && unSteeringIn < unSteeringMax) {
      gSteering = map(unSteeringIn,unSteeringMin,unSteeringMax,-255,255);
      gSteering = constrain(gSteering,-255,255);
      if (abs(gSteering) <= RC_DEADBAND) {
        gSteering = 0;
      }
    }
    //Mix gThrottle and gSteering
    gMotorLeft = constrain((gThrottle + gSteering),-255,255);
    gMotorRight = constrain((gThrottle - gSteering),-255,255);
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
     ledcWrite(0, abs(gMotorLeft));
     ledcWrite(1, abs(gMotorRight));
  }
}