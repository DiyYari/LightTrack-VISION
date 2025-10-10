#include <Arduino.h>
#include <LD2450.h>
#include <FastLED.h>
#include <EEPROM.h>
#include <SPIFFS.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>
#include <time.h>
#include <ArduinoOTA.h>
#include <stdlib.h>
#include "esp_wifi.h"
#include <math.h>
#include <utility> // For std::swap

// --- PIN CONFIGURATION ---
const int RX1_PIN = 8;
const int TX1_PIN = 9;
const int LED_PIN = 7;

// --- LED STRIP CONFIGURATION ---
#define MAX_LED_DENSITY 60
#define STRIP_LENGTH    10
#define MAX_NUM_LEDS    (MAX_LED_DENSITY * STRIP_LENGTH)
#define LED_TYPE        WS2812B
#define COLOR_ORDER     GRB
CRGB leds[MAX_NUM_LEDS];

// --- CURRENT STRIP VARIABLES ---
int currentLedDensity = 60;
int currentNumLeds = (currentLedDensity * STRIP_LENGTH);

// --- RADAR CONFIGURATION ---
LD2450 ld2450;

// --- TARGET TRACKING CONFIGURATION ---
#define MAX_TARGETS 1
#define TARGET_TIMEOUT 2000
#define GHOST_FILTER_TIMEOUT 5000 
#define GHOST_REMOVAL_TIMEOUT 10000 
#define MERGE_DISTANCE 500 

// --- RADAR PROCESSING RANGE ---
#define MIN_PROCESS_DISTANCE  150
#define MAX_PROCESS_DISTANCE  8000

// --- CALIBRATION TABLES ---
#define CALIBRATION_POINTS 16
const int calibration30[CALIBRATION_POINTS][2] = {
  {150,   5}, {250,   8}, {500,  15}, {750,  23}, {1000,  30}, {1500,  45},
  {2000,  60}, {2500,  75}, {3000,  90}, {4000, 120}, {5000, 150}, {6000, 180},
  {7000, 210}, {8000, 240}, {8500, 255}, {9000, 270}
};
const int calibration60[CALIBRATION_POINTS][2] = {
  {150,   10}, {250,  16}, {500,  30}, {750,  46}, {1000,  60}, {1500,  90},
  {2000, 120}, {2500, 150}, {3000, 180}, {4000, 240}, {5000, 300}, {6000, 360},
  {7000, 420}, {8000, 480}, {8500, 540}, {9000, 599}
};

// --- INACTIVE ZONES ---
#define MAX_INACTIVE_ZONES 4
struct InactiveZone {
  bool enabled;
  int minX; int minY; int maxX; int maxY;
};
InactiveZone inactiveZones[MAX_INACTIVE_ZONES];

// --- Individual Target Settings Structure ---
struct TargetSetting {
  bool enabled;
  CRGB color;
};

// --- GLOBAL SETTINGS ---
int ledUpdateInterval = 20;
int movementSensitivity = 1;
int wifiTimeoutMinutes = 7;
float movingIntensity = 0.5f;
float stationaryIntensity = 0.04f;
int movingLength = 20;
int gradientSoftness = 4;
int ledOffDelay = 5;
int centerShift = 0;
int additionalLEDs = 75;
bool backgroundModeActive = false;
int maxActiveTargets = 1;
TargetSetting targetSettings[MAX_TARGETS] = {
  {true, CRGB(255, 200, 50)}
};

// --- EEPROM ---
#define EEPROM_SIZE (1024 + sizeof(int))
#define EEPROM_VERSION 0xAD // Новая версия для сброса настроек

// --- TARGET STRUCTURE with Kalman Filter variables ---
struct Target {
  bool present;
  int x, y, distance, speed;
  unsigned long lastSeenTime, lastMovementTime;
  float smoothedLEDPosition, velocityLED;
  float p_pos, p_vel;
  float currentBrightness;
  int lastMovementDirection;
  bool isInitialized;
};
Target targets[MAX_TARGETS];

// --- WORKING VARIABLES ---
bool stripInitialized = false;
int startHour = 20, startMinute = 0;
int endHour = 8, endMinute = 0;
bool lightOn = true;
unsigned long lastTimeCheck = 0;
volatile bool smarthomeOverride = false;
volatile int clientTimezoneOffsetMinutes = 0;
volatile bool isTimeOffsetSet = false;
WebServer server(80);
WebSocketsServer webSocket = WebSocketsServer(81);
TaskHandle_t sensorTaskHandle = NULL, ledTaskHandle = NULL, webServerTaskHandle = NULL;
unsigned long wifiStartTime = 0;
bool wifiActive = true;

// --- FUNCTION PROTOTYPES ---
void sensorTask(void* parameter); void ledTask(void* parameter); void webServerTask(void* parameter);
void loadSettings(); void saveSettings(); bool initializeStrip(); int mapDistanceToLED(int rawDistance);
bool isTargetInInactiveZone(int targetX, int targetY); void setupWiFi(); void setupOTA();
void handleRoot(); void handleSetMovingIntensity(); void handleSetStationaryIntensity();
void handleSetMovingLength(); void handleSetAdditionalLEDs(); void handleSetCenterShift();
void handleSetLedDensity(); void handleSetGradientSoftness(); void handleSetLedOffDelay();
void handleSetTime(); void handleSetSchedule(); void handleNotFound(); void handleSmartHomeOn();
void handleSmartHomeOff(); void handleSmartHomeClear(); void handleToggleBackgroundMode();
void handleGetCurrentTime(); void handleRadarView(); void webSocketEvent(uint8_t, WStype_t, uint8_t*, size_t);
void broadcastRadarData(); void updateTime();
void handleSetTargetSettings();
void checkWiFiTimeout();

// ------------------------- Setup -------------------------
void setup() {
  setCpuFrequencyMhz(80);
  
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n\n--- setup() begins ---");
  Serial.println("LightTrack VISION (Final UI - Single Target)");
  Serial.printf("CPU Frequency: %d MHz\n", getCpuFrequencyMhz());
  Serial.println("=======================================================");

  randomSeed(ESP.getEfuseMac());

  Serial.println("Initializing Target variables (with Kalman Filter)...");
  for (auto & target : targets) {
    target.present = false;
    target.isInitialized = false;
    target.smoothedLEDPosition = 0;
    target.velocityLED = 0.0f;
    target.p_pos = 1000.0f;
    target.p_vel = 1000.0f;
    target.lastMovementTime = 0;
  }

  Serial.println("Setting all INACTIVE X/Y zones to DISABLED by default...");
  for (auto & zone : inactiveZones) {
    zone.enabled = false;
    zone.minX = 0; zone.maxX = 0; zone.minY = 0; zone.maxY = 0;
  }
  
  if (!SPIFFS.begin(true)) { Serial.println("!!! SPIFFS mount failed"); }

  Serial.println("--- Calling loadSettings() ---");
  loadSettings();

  Serial.println("Initializing LD2450 Radar...");
  Serial1.begin(LD2450_SERIAL_SPEED, SERIAL_8N1, RX1_PIN, TX1_PIN);
  delay(500); ld2450.begin(Serial1); delay(100);
  Serial.println("Radar initialized.");

  setupWiFi();
  wifiStartTime = millis();
  configTzTime("UTC0", "pool.ntp.org", "time.nist.gov");
  setupOTA();
  stripInitialized = initializeStrip();

  Serial.println("Starting RTOS tasks...");
  xTaskCreatePinnedToCore(sensorTask, "SensorTask", 4096, NULL, 5, &sensorTaskHandle, 0);
  xTaskCreatePinnedToCore(ledTask, "LEDTask", 8192, NULL, 4, &ledTaskHandle, 1);
  xTaskCreatePinnedToCore(webServerTask, "WebServerTask", 6144, NULL, 3, &webServerTaskHandle, 0);

  Serial.println("--------------------------------------");
  Serial.println("Setup complete. System is running.");
  Serial.print("Web UI: http://"); Serial.println(WiFi.softAPIP());
  Serial.print("Radar View: http://"); Serial.print(WiFi.softAPIP()); Serial.println("/radarview");
  Serial.printf("Wi-Fi will turn off automatically in %d minutes.\n", wifiTimeoutMinutes);
  Serial.println("--------------------------------------");
}

void loop() {
  updateTime();
  checkWiFiTimeout();
  vTaskDelay(pdMS_TO_TICKS(5000));
}

// --- CORE LOGIC FUNCTIONS ---
int mapDistanceToLED(int rawDistance) {
  const int (*selectedCalibration)[2] = (currentLedDensity == 60) ? calibration60 : calibration30;
  rawDistance = constrain(rawDistance, MIN_PROCESS_DISTANCE, MAX_PROCESS_DISTANCE);
  for (int i = 0; i < CALIBRATION_POINTS - 1; i++) {
    if (rawDistance >= selectedCalibration[i][0] && rawDistance <= selectedCalibration[i+1][0]) {
      if (selectedCalibration[i+1][0] == selectedCalibration[i][0]) return selectedCalibration[i][1];
      float proportion = (float)(rawDistance - selectedCalibration[i][0]) / (float)(selectedCalibration[i+1][0] - selectedCalibration[i][0]);
      return constrain(selectedCalibration[i][1] + round(proportion * (selectedCalibration[i+1][1] - selectedCalibration[i][1])), 0, currentNumLeds - 1);
    }
  }
  if (rawDistance >= selectedCalibration[CALIBRATION_POINTS-1][0]) return constrain(selectedCalibration[CALIBRATION_POINTS-1][1], 0, currentNumLeds - 1);
  return constrain(selectedCalibration[0][1], 0, currentNumLeds - 1);
}

bool isTargetInInactiveZone(int targetX, int targetY) {
    for (int i = 0; i < MAX_INACTIVE_ZONES; ++i) {
        if (inactiveZones[i].enabled && targetX >= inactiveZones[i].minX && targetX <= inactiveZones[i].maxX && targetY >= inactiveZones[i].minY && targetY <= inactiveZones[i].maxY) {
            return true;
        }
    }
    return false;
}

void loadSettings() {
  Serial.println("\n--- loadSettings() begins ---");
  EEPROM.begin(EEPROM_SIZE);
  int addr = 0;
  byte checkVal;
  EEPROM.get(addr, checkVal);
  addr += sizeof(checkVal); 
  if (checkVal != EEPROM_VERSION) { 
      Serial.println("!!! EEPROM version mismatch or invalid. Resetting to default values.");
      EEPROM.end();
      saveSettings(); // Сохраняем дефолтные значения с новой версией
      return;
  }
  
  EEPROM.get(addr, ledOffDelay); addr += sizeof(ledOffDelay);
  EEPROM.get(addr, movingIntensity); addr += sizeof(movingIntensity);
  EEPROM.get(addr, stationaryIntensity); addr += sizeof(stationaryIntensity);
  EEPROM.get(addr, movingLength); addr += sizeof(movingLength);
  EEPROM.get(addr, centerShift); addr += sizeof(centerShift);
  EEPROM.get(addr, additionalLEDs); addr += sizeof(additionalLEDs);
  EEPROM.get(addr, targetSettings[0].color); addr += sizeof(targetSettings[0].color);
  EEPROM.get(addr, gradientSoftness); addr += sizeof(gradientSoftness);
  EEPROM.get(addr, startHour); addr += sizeof(startHour);
  EEPROM.get(addr, startMinute); addr += sizeof(startMinute);
  EEPROM.get(addr, endHour); addr += sizeof(endHour);
  EEPROM.get(addr, endMinute); addr += sizeof(endMinute);
  int tempTz; EEPROM.get(addr, tempTz); clientTimezoneOffsetMinutes = tempTz; addr += sizeof(clientTimezoneOffsetMinutes);
  EEPROM.get(addr, backgroundModeActive); addr += sizeof(backgroundModeActive);
  EEPROM.get(addr, currentLedDensity); addr += sizeof(currentLedDensity);
  
  currentNumLeds = constrain(currentLedDensity * STRIP_LENGTH, 1, MAX_NUM_LEDS);

  int zonesRead = 0;
  if (addr + MAX_INACTIVE_ZONES * sizeof(InactiveZone) <= EEPROM_SIZE) {
      for (int i = 0; i < MAX_INACTIVE_ZONES; i++) {
          InactiveZone tempZone;
          EEPROM.get(addr, tempZone);
          if (abs(tempZone.minX)<30000 && abs(tempZone.maxX)<30000 && abs(tempZone.minY)<30000 && abs(tempZone.maxY)<30000) {
              inactiveZones[i] = tempZone;
              if (inactiveZones[i].minX > inactiveZones[i].maxX) std::swap(inactiveZones[i].minX, inactiveZones[i].maxX);
              if (inactiveZones[i].minY > inactiveZones[i].maxY) std::swap(inactiveZones[i].minY, inactiveZones[i].maxY);
              if(inactiveZones[i].enabled) zonesRead++;
          }
          addr += sizeof(InactiveZone);
      }
      if(zonesRead > 0) Serial.printf("Loaded %d enabled INACTIVE X/Y zones from EEPROM.\n", zonesRead);
  }
  EEPROM.end();
}

void saveSettings() {
  Serial.println("\n--- saveSettings() begins ---");
  EEPROM.begin(EEPROM_SIZE);
  int addr = 0;
  byte checkVal = EEPROM_VERSION; // Сохраняем текущую версию
  EEPROM.put(addr, checkVal); addr += sizeof(checkVal);
  
  EEPROM.put(addr, ledOffDelay); addr += sizeof(ledOffDelay);
  EEPROM.put(addr, movingIntensity); addr += sizeof(movingIntensity);
  EEPROM.put(addr, stationaryIntensity); addr += sizeof(stationaryIntensity);
  EEPROM.put(addr, movingLength); addr += sizeof(movingLength);
  EEPROM.put(addr, centerShift); addr += sizeof(centerShift);
  EEPROM.put(addr, additionalLEDs); addr += sizeof(additionalLEDs);
  EEPROM.put(addr, targetSettings[0].color); addr += sizeof(targetSettings[0].color);
  EEPROM.put(addr, gradientSoftness); addr += sizeof(gradientSoftness);
  EEPROM.put(addr, startHour); addr += sizeof(startHour);
  EEPROM.put(addr, startMinute); addr += sizeof(startMinute);
  EEPROM.put(addr, endHour); addr += sizeof(endHour);
  EEPROM.put(addr, endMinute); addr += sizeof(endMinute);
  int tempTz = clientTimezoneOffsetMinutes; EEPROM.put(addr, tempTz); addr += sizeof(clientTimezoneOffsetMinutes);
  EEPROM.put(addr, backgroundModeActive); addr += sizeof(backgroundModeActive);
  EEPROM.put(addr, currentLedDensity); addr += sizeof(currentLedDensity);

  if (addr + MAX_INACTIVE_ZONES * sizeof(InactiveZone) <= EEPROM_SIZE) {
      for (int i = 0; i < MAX_INACTIVE_ZONES; i++) {
           EEPROM.put(addr, inactiveZones[i]);
           addr += sizeof(InactiveZone);
      }
  }
  
  boolean result = EEPROM.commit();
  EEPROM.end();
  Serial.print("EEPROM save finished. Result: "); Serial.println(result ? "OK" : "ERROR");
  Serial.printf("Total EEPROM bytes used: %d / %d\n", addr, EEPROM_SIZE);
}

bool initializeStrip() {
  Serial.println("Initializing LED Strip (FastLED)...");
  try {
      FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, MAX_NUM_LEDS).setCorrection(TypicalLEDStrip);
      FastLED.setBrightness(80);
      FastLED.clear(true);
      return true;
  } catch (...) {
      Serial.println("!!! CRITICAL ERROR in FastLED.addLeds()!");
      return false;
  }
}

// --- RTOS TASKS ---
void sensorTask(void * parameter) {
  Serial.println("Sensor Task started.");
  bool targetFoundThisCycle[MAX_TARGETS];
  for (;;) {
    if (ld2450.read() > 0) {
        unsigned long currentMillis = millis();
        for (int i=0; i<MAX_TARGETS; i++) targetFoundThisCycle[i] = false;
        
        int supportedTargets = min((int)ld2450.getSensorSupportedTargetCount(), maxActiveTargets);
        for (int i = 0; i < supportedTargets; i++) {
            LD2450::RadarTarget radarTarget = ld2450.getTarget(i);
            if (radarTarget.valid) {
                
                if (targets[i].present && (millis() - targets[i].lastMovementTime > GHOST_REMOVAL_TIMEOUT)) {
                    Serial.printf("Ghost target %d detected and ignored.\n", i);
                    continue; 
                }
                
                targetFoundThisCycle[i] = true;
                if (!targets[i].present) { 
                    targets[i].lastMovementTime = currentMillis; 
                }
                targets[i].x = radarTarget.x;
                targets[i].y = radarTarget.y;
                targets[i].distance = radarTarget.distance;
                targets[i].speed = radarTarget.speed;
                targets[i].present = true;
                targets[i].lastSeenTime = currentMillis;

                if (abs(radarTarget.speed) >= movementSensitivity) {
                    targets[i].lastMovementTime = currentMillis;
                    if (radarTarget.speed > movementSensitivity) targets[i].lastMovementDirection = 1;
                    else if (radarTarget.speed < -movementSensitivity) targets[i].lastMovementDirection = -1;
                }
            }
        }

        for (int i=0; i<MAX_TARGETS; i++) {
            if (targets[i].present && !targetFoundThisCycle[i]) { 
                targets[i].present = false;
            }
        }
    }
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

void ledTask(void * parameter) {
  Serial.println("LED Task (Kalman Filter Core) is starting...");
  unsigned long lastCycleTimeMicros = micros();
  while (true) {
    unsigned long currentMicros = micros();
    float deltaTime = (currentMicros - lastCycleTimeMicros) / 1000000.0f;
    lastCycleTimeMicros = currentMicros;
    if (deltaTime <= 0.0f || deltaTime > 0.1f) deltaTime = (float)ledUpdateInterval / 1000.0f;

    if (!lightOn && !smarthomeOverride) {
        fill_solid(leds, MAX_NUM_LEDS, CRGB::Black);
        FastLED.show();
        vTaskDelay(pdMS_TO_TICKS(ledUpdateInterval * 2)); continue;
    }

    if (backgroundModeActive) {
        CRGB bgColor = targetSettings[0].color;
        bgColor.nscale8_video(stationaryIntensity * 255);
        fill_solid(leds, currentNumLeds, bgColor);
    } else {
        fill_solid(leds, currentNumLeds, CRGB::Black);
    }
    if (currentNumLeds < MAX_NUM_LEDS) {
        fill_solid(leds + currentNumLeds, MAX_NUM_LEDS - currentNumLeds, CRGB::Black);
    }

    Target& target = targets[0]; // Работаем только с первой целью
      
    const float q_pos = 1.0f * deltaTime;
    const float q_vel = 5.0f * deltaTime;
    const float r_measure = 10.0f;
    
    if (target.present) {
        float measuredPosition = (float)mapDistanceToLED(target.distance);

        if (!target.isInitialized) {
            target.smoothedLEDPosition = measuredPosition;
            target.velocityLED = 0;
            target.p_pos = 1000.0f; 
            target.p_vel = 1000.0f; 
            target.isInitialized = true;
        }
        
        float predicted_pos = target.smoothedLEDPosition + target.velocityLED * deltaTime;
        target.p_pos += q_pos;
        target.p_vel += q_vel;
        
        float innovation = measuredPosition - predicted_pos;
        float s_inv = 1.0f / (target.p_pos + r_measure);
        
        float k_pos = target.p_pos * s_inv;
        float k_vel = target.p_vel * s_inv;

        target.smoothedLEDPosition = predicted_pos + k_pos * innovation;
        target.velocityLED += k_vel * innovation;

        target.p_pos *= (1.0f - k_pos);
        target.p_vel *= (1.0f - k_pos);

    } else if (target.isInitialized) { 
        // ЗАМОРАЖИВАЕМ СВЕТ, А НЕ ДВИГАЕМ ЕГО ДАЛЬШЕ
        
        float decayRate = 15.0f; 
        target.velocityLED *= (1.0f - constrain(decayRate * deltaTime, 0.0f, 1.0f));
        
        target.p_pos = min(target.p_pos * 1.5f, 1000.0f);
        target.p_vel = min(target.p_vel * 1.5f, 1000.0f);

        if (millis() - target.lastSeenTime > ((unsigned long)ledOffDelay * 1000 + 500)) {
            target.isInitialized = false;
        }
    }

    float maxVel = 500.0f * ((float)currentNumLeds / MAX_NUM_LEDS);
    target.velocityLED = constrain(target.velocityLED, -maxVel, maxVel);
    target.smoothedLEDPosition = constrain(target.smoothedLEDPosition, -1.0f, (float)currentNumLeds);

    // --- ЛОГИКА ОПРЕДЕЛЕНИЯ ЯРКОСТИ ---
    float targetBrightness = 0.0f;
    bool shouldBeLit = false;
    if (target.isInitialized) {
        if (target.present) {
            shouldBeLit = true;
        } else if (ledOffDelay > 0) {
            shouldBeLit = (millis() - target.lastSeenTime < (unsigned long)ledOffDelay * 1000);
        }
    }
    
    if (shouldBeLit) {
      if (!target.present || !isTargetInInactiveZone(target.x, target.y)) {
        bool isMoving = target.present && (abs(target.speed) >= movementSensitivity);
        
        if (isMoving) {
            targetBrightness = movingIntensity;
        } else { 
            if (!backgroundModeActive) {
                targetBrightness = movingIntensity; 
            } else { 
                if (target.present && (millis() - target.lastMovementTime < GHOST_FILTER_TIMEOUT)) {
                    targetBrightness = stationaryIntensity;
                } else if (!target.present) {
                    targetBrightness = movingIntensity;
                }
            }
        }
      }
    }
    
    if (ledOffDelay == 0 && !target.present) {
        target.currentBrightness = 0.0f;
    }
    
    float fadeRate = (target.currentBrightness < targetBrightness) ? 8.0f : 4.0f;
    target.currentBrightness += (targetBrightness - target.currentBrightness) * constrain(fadeRate * deltaTime, 0.0f, 1.0f);
    if (target.currentBrightness < 0.005f) target.currentBrightness = 0.0f;

    if (target.currentBrightness > 0.005f && target.smoothedLEDPosition >= 0) {
      int centerLED = round(target.smoothedLEDPosition) + centerShift;

      int direction = target.lastMovementDirection;
      if (abs(target.velocityLED) > 2.0f) { 
          direction = (target.velocityLED > 0) ? 1 : -1;
      }
      if (direction == 0) direction = 1;
      
      int leftEdge, rightEdge;
      if (direction > 0) {
          leftEdge = centerLED - (movingLength / 2);
          rightEdge = leftEdge + movingLength - 1 + additionalLEDs; 
      } else {
          rightEdge = centerLED + (movingLength / 2);
          leftEdge = rightEdge - (movingLength - 1) - additionalLEDs;
      }

      int totalLightLength = rightEdge - leftEdge + 1;
      if (totalLightLength <= 0) totalLightLength = 1;
      int fadeWidth = map(gradientSoftness, 0, 10, 0, max(1, totalLightLength / 3));
      float fadeExp = 1.0 + (gradientSoftness / 10.0) * 1.5;

      for (int i = max(0, leftEdge); i <= min(currentNumLeds - 1, rightEdge); ++i) {
          int pos = (direction > 0) ? (i - leftEdge) : (rightEdge - i);
          float intensity = 1.0f;
          if (gradientSoftness > 0 && fadeWidth > 0 && totalLightLength > 1) {
              if (pos < fadeWidth) intensity = powf((float)pos / fadeWidth, fadeExp);
              else if (pos >= (totalLightLength - fadeWidth)) intensity = powf((float)(totalLightLength - 1 - pos) / fadeWidth, fadeExp);
          }
          if (intensity > 0.01f) {
              CRGB drawColor = targetSettings[0].color;
              drawColor.nscale8_video(scale8(target.currentBrightness * 255, (uint8_t)(intensity * 255)));
              leds[i] += drawColor;
          }
      }
    }

    FastLED.show();
    long delayMicros = (ledUpdateInterval * 1000) - (micros() - currentMicros);
    vTaskDelay(pdMS_TO_TICKS(max(0L, delayMicros/1000)));
  }
}


// --- WEB & NETWORK FUNCTIONS ---
void setupWiFi() {
  WiFi.mode(WIFI_AP);
  Serial.println("Setting Wi-Fi TX power to minimum (2dBm)...");
  esp_wifi_set_max_tx_power(8); 

  IPAddress local_IP(192, 168, 4, 1);
  WiFi.softAPConfig(local_IP, local_IP, IPAddress(255, 255, 255, 0));
  
  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_WIFI_SOFTAP);
  char deviceName[25];
  sprintf(deviceName, "LightTrack-VISION-%02X%02X%02X", mac[3], mac[4], mac[5]);
  WiFi.softAP(deviceName, "12345678");

  server.on("/", HTTP_GET, handleRoot);
  server.on("/setMovingIntensity", HTTP_GET, handleSetMovingIntensity);
  server.on("/setStationaryIntensity", HTTP_GET, handleSetStationaryIntensity);
  server.on("/setMovingLength", HTTP_GET, handleSetMovingLength);
  server.on("/setAdditionalLEDs", HTTP_GET, handleSetAdditionalLEDs);
  server.on("/setCenterShift", HTTP_GET, handleSetCenterShift);
  server.on("/setLedDensity", HTTP_GET, handleSetLedDensity);
  server.on("/setGradientSoftness", HTTP_GET, handleSetGradientSoftness);
  server.on("/setLedOffDelay", HTTP_GET, handleSetLedOffDelay);
  server.on("/setTime", HTTP_GET, handleSetTime);
  server.on("/setSchedule", HTTP_GET, handleSetSchedule);
  server.on("/smarthome/on", HTTP_GET, handleSmartHomeOn);
  server.on("/smarthome/off", HTTP_GET, handleSmartHomeOff);
  server.on("/smarthome/clear", HTTP_GET, handleSmartHomeClear);
  server.on("/toggleBackgroundMode", HTTP_GET, handleToggleBackgroundMode);
  server.on("/getCurrentTime", HTTP_GET, handleGetCurrentTime);
  server.on("/radarview", HTTP_GET, handleRadarView);
  server.on("/setTarget", HTTP_GET, handleSetTargetSettings);

  server.onNotFound(handleNotFound);

  server.begin();
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
}

void checkWiFiTimeout() {
    if (wifiActive && wifiTimeoutMinutes > 0) {
        if (millis() - wifiStartTime > (unsigned long)wifiTimeoutMinutes * 60 * 1000) {
            Serial.println("Wi-Fi timeout reached. Turning off Wi-Fi.");
            WiFi.mode(WIFI_OFF);
            wifiActive = false;
        }
    }
}

void webServerTask(void * parameter) {
  Serial.println("Web Server Task started.");
  unsigned long lastRadarBroadcast = 0;
  for (;;) {
    if (wifiActive) {
        server.handleClient();
        webSocket.loop();
        ArduinoOTA.handle();
        if (webSocket.connectedClients() > 0 && (millis() - lastRadarBroadcast > 100)) {
            broadcastRadarData();
            lastRadarBroadcast = millis();
        }
    }
    vTaskDelay(pdMS_TO_TICKS(2));
  }
}
void setupOTA() {
  ArduinoOTA.onStart([]() { Serial.println("OTA Start"); });
  ArduinoOTA.onEnd([]() { Serial.println("\nOTA End"); });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) { Serial.printf("Progress: %u%%\r", (progress / (total / 100))); });
  ArduinoOTA.onError([](ota_error_t error) { Serial.printf("OTA Error[%u]\n", error); });
  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_WIFI_SOFTAP);
  char deviceName[25];
  sprintf(deviceName, "LightTrack-VISION-%02X%02X%02X", mac[3], mac[4], mac[5]);
  ArduinoOTA.setHostname(deviceName);
  ArduinoOTA.begin();
}
void handleSetTime() {
  if (server.hasArg("tz")) {
    clientTimezoneOffsetMinutes = server.arg("tz").toInt();
    isTimeOffsetSet = true;
  }
  if (server.hasArg("epoch")) {
    time_t epoch = strtoul(server.arg("epoch").c_str(), NULL, 10);
    struct timeval tv = { .tv_sec = epoch, .tv_usec = 0 };
    settimeofday(&tv, NULL);
  }
  updateTime();
  server.send(200, "text/plain", "OK");
}
void updateTime() {
  unsigned long currentMillis = millis();
  if (currentMillis - lastTimeCheck < 1000) return;
  lastTimeCheck = currentMillis;
  time_t nowUtc = time(nullptr);
  if (nowUtc < 1609459200UL || !isTimeOffsetSet) return;

  time_t clientLocalEpoch = nowUtc + (clientTimezoneOffsetMinutes * 60);
  struct tm timeinfo_local; gmtime_r(&clientLocalEpoch, &timeinfo_local);
  int currentTotalMinutes = timeinfo_local.tm_hour * 60 + timeinfo_local.tm_min;
  int startTotalMinutes = startHour * 60 + startMinute;
  int endTotalMinutes = endHour * 60 + endMinute;
  
  bool shouldBeOn = (startTotalMinutes <= endTotalMinutes) ? 
                    (currentTotalMinutes >= startTotalMinutes && currentTotalMinutes < endTotalMinutes) :
                    (currentTotalMinutes >= startTotalMinutes || currentTotalMinutes < endTotalMinutes);

  if (!smarthomeOverride && (lightOn != shouldBeOn)) {
      lightOn = shouldBeOn;
      Serial.printf("Schedule changed light to: %s\n", lightOn ? "ON" : "OFF");
  }
}
void handleGetCurrentTime() {
  char timeStr[20] = "N/A";
  time_t nowUtc = time(nullptr);
  if (nowUtc > 1609459200UL && isTimeOffsetSet) {
      time_t localEpoch = nowUtc + (clientTimezoneOffsetMinutes * 60);
      strftime(timeStr, sizeof(timeStr), "%H:%M:%S", gmtime(&localEpoch));
  } else if (nowUtc > 1609459200UL) { strcpy(timeStr, "No TZ"); }
  else { strcpy(timeStr, "Syncing..."); }
  server.send(200, "application/json", String("{\"time\":\"") + timeStr + "\"}");
}

void handleRoot() {
  char scheduleStartStr[6], scheduleEndStr[6];
  sprintf(scheduleStartStr, "%02d:%02d", startHour, startMinute);
  sprintf(scheduleEndStr, "%02d:%02d", endHour, endMinute);

  String html = "";
  html.reserve(7000);
  
  char colorStr[8];
  sprintf(colorStr, "#%02x%02x%02x", targetSettings[0].color.r, targetSettings[0].color.g, targetSettings[0].color.b);

  html += R"rawliteral(
<!DOCTYPE html><html><head><title>LightTrack VISION</title>
<meta name='viewport' content='width=device-width, initial-scale=1, maximum-scale=1, user-scalable=no'>
<meta charset='UTF-8'>
<style>
body{background-color:#282c34;color:#abb2bf;font-family:sans-serif;margin:0;padding:15px}
.container{max-width:700px;margin:auto;background-color:#3a3f4b;padding:20px;border-radius:8px;box-shadow:0 4px 8px #00000033}
h1,h2{color:#61afef;text-align:center;border-bottom:1px solid #4b5263;padding-bottom:10px;margin-top:0}
h2{margin-top:25px;border-bottom:none}
label{display:block;margin-top:15px;margin-bottom:5px;color:#98c379;font-weight:bold}
.toggle-label{display:flex;align-items:center;justify-content:space-between;cursor:pointer;}
input[type=color]{width:40px;height:40px;border:none;border-radius:5px;padding:0;cursor:pointer;background-color:transparent;vertical-align:middle;margin-left:10px}
input[type=time]{font-size:1em;padding:5px;border-radius:4px;border:1px solid #4b5263;background-color:#282c34;color:#abb2bf;margin:0 5px}
input[type=radio]{margin:0 5px 0 10px;vertical-align:middle;transform:scale(1.2)}
button,a.button-link{display:inline-block;text-decoration:none;font-size:1em;margin:10px 5px;padding:10px 15px;border:none;border-radius:5px;background-color:#61afef;color:#282c34!important;cursor:pointer;transition:background-color .2s;text-align:center}
button:hover,a.button-link:hover{background-color:#5295cc}
.button-off{background-color:#e06c75}.button-off:hover{background-color:#be5046}
.button-nav{background-color:#c678dd}.button-nav:hover{background-color:#a968bd}
hr{border:none;height:1px;background:#4b5263;margin:25px 0}
.value-display{color:#e5c07b;font-weight:normal;display:inline}
.time-container{display:flex;justify-content:center;align-items:center;gap:15px;margin-top:5px;flex-wrap:wrap}
.current-time{font-size:.9em;color:#5c6370;text-align:center;margin-top:15px}
.radio-group label{display:inline;font-weight:normal;color:#abb2bf}
.footer{font-size:.8em;color:#5c6370;text-align:center;margin-top:20px}
input[type=range]{width:100%;-webkit-appearance:none;background:#4b5263;height:10px;border-radius:5px;margin-bottom:5px;pointer-events:none}
input[type=range]::-webkit-slider-thumb{-webkit-appearance:none;width:22px;height:22px;background:#61afef;border-radius:50%;cursor:pointer;border:3px solid #282c34;pointer-events:auto}
input[type=range]::-moz-range-thumb{width:20px;height:20px;background:#61afef;border-radius:50%;cursor:pointer;border:3px solid #282c34;pointer-events:auto}
details{border:1px solid #4b5263;border-radius:5px;padding:10px;margin-top:10px;background-color:#2c313a}
details[open]{padding-bottom:15px}
summary{font-weight:bold;color:#c678dd;cursor:pointer;padding:5px;list-style-position:inside}
.color-label{display:flex;align-items:center;justify-content:space-between;}
</style>
<script>
const STRIP_LENGTH=)rawliteral" + String(STRIP_LENGTH) + R"rawliteral(;
function s(u,cb){fetch(u).then(r=>r.text()).then(d=>{if(cb)cb(d)}).catch(e=>console.error(u,e))}
function u(i,v){document.getElementById(i).innerText=v}
function t(){var d=new Date(),e=Math.floor(d.getTime()/1000),z=-d.getTimezoneOffset();fetch(`/setTime?epoch=${e}&tz=${z}`).then(()=>c())}
function c(){fetch('/getCurrentTime').then(r=>r.json()).then(d=>u('currentTimeDisplay',d.time))}
function updateDensity(val){
 s('/setLedDensity?value='+val, ()=>{
  const numLeds=val*STRIP_LENGTH;
  const halfLeds=Math.floor(numLeds/2);
  document.getElementById('v_ml').parentElement.nextElementSibling.max=numLeds;
  document.getElementById('v_al').parentElement.nextElementSibling.max=halfLeds;
  const cs=document.getElementById('v_cs').parentElement.nextElementSibling;
  cs.max=halfLeds; cs.min=-halfLeds;
  document.getElementById('led_count_span').innerText = `(${numLeds} LEDs)`;
 });
}
window.onload=t;setInterval(c,5000);setInterval(t,36e5);
</script>
</head><body><div class='container'>
<h1>LightTrack VISION</h1>
<div style='text-align:center;margin-bottom:20px'>
<p><b>Inactive Zone Setup (X/Y Masks):</b></p>
<a href='/radarview' class='button-link button-nav'>Visualization & Zone Setup</a>
</div><hr>

<h2>Main Light Settings</h2>
<div class='color-label'>
  <label for='target_color'>Color:</label>
  <input type='color' id='target_color' value=')rawliteral" + String(colorStr) + R"rawliteral(' onchange="s('/setTarget?id=0&r='+parseInt(this.value.substring(1,3),16)+'&g='+parseInt(this.value.substring(3,5),16)+'&b='+parseInt(this.value.substring(5,7),16))">
</div>
<label>Brightness (Moving): <span class='value-display' id='v_mi'>)rawliteral" + String((int)(movingIntensity*100.0+0.5)) + R"rawliteral(</span>%</label>
<input type='range' min='0' max='100' value=')rawliteral" + String((int)(movingIntensity*100.0+0.5)) + R"rawliteral(' oninput='u("v_mi",this.value)' onchange='s("/setMovingIntensity?value="+this.value)'>
<label>Length (Moving): <span class='value-display' id='v_ml'>)rawliteral" + String(movingLength) + R"rawliteral(</span> LEDs</label>
<input type='range' min='1' max=')rawliteral" + String(currentNumLeds) + R"rawliteral(' value=')rawliteral" + String(movingLength) + R"rawliteral(' oninput='u("v_ml",this.value)' onchange='s("/setMovingLength?value="+this.value)'>
<label>Additional Length (Directional): <span class='value-display' id='v_al'>)rawliteral" + String(additionalLEDs) + R"rawliteral(</span> LEDs</label>
<input type='range' min='0' max=')rawliteral" + String(currentNumLeds/2) + R"rawliteral(' value=')rawliteral" + String(additionalLEDs) + R"rawliteral(' oninput='u("v_al",this.value)' onchange='s("/setAdditionalLEDs?value="+this.value)'>
<label>Center Shift: <span class='value-display' id='v_cs'>)rawliteral" + String(centerShift) + R"rawliteral(</span> LEDs</label>
<input type='range' min='-)rawliteral" + String(currentNumLeds/2) + R"rawliteral(' max=')rawliteral" + String(currentNumLeds/2) + R"rawliteral(' value=')rawliteral" + String(centerShift) + R"rawliteral(' oninput='u("v_cs",this.value)' onchange='s("/setCenterShift?value="+this.value)'>
<label>Gradient Softness: <span class='value-display' id='v_gs'>)rawliteral" + String(gradientSoftness) + R"rawliteral(</span> (0-10)</label>
<input type='range' min='0' max='10' value=')rawliteral" + String(gradientSoftness) + R"rawliteral(' oninput='u("v_gs",this.value)' onchange='s("/setGradientSoftness?value="+this.value)'>
<label>Off Delay: <span class='value-display' id='v_od'>)rawliteral" + String(ledOffDelay) + R"rawliteral(</span> sec</label>
<input type='range' min='0' max='60' value=')rawliteral" + String(ledOffDelay) + R"rawliteral(' oninput='u("v_od",this.value)' onchange='s("/setLedOffDelay?value="+this.value)'><hr>

<h2>Background Mode</h2>
<button onclick="s('/toggleBackgroundMode');setTimeout(()=>location.reload(),200)" class=')rawliteral" + (backgroundModeActive ? "button-off" : "") + "'>" + (backgroundModeActive ? "Turn Off" : "Turn On") + R"rawliteral( Background</button>
<label style='margin-top:10px'>Brightness (Background/Still): <span class='value-display' id='v_si'>)rawliteral" + String(stationaryIntensity*100.0, 1) + R"rawliteral(</span>%</label>
<input type='range' min='0' max='10' step='0.1' value=')rawliteral" + String(stationaryIntensity*100.0, 1) + R"rawliteral(' oninput='u("v_si",parseFloat(this.value).toFixed(1))' onchange='s("/setStationaryIntensity?value="+this.value)'><hr>

<h2>Schedule</h2>
<div class='time-container'>
<input type='time' id='s_start' value=')rawliteral" + String(scheduleStartStr) + R"rawliteral(' onchange="s('/setSchedule?startHour='+this.value.split(':')[0]+'&startMinute='+this.value.split(':')[1]+'&endHour='+document.getElementById('s_end').value.split(':')[0]+'&endMinute='+document.getElementById('s_end').value.split(':')[1])">
<span>-</span>
<input type='time' id='s_end' value=')rawliteral" + String(scheduleEndStr) + R"rawliteral(' onchange="s('/setSchedule?startHour='+document.getElementById('s_start').value.split(':')[0]+'&startMinute='+document.getElementById('s_start').value.split(':')[1]+'&endHour='+this.value.split(':')[0]+'&endMinute='+this.value.split(':')[1])">
</div>
<div class='current-time'>Local Time: <span id='currentTimeDisplay'>Syncing...</span></div><hr>

<details>
<summary>Strip Settings</summary>
<label>LED Density: <span id="led_count_span" class='value-display'>( )rawliteral" + String(currentNumLeds) + R"rawliteral( LEDs)</span></label>
<div class='radio-group'>
<input type='radio' id='density30' name='ledDensity' value='30' )rawliteral" + (currentLedDensity==30?"checked":"") + R"rawliteral( onchange="updateDensity(30)"><label for='density30'>30 LEDs/m</label>
<input type='radio' id='density60' name='ledDensity' value='60' )rawliteral" + (currentLedDensity==60?"checked":"") + R"rawliteral( onchange="updateDensity(60)"><label for='density60'>60 LEDs/m</label>
</div>
</details><hr>

<div class='footer'>LightTrack VISION</div>
</div></body></html>
)rawliteral";
  server.send(200, "text/html; charset=utf-8", html);
}

void handleSetTargetSettings() { if (server.hasArg("id")) { int id = server.arg("id").toInt(); if (id == 0) { if (server.hasArg("r")) { targetSettings[0].color = CRGB(server.arg("r").toInt(), server.arg("g").toInt(), server.arg("b").toInt()); } saveSettings(); server.send(200, "text/plain", "OK"); return; } } server.send(400, "text/plain", "Bad Request"); }
void handleSetMovingIntensity(){if(server.hasArg("value")){movingIntensity=constrain(server.arg("value").toFloat()/100.0,0.0,1.0);saveSettings();server.send(200,"text/plain","OK");}}
void handleSetStationaryIntensity(){if(server.hasArg("value")){stationaryIntensity=constrain(server.arg("value").toFloat()/100.0,0.0,0.1);saveSettings();server.send(200,"text/plain","OK");}}
void handleSetMovingLength(){if(server.hasArg("value")){movingLength=constrain(server.arg("value").toInt(),1,currentNumLeds);saveSettings();server.send(200,"text/plain","OK");}}
void handleSetAdditionalLEDs(){if(server.hasArg("value")){additionalLEDs=constrain(server.arg("value").toInt(),0,currentNumLeds/2);saveSettings();server.send(200,"text/plain","OK");}}
void handleSetCenterShift(){if(server.hasArg("value")){centerShift=constrain(server.arg("value").toInt(),-currentNumLeds/2,currentNumLeds/2);saveSettings();server.send(200,"text/plain","OK");}}
void handleSetLedDensity(){if(server.hasArg("value")){int n=server.arg("value").toInt();if((n==30||n==60)&&n!=currentLedDensity){currentLedDensity=n;currentNumLeds=constrain(n*STRIP_LENGTH,1,MAX_NUM_LEDS);movingLength=constrain(movingLength,1,currentNumLeds);additionalLEDs=constrain(additionalLEDs,0,currentNumLeds/2);centerShift=constrain(centerShift,-currentNumLeds/2,currentNumLeds/2);saveSettings();fill_solid(leds,MAX_NUM_LEDS,CRGB::Black);FastLED.show();}server.send(200,"text/plain","OK");}}
void handleSetGradientSoftness(){if(server.hasArg("value")){gradientSoftness=constrain(server.arg("value").toInt(),0,10);saveSettings();server.send(200,"text/plain","OK");}}
void handleSetLedOffDelay(){if(server.hasArg("value")){ledOffDelay=constrain(server.arg("value").toInt(),0,600);saveSettings();server.send(200,"text/plain","OK");}}
void handleSetSchedule(){if(server.hasArg("startHour")){startHour=constrain(server.arg("startHour").toInt(),0,23);startMinute=constrain(server.arg("startMinute").toInt(),0,59);endHour=constrain(server.arg("endHour").toInt(),0,23);endMinute=constrain(server.arg("endMinute").toInt(),0,59);saveSettings();updateTime();server.send(200,"text/plain","OK");}}
void handleToggleBackgroundMode(){backgroundModeActive=!backgroundModeActive;saveSettings();server.send(200,"text/plain","OK");}
void handleSmartHomeOn(){lightOn=true;smarthomeOverride=true;server.send(200,"text/plain","ON");}
void handleSmartHomeOff(){lightOn=false;smarthomeOverride=true;server.send(200,"text/plain","OFF");}
void handleSmartHomeClear(){smarthomeOverride=false;updateTime();server.send(200,"text/plain","CLEARED");}
void handleNotFound(){server.send(404,"text/plain","404: Not found");}
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
    if (type == WStype_CONNECTED) {
        IPAddress ip = webSocket.remoteIP(num);
        Serial.printf("[%u] WebSocket Connected from IP %s\n", num, ip.toString().c_str());
        JsonDocument doc;
        JsonArray zonesArray = doc["inactiveZones"].to<JsonArray>();
        for(int i=0; i<MAX_INACTIVE_ZONES; ++i) {
             JsonObject zoneObj = zonesArray.add<JsonObject>();
             zoneObj["id"] = i; zoneObj["enabled"] = inactiveZones[i].enabled;
             zoneObj["minX"] = inactiveZones[i].minX; zoneObj["minY"] = inactiveZones[i].minY;
             zoneObj["maxX"] = inactiveZones[i].maxX; zoneObj["maxY"] = inactiveZones[i].maxY;
        }
        String jsonOutput; serializeJson(doc, jsonOutput);
        webSocket.sendTXT(num, jsonOutput);
    } else if (type == WStype_TEXT) {
        JsonDocument doc;
        if (deserializeJson(doc, payload, length) == DeserializationError::Ok) {
            bool shouldSave = !doc["saveInactiveZones"].isNull();
            if (shouldSave || !doc["setInactiveZones"].isNull()) {
                JsonArray zonesArray = doc[shouldSave ? "saveInactiveZones" : "setInactiveZones"];
                for (JsonObject zoneData : zonesArray) {
                    int id = zoneData["id"] | -1;
                    if (id >= 0 && id < MAX_INACTIVE_ZONES) {
                        inactiveZones[id].enabled = zoneData["enabled"];
                        inactiveZones[id].minX = zoneData["minX"]; inactiveZones[id].minY = zoneData["minY"];
                        inactiveZones[id].maxX = zoneData["maxX"]; inactiveZones[id].maxY = zoneData["maxY"];
                    }
                }
                if (shouldSave) {
                    saveSettings();
                    webSocket.sendTXT(num, "{\"zonesSaved\": true}");
                }
            }
        }
    }
}
void broadcastRadarData() {
    if (webSocket.connectedClients() == 0) return;
    JsonDocument doc;
    JsonArray targetsArray = doc["targets"].to<JsonArray>();
    for (int i = 0; i < maxActiveTargets; i++) {
        if (targets[i].present && targetSettings[i].enabled) {
            JsonObject targetObj = targetsArray.add<JsonObject>();
            targetObj["id"] = i; targetObj["x"] = targets[i].x; targetObj["y"] = targets[i].y; targetObj["s"] = targets[i].speed;
        }
    }
    String jsonOutput; serializeJson(doc, jsonOutput);
    webSocket.broadcastTXT(jsonOutput);
}
void handleRadarView() {
  String html = R"rawliteral(
<!DOCTYPE html><html lang="en"><head><meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no">
<title>Radar Visualization & Zone Setup</title><style>body{margin:0;font-family:sans-serif;background-color:#282c34;color:#abb2bf;display:flex;flex-direction:column;align-items:center;padding-top:5px;box-sizing:border-box;overflow:hidden}h1{color:#61afef;margin-top:0;margin-bottom:5px;text-align:center;flex-shrink:0;font-size:1.3em}#canvasContainer{position:relative;width:95%;max-width:500px;flex-grow:1;display:flex;justify-content:center;align-items:center;margin-bottom:5px}#radarCanvas{border:1px solid #4b5263;background-color:#3a3f4b;touch-action:none;max-width:100%;max-height:100%;object-fit:contain;cursor:default}#status{margin-top:3px;color:#5c6370;font-size:.85em;flex-shrink:0;text-align:center;height:1.1em}.controls{margin-top:5px;display:flex;flex-wrap:wrap;justify-content:center;align-items:center;gap:8px;flex-shrink:0}.controls label{margin:0 5px;white-space:nowrap;font-size:.9em}.controls input[type=range]{vertical-align:middle;cursor:pointer;height:8px}.controls button{padding:6px 10px;font-size:.85em;border:none;border-radius:4px;cursor:pointer;background-color:#61afef;color:#282c34}button.del-btn{background-color:#e06c75}button:disabled{background-color:#4b5263;color:#5c6370;cursor:not-allowed;opacity:.7}.zoom-value{color:#e5c07b;font-weight:bold}a.button-link{display:inline-block;text-decoration:none;font-size:.85em;margin:5px;padding:6px 10px;border:none;border-radius:4px;background-color:#98c379;color:#282c34!important;cursor:pointer;text-align:center;flex-shrink:0}.zone-handle{position:absolute;width:24px;height:24px;margin-left:-12px;margin-top:-12px;border-radius:50%;display:none;z-index:10}.zone-handle::before{content:'';position:absolute;left:8px;top:8px;width:8px;height:8px;background-color:#fff;border-radius:50%;border:1px solid #333}</style>
</head><body><h1>Radar Map / Inactive Zone Editor</h1>
<div id="canvasContainer"><canvas id="radarCanvas"></canvas>
<div class="zone-handle" id="handle-tl" style="cursor:nwse-resize"></div><div class="zone-handle" id="handle-tr" style="cursor:nesw-resize"></div><div class="zone-handle" id="handle-bl" style="cursor:nesw-resize"></div><div class="zone-handle" id="handle-br" style="cursor:nwse-resize"></div>
<div class="zone-handle" id="handle-tm" style="cursor:ns-resize"></div><div class="zone-handle" id="handle-bm" style="cursor:ns-resize"></div><div class="zone-handle" id="handle-lm" style="cursor:ew-resize"></div><div class="zone-handle" id="handle-rm" style="cursor:ew-resize"></div>
</div><div id="status">Connecting...</div><div class="controls">
<label>Zoom: <input type="range" id="zoomSlider" min="40" max="180" value="60" step="1"> <span id="zoomValue" class="zoom-value">60</span> px/m</label>
<button id="addZoneBtn">Add Zone</button>
<button id="deleteZoneBtn" class="del-btn" disabled>Delete Selected</button>
<button id="saveZonesBtn">Save Zones</button></div>
<a href="/" class="button-link">Back to Main Settings</a>
<script>
const C=document.getElementById('radarCanvas'),X=C.getContext('2d'),S=document.getElementById('status'),Z=document.getElementById('zoomSlider'),V=document.getElementById('zoomValue'),B=document.getElementById('saveZonesBtn'),T=document.getElementById('addZoneBtn'),Q=document.getElementById('deleteZoneBtn'),H={tl:document.getElementById('handle-tl'),tr:document.getElementById('handle-tr'),bl:document.getElementById('handle-bl'),br:document.getElementById('handle-br'),tm:document.getElementById('handle-tm'),bm:document.getElementById('handle-bm'),lm:document.getElementById('handle-lm'),rm:document.getElementById('handle-rm')};let tg=[],sc=Z.value/1000,ws,af,zd=[],sz=-1,di={a:!1,h:null,sX:0,sY:0},nr=!0;const O=()=>({x:C.width/2,y:.9*C.height}),M=(x,y)=>{const o=O();return{x:o.x+x*sc,y:o.y-y*sc}},P=(x,y)=>{const o=O();return{x:(x-o.x)/sc,y:(o.y-y)/sc}},E=(e,t)=>{const n=t.getBoundingClientRect(),o=e.clientX??e.touches?.[0]?.clientX,c=e.clientY??e.touches?.[0]?.clientY;return o===void 0||c===void 0?null:{x:o-n.left,y:c-n.top}};function R(){const e=document.getElementById("canvasContainer").clientWidth,t=document.querySelector(".controls").offsetHeight+S.offsetHeight+document.querySelector("h1").offsetHeight+document.querySelector(".button-link").offsetHeight+20,n=window.innerHeight-t-10,o=Math.min(e,500);let c=o,l=o;l>n&&(l=n,c*=n/l),c=Math.max(c,150),l=Math.max(l,150),C.width===Math.floor(c)&&C.height===Math.floor(l)||(C.width=Math.floor(c),C.height=Math.floor(l),nr=!0)}
function D(){const e=C.width,t=C.height;if(e<=0||t<=0)return;const n=O();X.fillStyle="#3a3f4b",X.fillRect(0,0,e,t),X.strokeStyle="#4b5263",X.lineWidth=.5,X.beginPath(),X.moveTo(n.x,n.y),X.lineTo(n.x,0),X.stroke(),X.beginPath(),X.moveTo(0,n.y),X.lineTo(e,n.y),X.stroke(),X.textAlign="center",X.fillStyle="#5c6370";const o=Math.max(8,Math.min(12,Math.floor(e/35)));X.font=`${o}px sans-serif`;const c=Math.ceil(P(0,0).y/1e3);for(let l=1;l<=c;l++){const a=1e3*l*sc;a<5||(X.beginPath(),X.arc(n.x,n.y,a,1.02*Math.PI,1.98*Math.PI),X.stroke(),n.y-a>o+2&&X.fillText(`${l}m`,n.x,n.y-a-3))}n.x+1e3*sc<e-15&&X.fillText("1m",n.x+1e3*sc,n.y+o+2),n.x>o+5&&X.fillText("Y",n.x,o);let l=-1;const a=["rgba(224,108,117,.3)","rgba(229,192,123,.3)","rgba(152,195,121,.3)","rgba(97,175,239,.3)"],i=["#e06c75","#e5c07b","#98c379","#61afef"];zd.forEach((d,f)=>{if(!d.enabled)return;let m,g,p,h,u,w;try{m=M(d.minX,d.maxY),g=M(d.maxX,d.minY),p=m.x,h=m.y,u=g.x-m.x,w=g.y-m.y}catch{return}X.save();const y=sz===f;X.strokeStyle=y?"#FFFFFF":i[f],X.fillStyle=a[f],X.lineWidth=2,isFinite(p)&&isFinite(h)&&isFinite(u)&&isFinite(w)&&u>0&&w>0&&(X.fillRect(p,h,u,w),X.strokeRect(p,h,u,w),X.fillStyle=y?"#FFFFFF":i[f],X.textAlign="left",X.fillText(`${d.id+1}`,p+4,h+o+4),y&&(l=f,A(p,h,u,w))),X.restore()}),l===-1&&I(),X.font=`${Math.max(9,Math.min(13,Math.floor(e/30)))}px sans-serif`;const s=Math.max(4,Math.min(8,Math.floor(e/50)));tg.forEach(d=>{const f=M(d.x,d.y);let m="#e06c75";d.s>3?m="#98c379":d.s<-3&&(m="#61afef");let g=zd.some(p=>p.enabled&&d.x>=p.minX&&d.x<=p.maxX&&d.y>=p.minY&&d.y<=p.maxY);X.globalAlpha=g?.4:1,X.fillStyle=m,X.beginPath(),X.arc(f.x,f.y,s,0,2*Math.PI),X.fill(),X.globalAlpha=1,X.fillStyle="#abb2bf",X.textAlign="left",X.fillText(`${d.id+1}`,f.x+s+2,f.y+s/2)}),U(),nr=!1}
function L(){nr&&D(),af=requestAnimationFrame(L)}
function N(){const e=`ws://${window.location.hostname}:81/`;S.textContent=`Connecting to ${e}...`,ws&&ws.readyState!==WebSocket.CLOSED&&(ws.onclose=null,ws.close()),ws=new WebSocket(e),ws.onopen=()=>{console.log("[WebSocket] OPEN."),af||L(),nr=!0},ws.onclose=()=>{console.log("[WebSocket] CLOSED."),S.textContent="Disconnected. Reconnecting in 5s...",tg=[],zd=[],sz=-1,U(),af&&(cancelAnimationFrame(af),af=null),nr=!0,D(),setTimeout(N,5e3)},ws.onerror=e=>{console.error("[WebSocket] Error:",e),S.textContent="WebSocket Error"},ws.onmessage=e=>{try{const t=JSON.parse(e.data);let n=!1;if(t.targets!==void 0&&(tg=t.targets,n=!0),t.inactiveZones!==void 0&&(zd=t.inactiveZones,sz>=zd.length&&(sz=-1),U(),n=!0),t.zonesSaved!==void 0&&alert(t.zonesSaved?"Zones saved to device memory!":"Error saving zones."),n)nr=!0}catch(t){console.error("[WebSocket] JSON Parse Error:",t,e.data),S.textContent="Data Error"}}}
function G(e,t){for(let n=zd.length-1;n>=0;n--){const o=zd[n];if(!o.enabled)continue;const c=M(o.minX,o.minY),l=M(o.maxX,o.maxY);if(isFinite(c.x)&&isFinite(l.y)&&isFinite(l.x)&&isFinite(c.y)&&e>=c.x&&e<=l.x&&t>=l.y&&t<=c.y)return n}return -1}
function F(){if(!ws||ws.readyState!==WebSocket.OPEN)return;const e={setInactiveZones:zd.map(t=>({id:t.id,enabled:t.enabled,minX:t.minX,minY:t.minY,maxX:t.maxX,maxY:t.maxY}))};ws.send(JSON.stringify(e))}
function U(){const e=sz!==-1;Q.disabled=!e,T.disabled=zd.filter(d=>d.enabled).length>=zd.length;const t=e?` | Editing Zone ${sz+1}`:"";let n="Connecting...";ws&&(n=ws.readyState===WebSocket.OPEN?"Connected":ws.readyState===WebSocket.CONNECTING?"Connecting...":"Offline"),S.textContent=`${n} | Targets: ${tg.length}${t}`,C.style.cursor=di.h?H[di.h].style.cursor:"move"}
function A(e,t,n,o){if(![e,t,n,o].every(isFinite)){I();return}const c=(i,s,d)=>{i.style.left=`${s}px`,i.style.top=`${d}px`,i.style.display="block"};c(H.tl,e,t),c(H.tr,e+n,t),c(H.bl,e,t+o),c(H.br,e+n,t+o),c(H.tm,e+n/2,t),c(H.bm,e+n/2,t+o),c(H.lm,e,t+o/2),c(H.rm,e,t+o/2)}
function I(){for(const e in H)H[e].style.display="none"}
function K(e){const t=E(e,C);if(!t)return;let n=!1,o=-1;di.a=!1,di.h=null;const c=25;for(const l in H)if(H[l].style.display!=="none"){const a=parseFloat(H[l].style.left),i=parseFloat(H[l].style.top);if(t.x>=a-c&&t.x<=a+c&&t.y>=i-c&&t.y<=i+c){di.h=l,o=sz,n=!0;break}}n||(G(t.x,t.y)!==-1&&(o=G(t.x,t.y),di.h=null,n=!0)),o!==sz&&(sz=o,nr=!0),n&&sz!==-1&&(e.preventDefault(),di.a=!0,di.sX=t.x,di.sY=t.y,di.z=Object.assign({},zd[sz])),U()}
function Y(e){if(!di.a||sz===-1)return;const t=E(e,C);if(!t)return;e.preventDefault();const n=zd[sz],o=(t.x-di.sX)/sc,c=(t.y-di.sY)/sc;if(di.h){di.h.includes("l")&&(n.minX=di.z.minX+o),di.h.includes("r")&&(n.maxX=di.z.maxX+o),di.h.includes("t")&&(n.maxY=di.z.maxY-c),di.h.includes("b")&&(n.minY=di.z.minY-c)}else n.minX=di.z.minX+o,n.maxX=di.z.maxX+o,n.minY=di.z.minY-c,n.maxY=di.z.maxY-c;nr=!0,F()}
function J(e){di.a&&(di.a=!1,sz!==-1&&([zd[sz].minX,zd[sz].maxX]=zd[sz].minX>zd[sz].maxX?[zd[sz].maxX,zd[sz].minX]:[zd[sz].minX,zd[sz].maxX],[zd[sz].minY,zd[sz].maxY]=zd[sz].minY>zd[sz].maxY?[zd[sz].maxY,zd[sz].minY]:[zd[sz].minY,zd[sz].maxY]),F(),nr=!0,U())}
C.addEventListener("mousedown",K),C.addEventListener("mousemove",Y),C.addEventListener("mouseup",J),C.addEventListener("mouseleave",J),C.addEventListener("touchstart",K,{passive:!1}),C.addEventListener("touchmove",Y,{passive:!1}),C.addEventListener("touchend",J),C.addEventListener("touchcancel",J),B.onclick=()=>{if(!ws||ws.readyState!==WebSocket.OPEN)return;const e={setInactiveZones:zd.map(t=>({id:t.id,enabled:t.enabled,minX:t.minX,minY:t.minY,maxX:t.maxX,maxY:t.maxY}))};ws.send(JSON.stringify(e))},Z.oninput=()=>{sc=Z.value/1e3,V.textContent=Z.value,nr=!0},T.onclick=()=>{const e=zd.find(t=>!t.enabled);e?(e.enabled=!0,e.minY=1800,e.maxY=2200,e.minX=-200,e.maxX=200,sz=e.id,F(),nr=!0,U()):alert(`All ${zd.length} zones are in use. Delete one first.`)},Q.onclick=()=>{sz===-1||(zd[sz].enabled=!1,sz=-1,F(),nr=!0,U())},window.addEventListener("resize",R),window.addEventListener("load",()=>{R(),N()});
</script></body></html>
)rawliteral";
  server.send(200, "text/html; charset=utf-8", html);
}
