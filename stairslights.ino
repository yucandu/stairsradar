#include <Arduino.h>
#include <ArduinoOTA.h>
#include <ld2410.h>
#include <FastLED.h>
#include <WiFi.h>


const char *ssid = "mikesnet";
const char *password = "springchicken";

#define LED_PIN 5
#define DEFAULT_NUM_LEDS 32
#define DEFAULT_BRIGHTNESS 255
#define LED_TYPE WS2811
#define COLOR_ORDER GRB

#define DEFAULT_MOVING_LIGHT_SPAN 3
#define DEFAULT_MIN_DISTANCE 175
#define DEFAULT_MAX_DISTANCE 550
#define EEPROM_INITIALIZED_MARKER 123
#define EEPROM_SIZE 32
#define DEFAULT_RED 255
#define DEFAULT_GREEN 255
#define DEFAULT_BLUE 255

CRGB leds[DEFAULT_NUM_LEDS];

// 🎯 LD2410 Config
#define RADAR_SERIAL Serial1
#define RADAR_RX_PIN 21
#define RADAR_TX_PIN 20

#define THRESHOLD_VAL 100
#define FADE_STEPS 255  // How many steps to take during fade
int fadeTime = 5;    // Time in ms for full fade transition
unsigned long lastFadeUpdate = 0;

// Arrays to store current and target colors for each LED
struct Color {
  uint8_t r, g, b;
};
Color* currentColors;
Color* targetColors;

int minDistance, maxDistance, brightness, movingLightSpan, numLeds;
int redValue, greenValue, blueValue;
int currentDistance = 0;
unsigned long turnOnTime = 0;
bool isLightOn = false;
bool wasTriggered = false;
unsigned long lastMotionTime = 0; // Add this global variable

void pride() 
{
  static uint16_t sPseudotime = 0;
  static uint16_t sLastMillis = 0;
  static uint16_t sHue16 = 0;
 
  uint8_t sat8 = beatsin88( 87, 220, 250);
  uint8_t brightdepth = beatsin88( 341, 96, 224);
  uint16_t brightnessthetainc16 = beatsin88( 203, (25 * 256), (40 * 256));
  uint8_t msmultiplier = beatsin88(147, 23, 60);

  uint16_t hue16 = sHue16;//gHue * 256;
  uint16_t hueinc16 = beatsin88(113, 1, 3000);
  
  uint16_t ms = millis();
  uint16_t deltams = ms - sLastMillis ;
  sLastMillis  = ms;
  sPseudotime += deltams * msmultiplier;
  sHue16 += deltams * beatsin88( 400, 5,9);
  uint16_t brightnesstheta16 = sPseudotime;
  
  for( uint16_t i = 0 ; i < DEFAULT_NUM_LEDS; i++) {
    hue16 += hueinc16;
    uint8_t hue8 = hue16 / 256;

    brightnesstheta16  += brightnessthetainc16;
    uint16_t b16 = sin16( brightnesstheta16  ) + 32768;

    uint16_t bri16 = (uint32_t)((uint32_t)b16 * (uint32_t)b16) / 65536;
    uint8_t bri8 = (uint32_t)(((uint32_t)bri16) * brightdepth) / 65536;
    bri8 += (255 - brightdepth);
    
    CRGB newcolor = CHSV( hue8, sat8, bri8);
    
    uint16_t pixelnumber = i;
    pixelnumber = (DEFAULT_NUM_LEDS-1) - pixelnumber;
    
    nblend( leds[pixelnumber], newcolor, 64);
  }
}

void setupEEPROM() {
    minDistance = DEFAULT_MIN_DISTANCE;
    maxDistance = DEFAULT_MAX_DISTANCE;
    brightness = DEFAULT_BRIGHTNESS;
    movingLightSpan = DEFAULT_MOVING_LIGHT_SPAN;
    redValue = DEFAULT_RED;
    greenValue = DEFAULT_GREEN;
    blueValue = DEFAULT_BLUE;
    numLeds = DEFAULT_NUM_LEDS;
  // Validate values to prevent issues
  if (minDistance < 0 || minDistance > 500) minDistance = DEFAULT_MIN_DISTANCE;
  if (maxDistance < 50 || maxDistance > 1000) maxDistance = DEFAULT_MAX_DISTANCE;
  if (brightness < 0 || brightness > 255) brightness = DEFAULT_BRIGHTNESS;
  if (movingLightSpan < 1 || movingLightSpan > 300) movingLightSpan = DEFAULT_MOVING_LIGHT_SPAN;
  if (numLeds < 1 || numLeds > 2000) numLeds = DEFAULT_NUM_LEDS;
  
}
 


void setupLEDs() {
  Serial.println("Initializing LED strip...");
  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, DEFAULT_NUM_LEDS)
    .setCorrection(TypicalLEDStrip)
    .setDither(brightness < 255);
  FastLED.setBrightness(brightness);

  // Allocate memory for color arrays
  currentColors = new Color[DEFAULT_NUM_LEDS];
  targetColors = new Color[DEFAULT_NUM_LEDS];

  // Initialize all colors to black
  for(int i = 0; i < DEFAULT_NUM_LEDS; i++) {
      currentColors[i] = {0, 0, 0};
      targetColors[i] = {0, 0, 0};
      leds[i] = CRGB::Black;
  }
  FastLED.show();
  Serial.println("LED strip initialized with " + String(numLeds) + " LEDs and brightness " + String(brightness));
}

void updateLEDConfig() {
  Serial.println("Updating LED configuration: numLeds=" + String(numLeds) + ", brightness=" + String(brightness));
  // FastLED does not support dynamic resizing, so you must restart to change numLeds
  FastLED.setBrightness(brightness);
  leds[3] = CRGB(0, 148, 211); // Violet (GRB)
  FastLED.show();
  delay(1000);
  while (millis()<5000){
    pride();
    FastLED.show();
  }
  fill_solid(leds, numLeds, CRGB::Black);
  FastLED.show();
}

// Helper to set target color
void setTargetPixelColor(int pixel, CRGB color) {
  if(pixel >= 0 && pixel < numLeds) {
      targetColors[pixel].r = color.r;
      targetColors[pixel].g = color.g;
      targetColors[pixel].b = color.b;
  }
}




void updateLEDs(int start_led) {
  // First, set all target colors to black
  for(int i = 0; i < numLeds; i++) {
      setTargetPixelColor(i, CRGB::Black);
  }

  // Set target colors based on isLightOn state
  if (isLightOn) {
      if (start_led - 3 >= 0) setTargetPixelColor(start_led - 3, CRGB(50, 200, 0));
      if (start_led - 2 >= 0) setTargetPixelColor(start_led - 2, CRGB(165, 255, 0));
      if (start_led - 1 >= 0) setTargetPixelColor(start_led - 1, CRGB(255, 255, 0));

      for (int i = start_led; i < start_led + movingLightSpan; i++) {
          if (i < numLeds) setTargetPixelColor(i, CRGB(greenValue, redValue, blueValue));
      }

      if (start_led + movingLightSpan < numLeds) setTargetPixelColor(start_led + movingLightSpan, CRGB(255, 0, 255));
      if (start_led + movingLightSpan + 1 < numLeds) setTargetPixelColor(start_led + movingLightSpan + 1, CRGB(0, 0, 255));
      if (start_led + movingLightSpan + 2 < numLeds) setTargetPixelColor(start_led + movingLightSpan + 2, CRGB(0, 148, 211));
  }

  // Handle fading
  unsigned long currentTime = millis();
  if (currentTime - lastFadeUpdate >= (fadeTime / FADE_STEPS)) {
      lastFadeUpdate = currentTime;
      bool needsUpdate = false;

      for(int i = 0; i < numLeds; i++) {
          // Fade each color channel
          for(int c = 0; c < 3; c++) {
              uint8_t* current = c == 0 ? &currentColors[i].r : (c == 1 ? &currentColors[i].g : &currentColors[i].b);
              uint8_t target = c == 0 ? targetColors[i].r : (c == 1 ? targetColors[i].g : targetColors[i].b);

              if(*current < target) {
                  (*current)++;
                  needsUpdate = true;
              } else if(*current > target) {
                  (*current)--;
                  needsUpdate = true;
              }
          }
          leds[i] = CRGB(currentColors[i].r, currentColors[i].g, currentColors[i].b);
      }

      if(needsUpdate) {
          FastLED.show();
      }
  }
}

ld2410 radar;

void setupRadar() {
  Serial.println("Initializing radar sensor...");
  RADAR_SERIAL.begin(256000, SERIAL_8N1, RADAR_RX_PIN, RADAR_TX_PIN);
  radar.begin(RADAR_SERIAL);
  Serial.println("Radar sensor initialized");
}

bool processRadarReading() {
  radar.read();

  if (radar.isConnected()) {
      int movingDistance = radar.movingTargetDistance();
      currentDistance = movingDistance;
      if (currentDistance == 0) currentDistance = radar.stationaryTargetDistance();
      Serial.println(currentDistance);

      // Update lastMotionTime if motion detected
      if (movingDistance > 0) {
          lastMotionTime = millis();
      }

      // Only turn on lights when motion is detected in threshold zone
      if (currentDistance >= (maxDistance - THRESHOLD_VAL) && currentDistance <= maxDistance) {
          isLightOn = true;
          turnOnTime = millis();
      } else if (millis() - turnOnTime > 5000) { // Turn off after 5 seconds (was 30)
          isLightOn = false;
      } else if (currentDistance <= minDistance) {
          isLightOn = false;
      }

      // NEW: If lights are on, but no motion for >10s, turn off
      if (isLightOn && (millis() - lastMotionTime > 10000)) {
          isLightOn = false;
      }

      if (currentDistance < minDistance) currentDistance = minDistance;
      
      // Calculate the LED start position based on current distance
      int start_led = map(currentDistance, minDistance, maxDistance, 0, numLeds - movingLightSpan);
      start_led = constrain(start_led, 0, numLeds - movingLightSpan);

      // Update the LEDs
      updateLEDs(start_led);
      return true;
  }
  else {
    Serial.println("Radar not connected!");
  }
  
  return false;
}



void setup() {
  Serial.begin(9600);
  setupEEPROM();  
  setupLEDs();    
  updateLEDConfig();
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  WiFi.setTxPower (WIFI_POWER_8_5dBm);
      while (WiFi.status() != WL_CONNECTED) {
        if (millis() > 20000) {
            break;
          }
          delay(100);
      }
  
  // Critical initialization sequence:
  // 1. EEPROM first to load settings
     
  
  // 2. LED strip after settings are loaded
     
  
  // 3. Update LED configuration with the loaded settings
  
  
  // 4. Initialize radar sensor
  setupRadar();      
  
  ArduinoOTA.setHostname("stairsRadar");
  ArduinoOTA.begin();
}

void loop() {
  ArduinoOTA.handle();
  processRadarReading();
}
