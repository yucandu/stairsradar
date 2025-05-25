#include <Arduino.h>
#include <ArduinoOTA.h>
#include <ld2410.h>
#include <Adafruit_NeoPixel.h>
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>


char auth[] = "xzOx2JrFeyFk7ea6zAZmHCzqBRC_ciuH";  //BLYNK

const char *ssid = "mikesnet";
const char *password = "springchicken";

#define LED_PIN 5
#define DEFAULT_NUM_LEDS 32
#define DEFAULT_BRIGHTNESS 255
#define DEFAULT_MOVING_LIGHT_SPAN 3
#define DEFAULT_MIN_DISTANCE 175
#define DEFAULT_MAX_DISTANCE 550
#define EEPROM_INITIALIZED_MARKER 123
#define EEPROM_SIZE 32

// Default color (white)
#define DEFAULT_RED 255
#define DEFAULT_GREEN 255
#define DEFAULT_BLUE 255

// 🎯 LD2410 Config
#define RADAR_SERIAL Serial1
#define RADAR_RX_PIN 21
#define RADAR_TX_PIN 20

#define THRESHOLD_VAL 175
#define FADE_STEPS 255  // How many steps to take during fade
int fadeTime = 5;    // Time in ms for full fade transition
unsigned long lastFadeUpdate = 0;

#define every(interval) \
    static uint32_t __every__##interval = millis(); \
    if (millis() - __every__##interval >= interval && (__every__##interval = millis()))

// Arrays to store current and target colors for each LED
struct Color {
  uint8_t r, g, b;
};
Color* currentColors;
Color* targetColors;

ld2410 radar;

int minDistance, maxDistance, brightness, movingLightSpan, numLeds;
int redValue, greenValue, blueValue;
int currentDistance = 0;
unsigned long turnOnTime = 0;
bool isLightOn = false;
bool wasTriggered = false;

bool getDistance = false;
Adafruit_NeoPixel strip = Adafruit_NeoPixel(DEFAULT_NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ400);
WidgetTerminal terminal(V50);

BLYNK_WRITE(V50) {
  if (String("help") == param.asStr()) {
    terminal.println("==List of available commands:==");
    terminal.println("wifi");
    terminal.println("blink");
    terminal.println("reset");
    terminal.println("dist");
    terminal.println("==End of list.==");
  }
  if (String("wifi") == param.asStr()) {
    terminal.print("Connected to: ");
    terminal.println(ssid);
    terminal.print("IP address:");
    terminal.println(WiFi.localIP());
    terminal.print("Signal strength: ");
    terminal.println(WiFi.RSSI());
    //printLocalTime();
  }

  if (String("blink") == param.asStr()) {
    terminal.println("Blinking...");
    for(int i = 0; i < DEFAULT_NUM_LEDS; i++) {
      strip.setPixelColor(i, strip.Color(0, 148, 211)); // GRB for violet
    }
    //strip.setPixelColor(3, strip.Color(0, 148, 211)); // GRB for violet
    strip.show(); // Initialize all pixels to 'off'
    delay(1000);
  }

  if (String("reset") == param.asStr()) {
    terminal.println("Restarting...");
    terminal.flush();
    ESP.restart();
  }
  if (String("dist") == param.asStr()) {
    getDistance = !getDistance;
    radar.read();

    if (radar.isConnected()) {
      currentDistance = radar.movingTargetDistance();
      if (currentDistance == 0) currentDistance = radar.stationaryTargetDistance();
      terminal.println("Radar is connected!");
      terminal.print("Distance: ");
      terminal.print(currentDistance);
      terminal.println(" cm");
    }
    else {
      terminal.println("Radar is NOT connected!");
    }

  }
  terminal.flush();
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
 
// Initialize LED strip - make it global and accessible from other modules


void setupLEDs() {
  Serial.println("Initializing LED strip...");
  strip.begin();
  strip.setBrightness(brightness);
  strip.show();
  
  // Allocate memory for color arrays
  currentColors = new Color[DEFAULT_NUM_LEDS];
  targetColors = new Color[DEFAULT_NUM_LEDS];
  
  // Initialize all colors to black
  for(int i = 0; i < DEFAULT_NUM_LEDS; i++) {
      currentColors[i] = {0, 0, 0};
      targetColors[i] = {0, 0, 0};
  }
  
  Serial.println("LED strip initialized with " + String(numLeds) + " LEDs and brightness " + String(brightness));
}

void updateLEDConfig() {
  Serial.println("Updating LED configuration: numLeds=" + String(numLeds) + ", brightness=" + String(brightness));
  strip.updateLength(numLeds);
  strip.setBrightness(brightness);
  for(int i = 0; i < DEFAULT_NUM_LEDS; i++) {
    strip.setPixelColor(i, strip.Color(0, 148, 211)); // GRB for violet
  }
  //strip.setPixelColor(3, strip.Color(0, 148, 211)); // GRB for violet
  strip.show(); // Initialize all pixels to 'off'
  delay(1000);
  strip.clear();
  strip.show();
}
// Global variables to track state

// Add this helper function
void setTargetPixelColor(int pixel, uint32_t color) {
  if(pixel >= 0 && pixel < numLeds) {
      targetColors[pixel].g = (color >> 16) & 0xFF;
      targetColors[pixel].r = (color >> 8) & 0xFF;
      targetColors[pixel].b = color & 0xFF;
  }
}




void updateLEDs(int start_led) {
  // First, set all target colors to black
  for(int i = 0; i < numLeds; i++) {
      setTargetPixelColor(i, 0);
  }

  // Set target colors based on isLightOn state
  if (isLightOn) {
      if (start_led - 3 >= 0) {
          setTargetPixelColor(start_led - 3, strip.Color(50, 200, 0));
      }
      if (start_led - 2 >= 0) {
          setTargetPixelColor(start_led - 2, strip.Color(165, 255, 0));
      }
      if (start_led - 1 >= 0) {
          setTargetPixelColor(start_led - 1, strip.Color(255, 255, 0));
      }
      
      for (int i = start_led; i < start_led + movingLightSpan; i++) {
          if (i < numLeds) {
              setTargetPixelColor(i, strip.Color(greenValue, redValue, blueValue));
          }
      }
      
      if (start_led + movingLightSpan < numLeds) {
          setTargetPixelColor(start_led + movingLightSpan, strip.Color(255, 0, 255));
      }
      if (start_led + movingLightSpan + 1 < numLeds) {
          setTargetPixelColor(start_led + movingLightSpan + 1, strip.Color(0, 0, 255));
      }
      if (start_led + movingLightSpan + 2 < numLeds) {
          setTargetPixelColor(start_led + movingLightSpan + 2, strip.Color(0, 148, 211));
      }
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
          
          // Update LED with current color
          strip.setPixelColor(i, strip.Color(currentColors[i].g, currentColors[i].r, currentColors[i].b));
      }
      
      if(needsUpdate) {
          strip.show();
      }
  }
}


void setupRadar() {
  Serial.println("Initializing radar sensor...");
  RADAR_SERIAL.begin(256000, SERIAL_8N1, RADAR_RX_PIN, RADAR_TX_PIN);
  radar.begin(RADAR_SERIAL);
  Serial.println("Radar sensor initialized");
}

bool processRadarReading() {
  radar.read();

  if (radar.isConnected()) {
      currentDistance = radar.movingTargetDistance();
      if (currentDistance == 0) currentDistance = radar.stationaryTargetDistance();
      Serial.println(currentDistance);

      // Only turn on lights when motion is detected in threshold zone
      if (currentDistance >= (maxDistance - THRESHOLD_VAL) && currentDistance <= maxDistance) {
          isLightOn = true;
          turnOnTime = millis();
      } else if (millis() - turnOnTime > 5000) { // Turn off after 30 seconds
          isLightOn = false;
      } else if (currentDistance <= minDistance) {isLightOn = false;}
      

      if (currentDistance < minDistance) currentDistance = minDistance;
      
      // Calculate the LED start position based on current distance
      int start_led = map(currentDistance, minDistance, maxDistance, 0, numLeds - movingLightSpan);
      start_led = constrain(start_led, 0, numLeds - movingLightSpan);

      // Update the LEDs
      updateLEDs(start_led);
      return true;
  }
  
  return false;
}



void setup() {
  Serial.begin(9600);
  setupLEDs();    
  
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  WiFi.setTxPower (WIFI_POWER_8_5dBm);
      while (WiFi.status() != WL_CONNECTED) {
        if (millis() > 20000) {
            break;
          }
          delay(100);
      }
  Blynk.config(auth, IPAddress(192, 168, 50, 197), 8080);
  Blynk.connect();
  // Critical initialization sequence:
  // 1. EEPROM first to load settings
  setupEEPROM();     
  
  // 2. LED strip after settings are loaded
     
  
  // 3. Update LED configuration with the loaded settings
  updateLEDConfig();
  
  // 4. Initialize radar sensor
  setupRadar();      
  
  ArduinoOTA.setHostname("stairsRadar");
  ArduinoOTA.begin();
  terminal.println("Stairs Radar Light System");

  terminal.print("Connected to ");
  terminal.println(ssid);
  terminal.print("IP address: ");
  terminal.println(WiFi.localIP());
  terminal.flush();
}

void loop() {
  ArduinoOTA.handle();
  Blynk.run();
  processRadarReading();
every(500) {
    if (getDistance) {
      Blynk.virtualWrite(V51, currentDistance);
      Blynk.virtualWrite(V52, radar.movingTargetDistance());
      Blynk.virtualWrite(V53, radar.stationaryTargetDistance());
    }
  }


}
