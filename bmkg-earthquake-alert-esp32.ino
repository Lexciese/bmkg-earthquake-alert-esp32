#include <math.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <HTTPClient.h>
#include <string.h>
#include <ArduinoJson.h>



// =================================================== CONFIG ============================================
// Replace with your network credentials
const char* ssid = "your wifi name";
const char* password = "your wifi password";

// Thresholds for alerts
float threshold_dist = 500;  // Distance threshold in kilometers
float threshold_mag = 2;     // Magnitude threshold

String apiKey = "enter your OpenWeatherMap API Key"; // change to your OpenWeatherMap API Key
String location = "your location"; // your location
String countryCode = "ID"; // your country code
// ========================================================================================================

// API URLs
String apiUrl_bmkg = "https://bmkg-content-inatews.storage.googleapis.com/lastQL.json?"; // don't change
String apiUrl_geo = "http://api.openweathermap.org/geo/1.0/direct?q=" + location + "," + countryCode + "&limit=5&appid=" + apiKey;

// Geolocation coordinates
double currentGeo[2] = { 0, 0 };  // Current location (lat, lon)
double targetGeo[2] = { 0, 0 };   // Earthquake location (lat, lon)

// NTP server and port
const char* ntpServer = "pool.ntp.org";
const int ntpPort = 123;
const int timeOffset = 25200;  // UTC+7 for WIB
unsigned long epochTime = 0;

// UDP instance for NTP communication
WiFiUDP udp;
const int NTP_PACKET_SIZE = 48;
byte packetBuffer[NTP_PACKET_SIZE];

// Global variables
String epicenter;  // Epicenter of the earthquake
float mag;         // Magnitude of the earthquake
double distance;   // Distance from the current location to the earthquake

// Task handles
TaskHandle_t bmkgTaskHandle = NULL;
TaskHandle_t alertTaskHandle = NULL;

// Mutex for shared resources (e.g., targetGeo, mag)
SemaphoreHandle_t xMutex = NULL;

// Function prototypes
void bmkgTask(void* parameter);
void alertTask(void* parameter);

void setup() {
  pinMode(2, OUTPUT);  // Built-in LED in ESP32

  // Start the Serial Monitor
  Serial.begin(115200);

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to Wi-Fi");

  // Initialize UDP for NTP
  udp.begin(ntpPort);

  // Create a mutex for shared resources
  xMutex = xSemaphoreCreateMutex();

  // Fetch current location from OpenWeatherMap
  Serial.println("Getting Current Location....");
  String geoResponse = apiRequest(apiUrl_geo);
  parseJsonLocation(geoResponse);

  // Create tasks
  xTaskCreate(
    bmkgTask,        // Task function
    "BMKG Task",     // Task name
    10000,           // Stack size (in words)
    NULL,            // Task input parameter
    1,               // Task priority
    &bmkgTaskHandle  // Task handle
  );

  xTaskCreate(
    alertTask,        // Task function
    "Alert Task",     // Task name
    10000,            // Stack size (in words)
    NULL,             // Task input parameter
    2,                // Task priority (higher priority)
    &alertTaskHandle  // Task handle
  );
}

void loop() {
  // FreeRTOS tasks handle everything, so the loop is empty
  vTaskDelay(portMAX_DELAY);  // Keep the loop running
}

// BMKG Task: Fetch and parse BMKG data
void bmkgTask(void* parameter) {
  while (1) {
    // Get the current time from the NTP server
    epochTime = getNtpTime();

    if (epochTime != 0) {
      // Adjust for WIB (UTC+7)
      epochTime += timeOffset;

      // Fetch BMKG data
      String bmkgResponse = apiRequest(apiUrl_bmkg + epochTime);

      Serial.print("dist: ");
      Serial.println(distance);

      // Parse BMKG data
      if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
        parseJsonBMKG(bmkgResponse);
        xSemaphoreGive(xMutex);
      }
    }

    vTaskDelay(2000 / portTICK_PERIOD_MS);  // Run every 2 seconds
  }
}

// Alert Task: Handle alerts based on distance and magnitude
void alertTask(void* parameter) {
  while (1) {
    if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
      distance = calculateDistance(currentGeo[0], currentGeo[1], targetGeo[0], targetGeo[1]);
      // Check if conditions for alert are met
      if (distance < threshold_dist && mag > threshold_mag) {
        digitalWrite(2, HIGH);  // Turn on LED
        vTaskDelay(500 / portTICK_PERIOD_MS);
        digitalWrite(2, LOW);  // Turn off LED
        vTaskDelay(500 / portTICK_PERIOD_MS);
      } else {
        digitalWrite(2, LOW);  // Ensure LED is off
      }

      xSemaphoreGive(xMutex);
    }

    vTaskDelay(10 / portTICK_PERIOD_MS);  // Run every 10 ms
  }
}

// Function to get the current time from the NTP server
unsigned long getNtpTime() {
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  packetBuffer[0] = 0b11100011;  // LI, Version, Mode
  packetBuffer[1] = 0;           // Stratum, or type of clock
  packetBuffer[2] = 6;           // Polling Interval
  packetBuffer[3] = 0xEC;        // Peer Clock Precision
  packetBuffer[12] = 49;
  packetBuffer[13] = 0x4E;
  packetBuffer[14] = 49;
  packetBuffer[15] = 52;

  udp.beginPacket(ntpServer, ntpPort);
  udp.write(packetBuffer, NTP_PACKET_SIZE);
  udp.endPacket();

  delay(1000);

  if (udp.parsePacket()) {
    udp.read(packetBuffer, NTP_PACKET_SIZE);
    unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
    unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
    unsigned long secsSince1900 = (highWord << 16) | lowWord;
    const unsigned long seventyYears = 2208988800UL;
    return secsSince1900 - seventyYears;
  }

  return 0;
}

// Function to parse the JSON response from BMKG
void parseJsonBMKG(String json) {
  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, json);

  if (error) {
    Serial.print("Failed to parse JSON: ");
    Serial.println(error.c_str());
    return;
  }

  JsonArray features = doc["features"];
  for (JsonObject feature : features) {
    JsonObject properties = feature["properties"];
    JsonObject geometry = feature["geometry"];
    JsonArray coordinates = geometry["coordinates"];

    targetGeo[0] = atof(coordinates[1]);  // Latitude
    targetGeo[1] = atof(coordinates[0]);  // Longitude
    mag = atof(properties["mag"]);
    epicenter = properties["place"].as<String>();

    Serial.print("Earthquake Location: ");
    Serial.print(targetGeo[0]);
    Serial.print(", ");
    Serial.println(targetGeo[1]);
    Serial.print("Magnitude: ");
    Serial.println(mag);
    Serial.print("Epicenter: ");
    Serial.println(epicenter);
  }
}

// Function to parse the JSON response from OpenWeatherMap
void parseJsonLocation(String json) {
  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, json);

  if (error) {
    Serial.print("Failed to parse JSON: ");
    Serial.println(error.c_str());
    return;
  }

  JsonObject location = doc[0];
  currentGeo[0] = location["lat"];
  currentGeo[1] = location["lon"];

  Serial.print("Current Location: ");
  Serial.print(currentGeo[0]);
  Serial.print(", ");
  Serial.println(currentGeo[1]);
}

// Function to make an API request
String apiRequest(String link) {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    http.begin(link);
    int httpResponseCode = http.GET();

    if (httpResponseCode == HTTP_CODE_OK) {
      String payload = http.getString();
      http.end();
      return payload;
    } else {
      Serial.print("HTTP Request failed. Error code: ");
      Serial.println(httpResponseCode);
      http.end();
      return "ERROR: HTTP Request";
    }
  } else {
    Serial.println("Wi-Fi disconnected. Reconnecting...");
    WiFi.begin(ssid, password);
  }
  return "";
}

// Function to calculate distance using Haversine formula
double calculateDistance(double lat1, double lon1, double lat2, double lon2) {
  const double R = 6371.0;
  lat1 = lat1 * M_PI / 180.0;
  lon1 = lon1 * M_PI / 180.0;
  lat2 = lat2 * M_PI / 180.0;
  lon2 = lon2 * M_PI / 180.0;

  double dLat = lat2 - lat1;
  double dLon = lon2 - lon1;

  double a = sin(dLat / 2) * sin(dLat / 2) + cos(lat1) * cos(lat2) * sin(dLon / 2) * sin(dLon / 2);
  double c = 2 * atan2(sqrt(a), sqrt(1 - a));
  double distance = R * c;

  // Serial.print("Distance: ");
  // Serial.println(distance);
  return distance;
}
