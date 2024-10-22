#include <Wire.h>
#include "MAX30102.h"
#include <WiFi.h>
#include <HTTPClient.h>

// Pin Definitions
#define CUSTOM_SDA 21
#define CUSTOM_SCL 22
#define FORCE_SENSOR_PIN 33
#define FORCE_SENSOR_PINO 26 
#define FORCE_SENSOR_PINL 27 
#define PIN_LM35 25
// WiFi Credentials
const char* ssid = "TECNO Pouvoir 3 Air";       // Replace with your WiFi SSID
const char* password = "no password"; // Replace with your WiFi password
// Server URL (replace with your server endpoint)
const char* serverURL = "http://your-server-endpoint.com/api/data";

// Constants
#define ADC_VREF_mV 3300.0  // Changed to 3.3V which is typical for ESP32
#define ADC_RESOLUTION 4096.0
#define SAMPLE_SIZE 10      // Number of samples for averaging

// Create MAX30102 instance
MAX30102 particleSensor(CUSTOM_SDA, CUSTOM_SCL);

// Variables for heart rate detection
unsigned long lastBeatTime = 0;
long bpm = 0;
long avgBPM = 0;
int beatCount = 0;
long beatSum = 0;

// Calibration variables
int fsrBaseline[3] = {0, 0, 0};
float tempOffset = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n\nStarting sensor initialization...");
    // Connect to WiFi
  connectToWiFi();
  // Initialize MAX30102
  Serial.println("Initializing MAX30102...");
  if (!particleSensor.begin()) {
    Serial.println("MAX30102 not found. Check wiring.");
  } else {
    Serial.println("MAX30102 initialized successfully");
  }

  // Calibration
  calibrateSensors();

  Serial.println("Setup and calibration completed. Starting main loop...\n");
}
void connectToWiFi() {
  Serial.printf("Connecting to %s ...\n", ssid);
  WiFi.begin(ssid, password);
  
  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  
  Serial.println("\nWiFi connected!");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void calibrateSensors() {
  Serial.println("Calibrating sensors...");
  
  // Calibrate FSR sensors
  long fsrSum[3] = {0, 0, 0};
  for(int i = 0; i < SAMPLE_SIZE; i++) {
    fsrSum[0] += analogRead(FORCE_SENSOR_PIN);
    fsrSum[1] += analogRead(FORCE_SENSOR_PINO);
    fsrSum[2] += analogRead(FORCE_SENSOR_PINL);
    delay(100);
  }
  
  for(int i = 0; i < 3; i++) {
    fsrBaseline[i] = fsrSum[i] / SAMPLE_SIZE;
    Serial.printf("FSR %d baseline: %d\n", i+1, fsrBaseline[i]);
  }

  // Calibrate temperature sensor
  long tempSum = 0;
  for(int i = 0; i < SAMPLE_SIZE; i++) {
    tempSum += analogRead(PIN_LM35);
    delay(100);
  }
  int tempBaseline = tempSum / SAMPLE_SIZE;
  float expectedTemp = 25.0; // Assume room temperature is 25°C
  float measuredTemp = (tempBaseline * ADC_VREF_mV / ADC_RESOLUTION) / 10.0;
  tempOffset = expectedTemp - measuredTemp;
  
  Serial.printf("Temperature offset: %.2f°C\n", tempOffset);
}

int readFSR(int pin, int baseline) {
  long sum = 0;
  for(int i = 0; i < SAMPLE_SIZE; i++) {
    sum += analogRead(pin);
    delay(5);
  }
  int average = sum / SAMPLE_SIZE;
  return max(0, average - baseline);
}

void readFSRSensors() {
  Serial.println("\n=== Force Sensor Readings ===");
  
  int fsrPins[3] = {FORCE_SENSOR_PIN, FORCE_SENSOR_PINO, FORCE_SENSOR_PINL};
  int readings[3];
  
  for (int i = 0; i < 3; i++) {
    readings[i] = readFSR(fsrPins[i], fsrBaseline[i]);
    
    Serial.printf("FSR %d Calibrated Value: %d\n", i+1, readings[i]);
    
    String pressure;
    if (readings[i] < 50) pressure = "No pressure";
    else if (readings[i] < 500) pressure = "Light touch";
    else if (readings[i] < 1500) pressure = "Light squeeze";
    else if (readings[i] < 2500) pressure = "Medium squeeze";
    else pressure = "Big squeeze";
    
    Serial.printf("FSR %d Status: %s\n", i+1, pressure.c_str());
  }
}

void readTemperature() {
  Serial.println("\n=== Temperature Readings ===");
  
  long tempSum = 0;
  for(int i = 0; i < SAMPLE_SIZE; i++) {
    tempSum += analogRead(PIN_LM35);
    delay(10);
  }
  int avgReading = tempSum / SAMPLE_SIZE;
  
  float milliVolt = avgReading * (ADC_VREF_mV / ADC_RESOLUTION);
  float tempC = (milliVolt / 10.0) + tempOffset;
  float tempF = tempC * 9 / 5 + 32;

  Serial.printf("Calibrated Temperature: %.1f°C / %.1f°F\n", tempC, tempF);
}

void readHeartRate() {
  Serial.println("\n=== Heart Rate Readings ===");
  
  long irSum = 0;
  int validReadings = 0;
  
  // Take multiple readings
  for(int i = 0; i < SAMPLE_SIZE; i++) {
    long ir = particleSensor.readIR();
    if (ir > 5000) {  // Only count readings that might be valid
      irSum += ir;
      validReadings++;
    }
    delay(10);
  }
  
  if (validReadings < SAMPLE_SIZE / 2) {
    Serial.println("No finger detected or poor signal");
    return;
  }
  
  long avgIR = irSum / validReadings;
  Serial.printf("Average IR Value: %ld\n", avgIR);

  float processedIRValue = particleSensor.processSensorData(avgIR);
  
  if (particleSensor.isValidSignal(processedIRValue)) {
    detectHeartRate(processedIRValue);
    if (bpm >= 40 && bpm <= 200) {  // Only show reasonable heart rates
      Serial.printf("Current BPM: %ld\n", getBPM());
      Serial.printf("Average BPM: %ld\n", getAvgBPM());
    } else {
      Serial.println("Heart rate outside normal range, might be inaccurate");
    }
  } else {
    Serial.println("Invalid heart rate signal");
  }
}

void detectHeartRate(float processedValue) {
  unsigned long currentTime = millis();
  if (currentTime - lastBeatTime > 500) {
    long IBI = currentTime - lastBeatTime;
    bpm = 60000 / IBI;
    lastBeatTime = currentTime;
    
    // Only update average if the BPM is reasonable
    
      beatCount++;
      beatSum += bpm;
      avgBPM = beatSum / beatCount;
    
  }
}

long getBPM() {
  return bpm;
}

long getAvgBPM() {
  return avgBPM;
}

void loop() {
  Serial.println("\n\n====== Starting New Reading Cycle ======");
  
  readFSRSensors();
  readTemperature();
  readHeartRate();
  
  Serial.println("\n====== Reading Cycle Complete ======");
  Serial.println("Waiting 2 seconds before next cycle...");
  
  delay(2000);
}