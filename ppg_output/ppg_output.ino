/*
  Dual-Channel PPG Receiver - Persistent SpO2 Display
  ------------------------------------------------------
  Hardware:
  - A0: Low-Pass Filtered Signal (DC Component)
  - A1: High-Pass Filtered Signal (AC Component)
  - I2C OLED Display (SDA -> A4, SCL -> A5 on Uno/Nano)
  ------------------------------------------------------
*/

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// --- OLED CONFIGURATION ---
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1 
#define SCREEN_ADDRESS 0x3C 
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

const int pinDC = A0; // Low-pass filtered input
const int pinAC = A1; // High-pass filtered input

// TIMING
const unsigned long PHASE_PERIOD_MS = 10000UL; 
const unsigned long SAMPLE_INTERVAL_MS = 25UL; 

// --- GLOBAL VARIABLES (To hold data between loops) ---
float currentSpO2 = 0.0;
float currentR = 0.0;

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);

  // --- OLED SETUP ---
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }
  
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 20);
  display.println(F("System Initializing..."));
  display.display();
  delay(2000);
}

void loop() {
  
  // ==========================================
  // PHASE 1: RED
  // ==========================================
  
  // Show the LAST calculated SpO2 while we measure RED
  updateDisplay("Reading: RED", currentSpO2);

  long  dcSum = 0;
  int   acMin = 1023;
  int   acMax = 0;
  long  sampleCount = 0;
  
  unsigned long startPhase = millis();
  unsigned long nextSample = startPhase;

  // Sync Signal (Blink)
  digitalWrite(LED_BUILTIN, HIGH); 
  delay(50); 
  digitalWrite(LED_BUILTIN, LOW);

  while (millis() - startPhase < PHASE_PERIOD_MS) {
    if (millis() >= nextSample) {
      int valDC = analogRead(pinDC);
      int valAC = analogRead(pinAC);
      
      // Plot for Serial Plotter
      Serial.print("AC:"); Serial.print(valAC);
      Serial.print(",");      
      Serial.print("DC:"); Serial.println(valDC);  
      
      dcSum += valDC;
      if (valAC < acMin) acMin = valAC;
      if (valAC > acMax) acMax = valAC;
      
      sampleCount++;
      nextSample += SAMPLE_INTERVAL_MS;
    }
  }

  float redDC_mean = (float)dcSum / sampleCount;
  float redAC_ptp  = (float)(acMax - acMin);

  // ==========================================
  // PHASE 2: IR
  // ==========================================
  
  // Show the LAST calculated SpO2 while we measure IR
  updateDisplay("Reading: IR", currentSpO2);
  
  dcSum = 0;
  acMin = 1023;
  acMax = 0;
  sampleCount = 0;

  startPhase = millis();
  nextSample = startPhase;

  while (millis() - startPhase < PHASE_PERIOD_MS) {
    if (millis() >= nextSample) {
      int valDC = analogRead(pinDC);
      int valAC = analogRead(pinAC);
      
      Serial.print("AC:"); Serial.print(valAC);
      Serial.print(",");      
      Serial.print("DC:"); Serial.println(valDC); 

      dcSum += valDC;
      if (valAC < acMin) acMin = valAC;
      if (valAC > acMax) acMax = valAC;
      
      sampleCount++;
      nextSample += SAMPLE_INTERVAL_MS;
    }
  }

  float irDC_mean = (float)dcSum / sampleCount;
  float irAC_ptp  = (float)(acMax - acMin);

  // ==========================================
  // CALCULATE
  // ==========================================

  // Basic validation
  if (redDC_mean > 1 && irDC_mean > 1 && irAC_ptp > 1) {
      
      float ratioRED = redAC_ptp / redDC_mean;
      float ratioIR  = irAC_ptp / irDC_mean;
      
      // Update global variables
      currentR = ratioRED / ratioIR;
      float calculated_spo2 = 110.0 - (25.0 * currentR);

      // Clamp limits
      if (calculated_spo2 > 100) calculated_spo2 = 100;
      if (calculated_spo2 < 0) calculated_spo2 = 0;
      
      currentSpO2 = calculated_spo2;
  } else {
      // If signal is bad, we might want to keep showing the old value 
      // or indicate an error. For now, we just print error to Serial.
      Serial.println("Signal Error - Values not updated");
  }

  // Print results to Serial
  Serial.println("--- UPDATE ---");
  Serial.print("R: "); Serial.println(currentR, 3);
  Serial.print("SpO2: "); Serial.println(currentSpO2); 
  Serial.println("--------------");
  
  // Update screen immediately with the NEW value
  updateDisplay("Updated!", currentSpO2);
  
  // Small delay to let the user see "Updated!" before it switches back to "Reading RED"
  delay(1000); 
}

// ------------------------------------------------
// HELPER FUNCTION
// ------------------------------------------------

void updateDisplay(String statusMsg, float spo2Value) {
  display.clearDisplay();
  
  // --- TOP BAR (Status) ---
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print(F("Status: "));
  display.println(statusMsg);
  
  // Draw a line under the status
  display.drawLine(0, 10, 128, 10, SSD1306_WHITE);
  
  // --- MIDDLE (SpO2 Label) ---
  display.setCursor(0, 20);
  display.setTextSize(1);
  display.println(F("Latest SpO2:"));

  // --- BOTTOM (Big Number) ---
  display.setCursor(10, 35);
  display.setTextSize(3); // Large font
  display.print((int)spo2Value);
  display.setTextSize(2);
  display.print(F(" %"));
  
  display.display();
}