/*
  Dual-Channel PPG Receiver & SpO2 Calculator
  ------------------------------------------------------
  Hardware:
  - A0: Low-Pass Filtered Signal (DC Component)
  - A1: High-Pass Filtered Signal (AC Component)
  
  Logic:
  1. RED Phase (10s): Sample A0 (Mean) and A1 (Peak-to-Peak). Plot A1.
  2. IR Phase (10s):  Sample A0 (Mean) and A1 (Peak-to-Peak). Plot A1.
  3. Calculate SpO2 using (AC_ptp / DC_mean) ratio.
  ------------------------------------------------------
*/

const int pinDC = A0; // Low-pass filtered input
const int pinAC = A1; // High-pass filtered input

// TIMING (Must match Sender)
const unsigned long PHASE_PERIOD_MS = 10000UL; 
const unsigned long SAMPLE_INTERVAL_MS = 25UL; // 40Hz sampling rate

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  
  // Optional: Print a header for CSV capture if needed
  // Serial.println("Signal,Time");
}

void loop() {
  
  // ==========================================
  // PHASE 1: RED (Assumed First)
  // ==========================================
  
  long  dcSum = 0;
  int   acMin = 1023;
  int   acMax = 0;
  long  sampleCount = 0;
  
  unsigned long startPhase = millis();
  unsigned long nextSample = startPhase;

  // Signal start of cycle
  digitalWrite(LED_BUILTIN, HIGH); 
  delay(50); 
  digitalWrite(LED_BUILTIN, LOW);

  while (millis() - startPhase < PHASE_PERIOD_MS) {
    if (millis() >= nextSample) {
      int valDC = analogRead(pinDC);
      int valAC = analogRead(pinAC);
      
      // 1. PLOT the PPG Pulse (AC signal)
      // This allows you to see the heartbeat in Serial Plotter
      // Plot both AC and DC
      Serial.print("Red:  AC:");
      Serial.print(valAC);
      Serial.print(",");      // Comma separator
      Serial.print("DC:");
      Serial.println(valDC);  // New line at the end
      
      // Optional: Plot DC on a secondary scale if desired, 
      // but usually they are too far apart in value.
      // Serial.println(); 

      // 2. Accumulate Statistics
      // DC: Calculate Mean
      dcSum += valDC;
      
      // AC: Find Peak-to-Peak
      if (valAC < acMin) acMin = valAC;
      if (valAC > acMax) acMax = valAC;
      
      sampleCount++;
      nextSample += SAMPLE_INTERVAL_MS;
    }
  }

  // Compute RED parameters
  float redDC_mean = (float)dcSum / sampleCount;
  float redAC_ptp  = (float)(acMax - acMin);

  // ==========================================
  // PHASE 2: IR
  // ==========================================
  
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
      
      // 1. PLOT
      // Plot both AC and DC
      Serial.print("IR:  AC:");
      Serial.print(valAC);
      Serial.print(",");      // Comma separator
      Serial.print("DC:");
      Serial.println(valDC);  // New line at the end

      // 2. Accumulate Statistics
      dcSum += valDC;
      
      if (valAC < acMin) acMin = valAC;
      if (valAC > acMax) acMax = valAC;
      
      sampleCount++;
      nextSample += SAMPLE_INTERVAL_MS;
    }
  }

  // Compute IR parameters
  float irDC_mean = (float)dcSum / sampleCount;
  float irAC_ptp  = (float)(acMax - acMin);

  // ==========================================
  // CALCULATE & PRINT
  // ==========================================

  // Safety check to prevent division by zero
  if (redDC_mean < 1 || irDC_mean < 1 || irAC_ptp < 1) {
    Serial.println("Error: Signal too weak or DC missing.");
    return;
  }

  // Ratio R = (AC_red / DC_red) / (AC_ir / DC_ir)
  float ratioRED = redAC_ptp / redDC_mean;
  float ratioIR  = irAC_ptp / irDC_mean;
  
  float R = ratioRED / ratioIR;
  
  // Standard calibration curve (adjust -25.0 and 110.0 based on calibration)
  float spo2 = 110.0 - (25.0 * R);

  // Clamp limits
  if (spo2 > 100) spo2 = 100;
  if (spo2 < 0) spo2 = 0;

  Serial.println("--- RESULT ---");
  Serial.print("RED: DC="); Serial.print(redDC_mean); Serial.print(" AC(ptp)="); Serial.println(redAC_ptp);
  Serial.print("IR : DC="); Serial.print(irDC_mean);  Serial.print(" AC(ptp)="); Serial.println(irAC_ptp);
  Serial.print("R Value: "); Serial.println(R, 3);
  Serial.print("SpO2: "); Serial.print(spo2); Serial.println("%");
  Serial.println("--------------");
}