// ------------------------------------------------------
// Long-period RED / IR pulse oximeter sketch
// RED on for 40 s → sample, compute DC_red & AC_red
// IR  on for 40 s → sample, compute DC_ir  & AC_ir
// Then compute R and SpO₂
// ------------------------------------------------------

const int ledRed = 9;
const int ledIR  = 10;
const int adcPin = A0;

const unsigned long RED_PERIOD_MS = 10000UL;   // 40 seconds
const unsigned long IR_PERIOD_MS  = 10000UL;   // 40 seconds
const unsigned long SAMPLE_INTERVAL_MS = 25UL; 

void setup() {
  pinMode(ledRed, OUTPUT);
  pinMode(ledIR,  OUTPUT);
  digitalWrite(ledRed, LOW);
  digitalWrite(ledIR,  LOW);
  Serial.begin(115200);
}

void loop() {
  // --- RED phase ---
  digitalWrite(ledRed, HIGH);
  digitalWrite(ledIR,  LOW);

  unsigned long t0 = millis();
  unsigned long nextSample = t0;
  
  // Variables to collect RED measurements
  const int MAX_SAMPLES = RED_PERIOD_MS / SAMPLE_INTERVAL_MS + 10;
  float redSum = 0;
  float redSqSum = 0;
  int   redMin = 1023, redMax = 0;
  int   redCount = 0;

  while (millis() - t0 < RED_PERIOD_MS) {
    unsigned long now = millis();
    if (now >= nextSample) {
      int v = analogRead(adcPin);
      redSum   += v;
      redSqSum += (float)v * v;
      if (v < redMin) redMin = v;
      if (v > redMax) redMax = v;
      redCount++;
      nextSample += SAMPLE_INTERVAL_MS;
    }
    // optionally do other non-blocking tasks here
  }
  digitalWrite(ledRed, LOW);

  float dc_red = redSum / redCount;                // average  
  float ac_red_ptp = redMax - redMin;             // peak-to-peak  
  float ac_red_rms = sqrt( (redSqSum / redCount) - dc_red*dc_red );  // RMS of AC (approx)

  Serial.print("RED: DC = "); Serial.print(dc_red,2);
  Serial.print("  AC_ptp = "); Serial.print(ac_red_ptp,2);
  Serial.print("  AC_rms = "); Serial.println(ac_red_rms,2);

  delay(20); // short pause before next phase (if desired)

  // --- IR phase ---
  digitalWrite(ledRed, LOW);
  digitalWrite(ledIR,  HIGH);

  t0 = millis();
  nextSample = t0;

  float irSum = 0;
  float irSqSum = 0;
  int   irMin = 1023, irMax = 0;
  int   irCount = 0;

  while (millis() - t0 < IR_PERIOD_MS) {
    unsigned long now = millis();
    if (now >= nextSample) {
      int v = analogRead(adcPin);
      irSum   += v;
      irSqSum += (float)v * v;
      if (v < irMin) irMin = v;
      if (v > irMax) irMax = v;
      irCount++;
      nextSample += SAMPLE_INTERVAL_MS;
    }
    // optionally do other non-blocking tasks here
  }
  digitalWrite(ledIR, LOW);

  float dc_ir = irSum / irCount;
  float ac_ir_ptp = irMax - irMin;
  float ac_ir_rms = sqrt( (irSqSum / irCount) - dc_ir*dc_ir );

  Serial.print("IR : DC = "); Serial.print(dc_ir,2);
  Serial.print("  AC_ptp = "); Serial.print(ac_ir_ptp,2);
  Serial.print("  AC_rms = "); Serial.println(ac_ir_rms,2);

  // --- Compute ratio & SpO₂ (using peak-to-peak or rms AC) ---
  // Using PTP-based AC:
  float R_ptp = (ac_red_ptp / dc_red) / (ac_ir_ptp / dc_ir);
  float spo2_ptp = 110.0 - 25.0 * R_ptp;

  // Using RMS-based AC:
  float R_rms = (ac_red_rms / dc_red) / (ac_ir_rms / dc_ir);
  float spo2_rms = 110.0 - 25.0 * R_rms;

  Serial.print("SpO2 (ptp) = "); Serial.print(spo2_ptp,1);
  Serial.print("  SpO2 (rms) = "); Serial.println(spo2_rms,1);

  delay(20);  // wait a bit before starting next RED cycle
}
