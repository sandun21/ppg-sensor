// ------------------------------------------------------
// Pulse-Oximeter Sketch (RED/IR multiplex + ADC + HR + SpO₂)
// Based on your switching logic + added processing
// ------------------------------------------------------
const int ledRed  = 9;
const int ledIR   = 10;
const int adcPin  = A0;

// const unsigned long onTime        = 220UL;    // µs LED ON
// const unsigned long cooloffTime   = 320UL;    // µs LED OFF / settle
// const unsigned long offTime       = 1780UL;   // µs total off interval
const unsigned long redOonTime    = 220UL;    // µs LED ON
const unsigned long cooloffTime   = 320UL;    // µs LED OFF / settle
const unsigned long offTime       = 1780UL;   // µs total off interval
const unsigned long processingTime= offTime - cooloffTime - onTime;

unsigned long lastSwitchTime = 0;
enum Phase { RED_ON, RED_OFF, IR_ON, IR_OFF };
Phase phase = RED_ON;
unsigned long phaseStartTime = 0;

const int WINDOW_SIZE = 50;  // number of cycles to buffer for AC/DC
float redVals[WINDOW_SIZE];
float irVals [WINDOW_SIZE];
int   idx = 0;
bool  bufferFilled = false;

// For heart-rate detection (simple peak detection on IR channel)
unsigned long lastBeatTime = 0;
float lastIrVal = 0;
float beatInterval = 0;
float heartRate = 0;

void setup() {
  pinMode(ledRed, OUTPUT);
  pinMode(ledIR,  OUTPUT);
  digitalWrite(ledRed, LOW);
  digitalWrite(ledIR,  LOW);

  Serial.begin(115200);
  phaseStartTime = micros();
  lastSwitchTime = phaseStartTime;
}

void loop() {
  unsigned long now = micros();
  switch (phase) {
    case RED_ON:
      digitalWrite(ledRed, HIGH);
      digitalWrite(ledIR,  LOW);
      if (now - phaseStartTime >= onTime) {
        // read ADC for RED
        int raw = analogRead(adcPin);
        redVals[idx] = raw;
        // move to next phase
        phase = RED_OFF;
        phaseStartTime = now;
      }
      break;

    case RED_OFF:
      digitalWrite(ledRed, LOW);
      digitalWrite(ledIR,  LOW);
      if (now - phaseStartTime >= cooloffTime){
        phase = IR_ON;
        phaseStartTime = now;
      }
      break;

    case IR_ON:
      digitalWrite(ledRed, LOW);
      digitalWrite(ledIR,  HIGH);
      if (now - phaseStartTime >= onTime) {
        // read ADC for IR
        int raw = analogRead(adcPin);
        irVals[idx] = raw;

        // heart-rate: simple threshold/edge detection
        float currentIr = raw;
        if (lastIrVal > 0) {
          // detect upward edge: simple heuristic
          if (currentIr > lastIrVal + 20) {  // threshold
            unsigned long t = now;
            if (lastBeatTime > 0) {
              beatInterval = (t - lastBeatTime) * 1e-6; // seconds
              heartRate = 60.0 / beatInterval; // bpm
            }
            lastBeatTime = t;
          }
        }
        lastIrVal = currentIr;

        // update buffer index
        idx++;
        if (idx >= WINDOW_SIZE) {
          idx = 0;
          bufferFilled = true;
        }

        // move to next phase
        phase = IR_OFF;
        phaseStartTime = now;
      }
      break;

    case IR_OFF:
      digitalWrite(ledRed, LOW);
      digitalWrite(ledIR,  LOW);
      if (now - phaseStartTime >= processingTime) {
        // buffer full? compute SpO₂
        if (bufferFilled) {
          // compute DC & AC for RED
          float sumR = 0, minR = redVals[0], maxR = redVals[0];

          for (int i = 0; i < WINDOW_SIZE; i++) {
            float v = redVals[i];
            sumR += v;
            if (v < minR) minR = v;
            if (v > maxR) maxR = v;
          }

          float dc_red = sumR / WINDOW_SIZE;
          float ac_red = maxR - minR;

          // compute DC & AC for IR
          float sumIR = 0, minIR = irVals[0], maxIR = irVals[0];
          for (int i = 0; i < WINDOW_SIZE; i++) {
            float v = irVals[i];
            sumIR += v;
            if (v < minIR) minIR = v;
            if (v > maxIR) maxIR = v;
          }
          float dc_ir = sumIR / WINDOW_SIZE;
          float ac_ir = maxIR - minIR;

          // ratio and SpO₂ estimation
          float R = (ac_red / dc_red) / (ac_ir / dc_ir);
          float spo2 = 110.0 - 25.0 * R;
          if (spo2 > 100) spo2 = 100;
          if (spo2 < 0)   spo2 = 0;

          // print results
          Serial.print("HR(bpm): ");   Serial.print(heartRate,1);
          Serial.print("  SpO2(%): "); Serial.println(spo2,1);
        }

        phase = RED_ON;
        phaseStartTime = now;
      }
      break;
  }
}
