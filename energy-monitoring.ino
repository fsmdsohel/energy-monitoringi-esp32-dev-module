// === Include Libraries ===
#include <ZMPT101B.h>

// === Analog Input Pins ===
const int currentPin = 32;    // GPIO32 for current sensor
const int voltagePin = 34;    // GPIO34 for ZMPT101B voltage sensor

// === ADC Settings ===
const float ADC_REF_VOLTAGE = 3.3;
const int ADC_RESOLUTION = 4095;
const int SAMPLES = 200;

// === Calibration Data (Change these based on your calibration results) ===
const int NUM_POINTS = 3;

// Calibration: RMS voltage output of current sensor vs actual current (Amps)
float currentVoltagePoints[NUM_POINTS] = {0.005, 0.159, 0.945};
float actualCurrentPoints[NUM_POINTS] = {0.000, 0.087, 0.860};

// Voltage calibration points
float voltageVoltagePoints[NUM_POINTS] = {5, 220, 230}; // RMS sensor voltages
float actualVoltagePoints[NUM_POINTS] = {0.0, 220.0, 230.0};   // Actual AC voltages

// === ZMPT101B Object ===
// Used only for getting raw readings
ZMPT101B voltageSensor(voltagePin, 50.0);

void setup() {
  Serial.begin(115200);
  analogReadResolution(12); // 12-bit resolution for ESP32 ADC
}

void loop() {
  // === Read Current Sensor ===
  float currentOffset = getDCOffset(currentPin);
  float currentRMSVoltage = getRMSVoltage(currentPin, currentOffset);
  float current = interpolate(currentRMSVoltage, currentVoltagePoints, actualCurrentPoints);

  // === Read Voltage Sensor using raw data method and interpolation ===

  float rmsVoltage = voltageSensor.getRmsVoltage();

  float voltage = interpolate(rmsVoltage, voltageVoltagePoints, actualVoltagePoints);
  
  // === Calculate Power (Watts) ===
  float power = voltage * current;
  
  // === Output to Serial ===
  Serial.print("Voltage: ");
  Serial.print(voltage, 1);
  Serial.print(" V  |  RMS Sensor Voltage: ");
  Serial.print(rmsVoltage, 3);
  
  Serial.print("  ||  Current: ");
  Serial.print(current, 3);
  Serial.print(" A  |  RMS Sensor Voltage: ");
  Serial.print(currentRMSVoltage, 3);
  Serial.print(" V");
  
  Serial.print("  ||  Power: ");
  Serial.print(power, 1);
  Serial.println(" W");

  delay(1000);
}

// === Get DC Offset (Center of Sine Wave) ===
float getDCOffset(int pin) {
  long total = 0;
  for (int i = 0; i < SAMPLES; i++) {
    total += analogRead(pin);
    delayMicroseconds(200);
  }
  return ((float)total / SAMPLES) * (ADC_REF_VOLTAGE / ADC_RESOLUTION);
}

// === Get RMS of Centered Signal ===
float getRMSVoltage(int pin, float offset) {
  float sumSquares = 0;
  for (int i = 0; i < SAMPLES; i++) {
    int adcValue = analogRead(pin);
    float voltage = adcValue * (ADC_REF_VOLTAGE / ADC_RESOLUTION);
    float centered = voltage - offset;
    sumSquares += centered * centered;
    delayMicroseconds(200);
  }
  float meanSquare = sumSquares / SAMPLES;
  return sqrt(meanSquare);
}

// === Interpolation Function for Mapping Sensor Voltage to Real Value ===
float interpolate(float voltage, float* voltagePoints, float* valuePoints) {
  if (voltage <= voltagePoints[0]) return valuePoints[0];

  for (int i = 0; i < NUM_POINTS - 1; i++) {
    if (voltage >= voltagePoints[i] && voltage <= voltagePoints[i + 1]) {
      float slope = (valuePoints[i + 1] - valuePoints[i]) / 
                    (voltagePoints[i + 1] - voltagePoints[i]);
      return valuePoints[i] + slope * (voltage - voltagePoints[i]);
    }
  }

  // Extrapolate beyond last point
  float v1 = voltagePoints[NUM_POINTS - 2];
  float v2 = voltagePoints[NUM_POINTS - 1];
  float c1 = valuePoints[NUM_POINTS - 2];
  float c2 = valuePoints[NUM_POINTS - 1];
  float slope = (c2 - c1) / (v2 - v1);
  return c2 + slope * (voltage - v2);
}
