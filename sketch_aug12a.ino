
#include <Wire.h>
#include <math.h>

#include <MAX3010x.h>
#include "filters.h"

#include <LiquidCrystal_I2C.h>
#include <MPU6050_light.h>

#include <OneWire.h>
#include <DallasTemperature.h>

// ---------------- Serial TX rate control ----------------
#define SEND_PERIOD_MS   1000   // <<--- change this to slow down/speed up Python consumption

// ---------------- Pins ----------------
#define SOUND_PIN    2   // digital sound sensor (UNO D2)
#define DS18B20_PIN  4   // DS18B20 pin

// ---------------- Thresholds ----------------
const int   HR_MIN_OK        = 50;
const int   HR_MAX_OK        = 120;
const float TEMP_HIGH_C      = 35.0;
const float TILT_ANGLE_DEG   = 60.0;
const unsigned long TILT_HOLD_MS       = 1000;
const unsigned long FALL_LATCH_TIME_MS = 3000;

// ---------------- PPG (MAX3010x) ----------------
MAX30105 sensor;
const auto  kSamplingRate      = sensor.SAMPLING_RATE_400SPS;
const float kSamplingFrequency = 400.0;
const unsigned long kFingerThreshold  = 10000;
const unsigned int  kFingerCooldownMs = 500;
const float kEdgeThreshold            = -2000.0;

// ---------------- Filters ----------------
const float kLowPassCutoff  = 5.0;
const float kHighPassCutoff = 0.5;

// ---------------- Globals ----------------
LiquidCrystal_I2C lcd(0x27, 16, 2);
MPU6050 mpu(Wire);
OneWire oneWire(DS18B20_PIN);
DallasTemperature dallas(&oneWire);

float tempC = NAN;
unsigned long last_temp_ms = 0;
const unsigned long TEMP_PERIOD_MS = 1000;

unsigned long last_lcd_update = 0;
const unsigned long LCD_UPDATE_INTERVAL_MS = 250;

unsigned long last_tx_ms = 0;

// PPG filters/stats
LowPassFilter low_pass_filter_red(kLowPassCutoff, kSamplingFrequency);
LowPassFilter low_pass_filter_ir(kLowPassCutoff, kSamplingFrequency);
HighPassFilter high_pass_filter(kHighPassCutoff, kSamplingFrequency);
Differentiator differentiator(kSamplingFrequency);
MinMaxAvgStatistic stat_red;
MinMaxAvgStatistic stat_ir;

// SpO2 calibration
float kSpO2_A = 1.5958422;
float kSpO2_B = -34.6596622;
float kSpO2_C = 112.6898759;

// HB detection helpers
long  last_heartbeat = 0;
float last_diff      = NAN;
bool  crossed        = false;
long  crossed_time   = 0;
int   last_bpm       = 0;
float last_spo2      = 0.0;

// Finger detection
long finger_timestamp = 0;
bool finger_detected  = false;

// Sound debounce
const unsigned long SOUND_DEBOUNCE_MS = 200;
unsigned long last_sound_read_ms = 0;
int last_sound_state = 0;

// Simple tilt fall detector
bool tilt_started = false;
unsigned long tilt_start_ms = 0;
bool fall_detected          = false;
unsigned long fall_latched_at = 0;

// ---------------- Helpers ----------------
void readDallasTemp() {
  if (millis() - last_temp_ms >= TEMP_PERIOD_MS) {
    last_temp_ms = millis();
    dallas.requestTemperatures();
    tempC = dallas.getTempCByIndex(0);
  }
}

void showFallOnLCD() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("*** FALL ! ***");
  lcd.setCursor(0, 1);
  lcd.print("TILT DETECTED");
}

void resetFall() {
  tilt_started = false;
  fall_detected = false;
  fall_latched_at = 0;
}

void sendFrameToPython(int bpm, float spo2, float temp, float angleX, int soundState) {
  unsigned long now = millis();
  if (now - last_tx_ms < SEND_PERIOD_MS) return;  // throttle TX
  last_tx_ms = now;

  // Ensure no NaNs are sent
  if (isnan(spo2)) spo2 = 0.0f;
  if (isnan(temp)) temp = 0.0f;

  Serial.print('a'); Serial.print(bpm);
  Serial.print('b'); Serial.print(spo2, 2);
  Serial.print('c'); Serial.print(temp, 2);
  Serial.print('d'); Serial.print(angleX, 2);
  Serial.print('e'); Serial.print(soundState);
  Serial.print('f');
  Serial.print('\n'); // newline for Python readline()
}

// ---------------- Setup ----------------
void setup() {
  Serial.begin(115200);
  Serial.setTimeout(10);
  Wire.begin();

  pinMode(SOUND_PIN, INPUT);
  dallas.begin();

  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Initializing...");

  if (sensor.begin() && sensor.setSamplingRate(kSamplingRate)) {
    Serial.println("MAX3010x: OK");
  } else {
    Serial.println("MAX3010x: FAIL");
    lcd.setCursor(0, 1);
    lcd.print("MAX3010x FAIL");
    while (1);
  }

  byte status = mpu.begin();
  if (status != 0) {
    Serial.print(F("MPU6050 fail code "));
    Serial.println(status);
    lcd.clear();
    lcd.print("MPU6050 FAIL");
  } else {
    mpu.calcOffsets();
    Serial.println("MPU6050: OK");
  }

  lcd.clear();
  lcd.print("Ready");
  delay(500);
  lcd.clear();
}

// ---------------- Main loop ----------------
void loop() {
  unsigned long now = millis();

  // ---- Read temp ----
  readDallasTemp();

  // ---- Read sound pin (debounced) ----
  int soundState = last_sound_state;
  if (now - last_sound_read_ms > SOUND_DEBOUNCE_MS) {
    last_sound_state = digitalRead(SOUND_PIN);
    soundState = last_sound_state;
    last_sound_read_ms = now;
  }

  // ---- PPG sample ----
  auto sample = sensor.readSample(1000);
  float current_value_red = sample.red;
  float current_value_ir  = sample.ir;

  // ---- MPU angleX only ----
  mpu.update();
  float angleX = mpu.getAngleX();

  // ---------------- FALL (tilt only) ----------------
  if (fall_detected) {
    if (now - fall_latched_at > FALL_LATCH_TIME_MS) {
      resetFall();
      lcd.clear();
    }
  } else {
    bool tilted = (fabs(angleX) > TILT_ANGLE_DEG);

    if (tilted && !tilt_started) {
      tilt_started  = true;
      tilt_start_ms = now;
    } else if (!tilted) {
      tilt_started = false;
    }

    if (tilt_started && (now - tilt_start_ms > TILT_HOLD_MS)) {
      fall_detected   = true;
      fall_latched_at = now;
      showFallOnLCD();
      Serial.println("***** FALL DETECTED *****");
    }
  }

  // ---------------- Finger detection ----------------
  if (sample.red > kFingerThreshold) {
    if (millis() - finger_timestamp > kFingerCooldownMs) {
      finger_detected = true;
    }
  } else {
    // reset PPG processing
    differentiator.reset();
    low_pass_filter_red.reset();
    low_pass_filter_ir.reset();
    high_pass_filter.reset();
    stat_red.reset();
    stat_ir.reset();

    finger_detected = false;
    finger_timestamp = millis();
  }

  // ---------------- Heart rate + SpO2 ----------------
  if (finger_detected && !fall_detected) {
    float red_lp = low_pass_filter_red.process(current_value_red);
    float ir_lp  = low_pass_filter_ir.process(current_value_ir);
    float current_value = high_pass_filter.process(red_lp);

    stat_red.process(red_lp);
    stat_ir.process(ir_lp);

    float current_diff = differentiator.process(current_value);

    if (!isnan(current_diff) && !isnan(last_diff)) {
      if (last_diff > 0 && current_diff < 0) {
        crossed = true;
        crossed_time = millis();
      }
      if (current_diff > 0) crossed = false;

      if (crossed && current_diff < kEdgeThreshold) {
        if (last_heartbeat != 0) {
          long interval = crossed_time - last_heartbeat;
          if (interval > 300 && interval < 2000) {
            int bpm = (int)(60000.0 / interval);
            float rred = (stat_red.maximum() - stat_red.minimum()) / stat_red.average();
            float rir  = (stat_ir.maximum() - stat_ir.minimum())  / stat_ir.average();
            float r    = (rir != 0.0f) ? (rred / rir) : 0.0f;
            float spo2 = kSpO2_A * r * r + kSpO2_B * r + kSpO2_C;

            if (bpm > 50 && bpm < 140) {
              last_bpm  = bpm;
              last_spo2 = spo2;
            }
          }
        }

        stat_red.reset();
        stat_ir.reset();
        last_heartbeat = crossed_time;
        crossed = false;
      }
    }
    last_diff = current_diff;
  }

  // ---------------- LCD (throttled) ----------------
  if (millis() - last_lcd_update > LCD_UPDATE_INTERVAL_MS) {
    last_lcd_update = millis();

    if (fall_detected) {
      // latched fall screen already
    } else if (!finger_detected) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Place finger   ");
      lcd.setCursor(0, 1);
      lcd.print("T:");
      if (!isnan(tempC)) lcd.print(tempC, 1); else lcd.print("--");
      lcd.print(" Ax:");
      lcd.print((int)angleX);
    } else {
      int hrOk = (last_bpm >= HR_MIN_OK && last_bpm <= HR_MAX_OK);
      int tempHigh = (!isnan(tempC) && tempC > TEMP_HIGH_C);

      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("BPM:");
      lcd.print(last_bpm);
      lcd.print(hrOk ? " OK" : " BAD");

      lcd.setCursor(0, 1);
      lcd.print("T:");
      if (!isnan(tempC)) lcd.print(tempC, 1); else lcd.print("--");
      lcd.print(tempHigh ? " HIGH" : " OK");
    }
  }

  // ---------------- Send framed line to Python (slow) ----------------
  sendFrameToPython(last_bpm, last_spo2, tempC, angleX, last_sound_state);
}
