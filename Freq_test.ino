#include <Arduino.h>
#include "driver/i2s.h"
#include "sos-iir-filter.h"
#include <arduinoFFT.h>

// Uncomment if using OLED
// #include <Wire.h>
// #include <SSD1306Wire.h>
// SSD1306Wire display(0x3c, 21, 22); // SDA=21, SCL=22

#define I2S_WS   25  // L/R Clock
#define I2S_SD   22  // Data
#define I2S_SCK  26  // Bit Clock
#define I2S_PORT I2S_NUM_0
#define BUZZER_PIN 23
#define SAMPLE_BUFFER_SIZE 1024
#define SAMPLING_RATE 44100

#define FREQ_MIN_TRIGGER 8700 //10700
#define FREQ_MAX_TRIGGER 15200 //13200
#define RMS_MIN_TRIGGER  0.1000 //0.110
#define RMS_MAX_TRIGGER  0.2500 //0.189
#define DBFS_MIN_TRIGGER -21.5 //-17.5
#define DBFS_MAX_TRIGGER -11.0 //-15.0

#define SAMPLE_INTERVAL_MS 300   // roughly 3-4 readings per second
#define WINDOW_DURATION_MS 8000  // 8-second averaging window
#define TRIGGER_COUNT      3     // need 3 consecutive valid readings

// ------------------------------
// Runtime Variables
// ------------------------------
unsigned long lastSampleTime = 0;
unsigned long windowStartTime = 0;
bool collecting = false;
bool whistleDetected = false;
int triggerCount = 0;

// For averaging
float sumFreq = 0, sumRms = 0, sumDbfs = 0;
int windowSamples = 0;

double rms = 0;
double dbfs = 0;
double freq = 0;
// ==================== FILTER DEFINITIONS ====================

// INMP441 mic calibration filter
SOS_IIR_Filter INMP441 = {
  1.00197834654696,
  {
    {-1.986920458344451, 0.986963226946616, 1.995178510504166, -0.995184322194091}
  }
};

// DC Blocker
SOS_IIR_Filter DC_BLOCKER = {
  0.9823854506141371,
  {
    {1.0, -0.9823854506141371, 0.0, 0.0}
  }
};

// A-weighting filter
SOS_IIR_Filter A_WEIGHTING = {
  1.0,
  {
    {-2.0, 1.0, 1.9999695, -0.9999695},
    {-1.990047454833984, 0.9900722503662109, 1.999994039535522, -0.999994039535522},
    {-1.99917209148407, 0.9991759657859802, 1.999948859214783, -0.999948859214783}
  }
};
double vReal[SAMPLE_BUFFER_SIZE];
double vImag[SAMPLE_BUFFER_SIZE];
int32_t samples[SAMPLE_BUFFER_SIZE];

ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, SAMPLE_BUFFER_SIZE, SAMPLING_RATE);
// ============================================================

void setup() {

  pinMode(BUZZER_PIN, OUTPUT);
  Serial.begin(115200);
  delay(1000);
  Serial.println("INMP441 Sound Level Meter Starting...");

  // OLED init (optional)
  // display.init();
  // display.flipScreenVertically();
  // display.setFont(ArialMT_Plain_16);

  // I2S Configuration
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLING_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 8,
    .dma_buf_len = 1024,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };

  i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_SCK,
    .ws_io_num = I2S_WS,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = I2S_SD
  };

  i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_PORT, &pin_config);
  i2s_start(I2S_PORT);
}

void read_values()
{
  int32_t samples[SAMPLE_BUFFER_SIZE];
  size_t bytes_read;

  // Read from I2S
  i2s_read(I2S_PORT, (void*)samples, sizeof(samples), &bytes_read, portMAX_DELAY);
  int samples_read = bytes_read / sizeof(int32_t);
  if (samples_read == 0) return;

  // Convert and compute RMS
  double sum_squares = 0;
  for (int i = 0; i < samples_read; i++) {
    // Convert 24-bit left-justified to float (-1.0 to 1.0)
    float s = (float)(samples[i] >> 14) / 32768.0f;
    sum_squares += s * s;
  }

  rms = sqrt(sum_squares / samples_read);
  dbfs = 20.0 * log10(rms);

  // Serial.printf("RMS: %.4f, dBFS: %.2f\n", rms, dbfs);

  // OLED display (optional)
  // display.clear();
  // display.drawString(0, 0, "dBFS:");
  // display.drawString(0, 20, String(dbfs, 2));
  // display.display();
  for (int i = 0; i < samples_read; i++) {
    vReal[i] = (double)samples[i] / 2147483648.0;  // normalize to [-1,1]
    vImag[i] = 0.0;
  }

  FFT.windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.compute(FFT_FORWARD);
  FFT.complexToMagnitude();

  freq = FFT.majorPeak();
}
void loop() {
  unsigned long now = millis();

  // sample every ~300ms
  if (now - lastSampleTime >= SAMPLE_INTERVAL_MS) {
    lastSampleTime = now;

    read_values();
    Serial.printf("RMS: %.4f, dBFS: %.2f, freq :%.2f\n", rms, dbfs, freq);

    bool freq_in_range = (freq >= FREQ_MIN_TRIGGER && freq <= FREQ_MAX_TRIGGER);
    bool rms_in_range  = (rms  >= RMS_MIN_TRIGGER  && rms  <= RMS_MAX_TRIGGER);
    bool dbfs_in_range = (dbfs >= DBFS_MIN_TRIGGER && dbfs <= DBFS_MAX_TRIGGER);

    if (!collecting) {
      // Check for 3 consecutive valid readings
      if (freq_in_range && rms_in_range && dbfs_in_range) {
        triggerCount++;
        if (triggerCount >= TRIGGER_COUNT) {
          collecting = true;
          windowStartTime = now;
          sumFreq = sumRms = sumDbfs = 0;
          windowSamples = 0;
          Serial.println("✅ Trigger detected → starting 8s window");
        }
      } else {
        triggerCount = 0;  // reset if one reading fails
      }
    } 
    else {
      // Collect for 8 seconds
      sumFreq += freq;
      sumRms  += rms;
      sumDbfs += dbfs;
      windowSamples++;

      if (now - windowStartTime >= WINDOW_DURATION_MS) {
        float avgFreq = sumFreq / windowSamples;
        float avgRms  = sumRms  / windowSamples;
        float avgDbfs = sumDbfs / windowSamples;

        if (avgFreq >= FREQ_MIN_TRIGGER && avgFreq <= FREQ_MAX_TRIGGER &&
            avgRms  >= RMS_MIN_TRIGGER  && avgRms  <= RMS_MAX_TRIGGER  &&
            avgDbfs >= DBFS_MIN_TRIGGER && avgDbfs <= DBFS_MAX_TRIGGER) {
          whistleDetected = true;
          Serial.println("🎯 Whistle detected!");
          digitalWrite(BUZZER_PIN, HIGH);
          delay(5000);  // ring for 5 seconds

          // Turn buzzer OFF
          digitalWrite(BUZZER_PIN, LOW);

        } else {
          whistleDetected = false;
          Serial.println("❌ No whistle detected (avg out of range)");
        }

        // reset
        collecting = false;
        triggerCount = 0;
      }
    }
  }
  // delay(250);
}
