//#define USE_AVR_PROGMEM
//#define FFT_SQRT_APPROXIMATION

#include <Arduino.h>
#include <arduinoFFT.h>
#include <SPI.h>
#include <math.h> 


// ====== Constants and Configs ======
// LIS3DH Registers
#define LIS3DH_REG_CTRL1    0x20
#define LIS3DH_REG_CTRL4    0x23
#define LIS3DH_REG_OUT_X_L  0x28

// LIS3DH Sensitivity & Filters
#define SENSITIVITY_4G (0.08f)  // g/LSB for ±4g mode
#define ALPHA_HPF 0.9f  // HPF coefficient (adjust as needed)
#define ALPHA_MAF 0.3f  // MAF smoothing factor (adjust as needed)

// SPI Chip Select pin (D8 -> PB4)
#define CS_LOW()  (PORTB &= ~(1 << PB4))
#define CS_HIGH() (PORTB |= (1 << PB4))

// LOGGING 
#define MAX_LOG_MESSAGES 5  // Log buffer size
#define MAX_MESSAGE_LENGTH 50  // Max characters per message
#define LOGGING_ENABLED 0        //Change to false (0) to disable logging

// Sampling & FFT
#define MAX_SAMPLES 128
#define SAMPLING_FREQUENCY 50 //Hz


// ====== Global Variables ======
// Sensor data
float x_g = 0.0;
float y_g = 0.0;
float z_g = 0.0;
float x_g_prev = 0.0;
float y_g_prev = 0.0;
float z_g_prev = 0.0;
float x_g_avg = 0.0;
float y_g_avg = 0.0;
float z_g_avg = 0.0;
float rotX_deg = 0.0;
float rotY_deg = 0.0;

// Sample storage
volatile uint16_t sampleIndex = 0;

int16_t xg_samples[MAX_SAMPLES];
int16_t yg_samples[MAX_SAMPLES];
int16_t zg_samples[MAX_SAMPLES];
int16_t rotX_samples[MAX_SAMPLES];
int16_t rotY_samples[MAX_SAMPLES];

// FFT
float dominantFrequencies[5];  // Result container


// Timer and Logging
volatile unsigned long lastTimer1Micros = 0;
volatile unsigned long startSamplingTime = 0;
volatile uint16_t lastSampleCount = 0; // Last sample count for Timer3


char logBuffer[MAX_LOG_MESSAGES][MAX_MESSAGE_LENGTH];
volatile uint8_t logHead = 0;
volatile uint8_t logTail = 0;

// ====== Function Prototypes ======
void setUpTimer1();
void setupTimer3();
void stopTimer1();
void startTimer1();
void stopTimer3();
void startTimer3();
void lis3dh_write(uint8_t reg, uint8_t value);
void lis3dh_read_multiple(uint8_t reg, uint8_t* buffer, uint8_t len);
void updateSensorData();
void analyzeAllAxes();
float computeDominantFrequency(int16_t* inputArray);
void queueLog(const char* msg);

// ====== Setup ======
void setup() {
  Serial.begin(115200);

  SPI.begin();
  SPI.beginTransaction(SPISettings(5000000, MSBFIRST, SPI_MODE0));

  DDRB |= (1 << PB4);    // Set D8 (PB4) as output
  PORTB |= (1 << PB4);   // Set D8 high (inactive)

  delay(100);

  // LIS3DH Configuration
  lis3dh_write(LIS3DH_REG_CTRL1, 0x47); // 50Hz ODR
  lis3dh_write(LIS3DH_REG_CTRL4, 0x10); // ±4g sensitivity

  delay(100);

  cli(); // Disable interrupts globally

  setUpTimer1();
  setupTimer3();

  sei(); // Enable interrupts globally
}

// ====== Main Loop ======
void loop() {
  if (Serial.available()) {
    char command = Serial.read();

    if (command == 's') {
      startTimer3();
      Serial.println(F(">Started 3-second sampling..."));
    }
    else if (command == 'l') {
      // Only print size when sampling is done (sampleIndex == 0 means finished)
      if (sampleIndex == 0) {
        Serial.print(F(">Samples collected: "));
        Serial.println(lastSampleCount);
      } else {
        Serial.println(F(">Sampling not complete yet!"));
      }
    }
    else if (command == 'f') {
      // Run FFT analysis on all axes
      analyzeAllAxes();

      Serial.print(F(">Dominant Frequencies (Hz):\n"));
      Serial.print("xg: "); Serial.println(dominantFrequencies[0], 2);
      Serial.print("yg: "); Serial.println(dominantFrequencies[1], 2);
      Serial.print("zg: "); Serial.println(dominantFrequencies[2], 2);
      Serial.print("rotX: "); Serial.println(dominantFrequencies[3], 2);
      Serial.print("rotY: "); Serial.println(dominantFrequencies[4], 2);
    }
  }

  if (LOGGING_ENABLED) {
    while (logTail != logHead) {
      Serial.print(logBuffer[logTail]);
      logTail = (logTail + 1) % MAX_LOG_MESSAGES;
    }
  }
}


// ====== Timer1 Interrupt Service Routine ======
ISR(TIMER1_COMPA_vect) {
  if (LOGGING_ENABLED) {
    unsigned long now = micros();
    unsigned long interval = now - lastTimer1Micros;
    lastTimer1Micros = now;

    updateSensorData(); // inside, it will queue sensor data too

    char msg[MAX_MESSAGE_LENGTH];
    snprintf(msg, MAX_MESSAGE_LENGTH, ">interval1_us:%lu\r\n", interval);
    queueLog(msg);
  } 
  else {
    updateSensorData(); // still update sensor even if logging disabled
  }
}

// ====== Timer3 Interrupt Service Routine ======
ISR(TIMER3_COMPA_vect) {
  stopTimer1();
  stopTimer3();

  if (LOGGING_ENABLED) {
    unsigned long elapsedMicros = micros() - startSamplingTime;
    
    char msg[MAX_MESSAGE_LENGTH];
    snprintf(msg, MAX_MESSAGE_LENGTH, ">interval3_us:%lu\r\n", elapsedMicros);
    queueLog(msg);
  }

  // Prepare for next sampling
  lastSampleCount = sampleIndex;
  sampleIndex = 0;
}

// ====== Functions ======
// Timer1 and Timer3 
void setUpTimer1() {
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  OCR1A = 624;             // ~20ms interval
  TCCR1B |= (1 << WGM12);  // CTC mode
  TIMSK1 |= (1 << OCIE1A); // Enable Timer1 Compare Match A interrupt
}

void setupTimer3() {
  TCCR3A = 0;
  TCCR3B = 0;
  TCNT3 = 0;
  OCR3A = 23800;           // Calibrated for ~3 seconds
  TCCR3B |= (1 << WGM32);  // CTC mode
  TIMSK3 |= (1 << OCIE3A); // Enable Timer3 Compare Match A interrupt
}

void stopTimer1() {
  TCCR1B &= ~((1 << CS12) | (1 << CS11) | (1 << CS10));
}

void startTimer1() {
  TCNT1 = 0;
  TCCR1B |= (1 << CS12); // Prescaler 256
}

void startTimer3() {
  TCNT3 = 0;
  TCNT1 = 0;
  startSamplingTime = micros();
  TCCR1B |= (1 << CS12); // Start Timer1
  TCCR3B |= (1 << CS32) | (1 << CS30); // Start Timer3
}

void stopTimer3() {
  TCCR3B &= ~((1 << CS32) | (1 << CS31) | (1 << CS30));
}


// LIS3DH SPI communication functions
void lis3dh_write(uint8_t reg, uint8_t value) {
  CS_LOW();
  SPI.transfer(0x00 | (reg & 0x3F));
  SPI.transfer(value);
  CS_HIGH();
}

void lis3dh_read_multiple(uint8_t reg, uint8_t* buffer, uint8_t len) {
  CS_LOW();
  SPI.transfer(0xC0 | (reg & 0x3F));
  for (uint8_t i = 0; i < len; i++) {
    buffer[i] = SPI.transfer(0x00);
  }
  CS_HIGH();
}

// Sensor Sampling
void updateSensorData() {
  uint8_t rawData[6];
  int16_t x_raw, y_raw, z_raw;

  lis3dh_read_multiple(LIS3DH_REG_OUT_X_L, rawData, 6);

  x_raw = (int16_t)(rawData[1] << 8 | rawData[0]) >> 6;
  y_raw = (int16_t)(rawData[3] << 8 | rawData[2]) >> 6;
  z_raw = (int16_t)(rawData[5] << 8 | rawData[4]) >> 6;

  x_g = x_raw * SENSITIVITY_4G;
  y_g = y_raw * SENSITIVITY_4G;
  z_g = z_raw * SENSITIVITY_4G;

  float rotX_rad = atan2(y_g, z_g);
  float rotY_rad = atan2(-x_g, z_g);

  rotX_deg = rotX_rad * (180.0 / PI);
  rotY_deg = rotY_rad * (180.0 / PI);

  if (sampleIndex < MAX_SAMPLES) {
    xg_samples[sampleIndex] = (int16_t)(x_g * 100);
    yg_samples[sampleIndex] = (int16_t)(y_g * 100);
    zg_samples[sampleIndex] = (int16_t)(z_g * 100);
    rotX_samples[sampleIndex] = (int16_t)(rotX_deg * 100);
    rotY_samples[sampleIndex] = (int16_t)(rotY_deg * 100);
    sampleIndex++;
  }

  if (LOGGING_ENABLED) {
    char msg[MAX_MESSAGE_LENGTH];
    snprintf(msg, MAX_MESSAGE_LENGTH,
      ">xg:%d,yg:%d,zg:%d,rx:%d,ry:%d\r\n",
      (int)(x_g * 100), (int)(y_g * 100), (int)(z_g * 100),
      (int)(rotX_deg * 100), (int)(rotY_deg * 100));

    queueLog(msg);

    //Serial.print('>');
    //Serial.print("x_g:"); Serial.print(x_g, 3); Serial.print(',');
    //Serial.print("y_g:"); Serial.print(y_g, 3); Serial.print(',');
    //Serial.print("z_g:"); Serial.print(z_g, 3); Serial.print(',');
    //Serial.print("rotX_deg:"); Serial.print(rotX_deg, 2); Serial.print(',');
    //Serial.print("rotY_deg:"); Serial.print(rotY_deg, 2);
    //Serial.print('\r');
    //Serial.print('\n');
  }
}

// FFT Processing
float computeDominantFrequency(int16_t* inputArray) {

  float inputArrayFloat[MAX_SAMPLES];
  for (uint16_t i = 0; i < MAX_SAMPLES; i++) {
    inputArrayFloat[i] = (inputArray[i])/100.0f; // Convert to float and scale
  }
  float vImag[MAX_SAMPLES] = {0.0};


  ArduinoFFT<float> FFT(inputArrayFloat, vImag, MAX_SAMPLES, SAMPLING_FREQUENCY); // Create local FFT object

  FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward, true); //Adding windowing & compensation
  FFT.compute(FFTDirection::Forward);
  FFT.complexToMagnitude();

  float peakFrequency = FFT.majorPeak();
  return peakFrequency;
}


void analyzeAllAxes() {
  dominantFrequencies[0] = computeDominantFrequency(xg_samples);
  dominantFrequencies[1] = computeDominantFrequency(yg_samples);
  dominantFrequencies[2] = computeDominantFrequency(zg_samples);
  dominantFrequencies[3] = computeDominantFrequency(rotX_samples);
  dominantFrequencies[4] = computeDominantFrequency(rotY_samples);
}

// Logging function
void queueLog(const char* msg) {
  if (!LOGGING_ENABLED) return;

  uint8_t nextHead = (logHead + 1) % MAX_LOG_MESSAGES;
  if (nextHead == logTail) {
    // Buffer full, drop new message
    return;
  }

  strncpy(logBuffer[logHead], msg, MAX_MESSAGE_LENGTH);
  logHead = nextHead;
}