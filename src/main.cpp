#include <Arduino.h>
#include <arduinoFFT.h>
#include <SPI.h>
#include <math.h> 

//LED logic
#include <Adafruit_NeoPixel.h>

#define NEOPIXEL_PIN 17
#define NUM_PIXELS   10

Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_PIXELS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);


// ====== Constants and Configs ======
#pragma region Constants and Configs
// LIS3DH Registers
#define LIS3DH_REG_CTRL1    0x20
#define LIS3DH_REG_CTRL4    0x23
#define LIS3DH_REG_OUT_X_L  0x28

// LIS3DH Sensitivity & Filters
#define SENSITIVITY_4G (0.08f)  // g/LSB for ±4g mode
#define ALPHA_HPF 0.85f  // HPF coefficient: lower = less noise, higher = losing more data
#define ALPHA_MAF 0.2f  // MAF smoothing factor: lower = smoother, higher = faster response

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

#pragma endregion Constants and Configs

// ====== Global Variables ======
#pragma region Global Variables
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

// Sample storage
volatile uint16_t sampleIndex = 0;
float xg_samples[MAX_SAMPLES];
float yg_samples[MAX_SAMPLES];
float zg_samples[MAX_SAMPLES];

// FFT
struct PeakInfo {   // Struct to hold frequency and magnitude information, used in FFT to export results
  float frequency;
  float magnitude;
};
//float dominantFreqMag[6]; // 0-2 frequencies (first three), 3-5 magnitudes (last three)
float generalizedFreq = 0.0;
float generalizedMag = 0.0;


// Timer
volatile unsigned long lastTimer1Micros = 0;
volatile unsigned long startSamplingTime = 0;
volatile uint16_t lastSampleCount = 0; // Last sample count for Timer3

// Logging
char logBuffer[MAX_LOG_MESSAGES][MAX_MESSAGE_LENGTH];
volatile uint8_t logHead = 0;
volatile uint8_t logTail = 0;

#pragma endregion Global Variables

// ====== Function Prototypes ======
#pragma region Function Prototypes
void realTimeTesting();
void setUpTimer1();
void setupTimer3();
void stopTimer1();
void startTimer1();
void stopTimer3();
void startTimer3();
void SensorSetup();
void lis3dh_write(uint8_t reg, uint8_t value);
void lis3dh_read_multiple(uint8_t reg, uint8_t* buffer, uint8_t len);
void updateSensorData();
void analyzeAllAxes();
PeakInfo computeDominantFrequency(float* inputArray);
void queueLog(const char* msg);
void loopPrintLogs();
void indicateConditionWithLED(float frequency, float amplitude);

#pragma endregion Function Prototypes

// ====== Arduino Framework ======
#pragma region Arduino Framework
void setup() {
  //Initialize led 
  strip.begin();
  strip.setBrightness(100);
  strip.show();

  // Initialize Serial and SPI
  Serial.begin(115200);
  SPI.begin();
  SPI.beginTransaction(SPISettings(5000000, MSBFIRST, SPI_MODE0));
  
  SensorSetup(); // Setup LIS3DH sensor

  cli(); // Disable interrupts globally

  setUpTimer1();  // Setup Timer1 for 20ms sampling, but not started yet
  setupTimer3();  // Setup Timer3 for 3 seconds sampling, but not started yet
  startTimer3(); // Start Timer3 for the first sampling

  sei(); // Enable interrupts globally

  //delay(1000); // Wait for a second before starting the sampling
  //Serial.println(F(">Starting 3-second sampling..."));
  
}

void loop() {
  //realTimeTesting(); // Check for serial commands


  loopPrintLogs(); // Print logs from logBuffer if logging is enabled
}

#pragma endregion Arduino Framework

// ====== Timer & ISR ======
#pragma region Timer & ISR
// This ISR is triggered when Timer1 reaches the set interval (20ms)
// It reads the sensor data and applies filtering, and logs the interval time if logging is enabled
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

// This ISR is triggered when Timer3 reaches the set interval (3 seconds)
// It stops Timer1 and Timer3, and prepares for the next sampling
// It also logs the elapsed time since the start of sampling
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
  Serial.println(F(">Sampling done!"));
  lastSampleCount = sampleIndex;
  sampleIndex = 0;
  analyzeAllAxes(); // Analyze the samples after sampling is done
  indicateConditionWithLED(generalizedFreq, generalizedMag); // Indicate condition with LED
  startTimer3(); // Restart Timer3 for the next sampling
}

#pragma endregion Timer & ISR

// ====== Functions ======
#pragma region Functions

#pragma region Timer Functions
// Timer1 and Timer3 
void setUpTimer1() {
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  OCR1A = 624;             // ~20ms interval 624
  TCCR1B |= (1 << WGM12);  // CTC mode
  TIMSK1 |= (1 << OCIE1A); // Enable Timer1 Compare Match A interrupt
}

void setupTimer3() {
  TCCR3A = 0;
  TCCR3B = 0;
  TCNT3 = 0;
  OCR3A = 23800;           // Calibrated for ~3 seconds (23800)
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
  Serial.println(F(">Starting Timer3..."));
  TCNT3 = 0;
  TCNT1 = 0;
  startSamplingTime = micros();
  TCCR1B |= (1 << CS12); // Start Timer1
  TCCR3B |= (1 << CS32) | (1 << CS30); // Start Timer3
}

void stopTimer3() {
  TCCR3B &= ~((1 << CS32) | (1 << CS31) | (1 << CS30));
}

#pragma endregion Timer Functions

#pragma region LIS3DH Functions

// LIS3DH SPI communication functions
void SensorSetup()
{
  DDRB |= (1 << PB4);  // Set D8 (PB4) as output
  PORTB |= (1 << PB4); // Set D8 high (inactive)

  delay(100);

  // LIS3DH Configuration
  lis3dh_write(LIS3DH_REG_CTRL1, 0x47); // 50Hz ODR
  lis3dh_write(LIS3DH_REG_CTRL4, 0x10); // ±4g sensitivity

  delay(100);
}

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
// This function reads the sensor data and applies a high-pass filter and moving average filter
// It also queues the data for logging if logging is enabled
void updateSensorData() {
  uint8_t rawData[6];
  int16_t x_raw, y_raw, z_raw;

  lis3dh_read_multiple(LIS3DH_REG_OUT_X_L, rawData, 6);

  x_raw = (int16_t)(rawData[1] << 8 | rawData[0]) >> 6;
  y_raw = (int16_t)(rawData[3] << 8 | rawData[2]) >> 6;
  z_raw = (int16_t)(rawData[5] << 8 | rawData[4]) >> 6;

  float xg_new = x_raw * SENSITIVITY_4G;
  float yg_new = y_raw * SENSITIVITY_4G;
  float zg_new = z_raw * SENSITIVITY_4G;

  // === High-Pass Filter (simple 1-pole)
  x_g = ALPHA_HPF * (x_g_prev + xg_new - x_g_prev);
  y_g = ALPHA_HPF * (y_g_prev + yg_new - y_g_prev);
  z_g = ALPHA_HPF * (z_g_prev + zg_new - z_g_prev);

  x_g_prev = xg_new;
  y_g_prev = yg_new;
  z_g_prev = zg_new;

  // === Moving Average Filter (Exponential Moving Average)
  x_g_avg = (1.0f - ALPHA_MAF) * x_g_avg + ALPHA_MAF * x_g;
  y_g_avg = (1.0f - ALPHA_MAF) * y_g_avg + ALPHA_MAF * y_g;
  z_g_avg = (1.0f - ALPHA_MAF) * z_g_avg + ALPHA_MAF * z_g;

  if (sampleIndex < MAX_SAMPLES) {
    xg_samples[sampleIndex] = x_g_avg;
    yg_samples[sampleIndex] = y_g_avg;
    zg_samples[sampleIndex] = z_g_avg;
    sampleIndex++;
  }

  if (LOGGING_ENABLED) {
    char msg[MAX_MESSAGE_LENGTH];
    char xgStr[10], ygStr[10], zgStr[10];

    dtostrf(x_g_avg, 0, 2, xgStr);
    dtostrf(y_g_avg, 0, 2, ygStr);
    dtostrf(z_g_avg, 0, 2, zgStr);

    snprintf(msg, MAX_MESSAGE_LENGTH,
      ">xg:%s,yg:%s,zg:%s\r\n",
      xgStr, ygStr, zgStr);

    queueLog(msg);
  }
}

#pragma endregion LIS3DH Functions

#pragma region FFT Functions

// FFT Processing
// This function computes the FFT of the input array and returns the dominant frequency and magnitude, packaged in a PeakInfo struct
PeakInfo computeDominantFrequency(float* inputArray) {
  float vImag[MAX_SAMPLES] = {0.0};
  ArduinoFFT<float> FFT(inputArray, vImag, MAX_SAMPLES, SAMPLING_FREQUENCY);

  FFT.dcRemoval();
  FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
  FFT.compute(FFTDirection::Forward);
  FFT.complexToMagnitude();

  PeakInfo result;
  FFT.majorPeak(&result.frequency, &result.magnitude);

  return result;
}

// Analyze all axes and compute dominant frequencies and magnitudes
void analyzeAllAxes() {
  PeakInfo xg = computeDominantFrequency(xg_samples);
  PeakInfo yg = computeDominantFrequency(yg_samples);
  PeakInfo zg = computeDominantFrequency(zg_samples);

  //dominantFreqMag[0] = xg.frequency;
  //dominantFreqMag[1] = yg.frequency;
  //dominantFreqMag[2] = zg.frequency;
  //dominantFreqMag[3] = xg.magnitude;
  //dominantFreqMag[4] = yg.magnitude;
  //dominantFreqMag[5] = zg.magnitude;

  if(1)
  {
    //Log it using dtostrf since the values are float
    //char msg[MAX_MESSAGE_LENGTH];
    //char xgStr[10], ygStr[10], zgStr[10], xgMagStr[10], ygMagStr[10], zgMagStr[10];
//
    //dtostrf(xg.frequency, 0, 2, xgStr);
    //dtostrf(yg.frequency, 0, 2, ygStr);
    //dtostrf(zg.frequency, 0, 2, zgStr);
    //dtostrf(xg.magnitude, 0, 2, xgMagStr);
    //dtostrf(yg.magnitude, 0, 2, ygMagStr);
    //dtostrf(zg.magnitude, 0, 2, zgMagStr);
//
    //snprintf(msg, MAX_MESSAGE_LENGTH,
    //  ">xg:%s Hz,m:%s;yg:%s Hz,m:%s;zg:%s Hz,m:%s\r\n",
    //  xgStr,xgMagStr,
    //  ygStr,ygMagStr,
    //  zgStr,zgMagStr);
    //
    //queueLog(msg);

    //use serial print to print all frequencies and magnitudes
    //Serial.print(F("xg: "));
    //Serial.print(xg.frequency, 2);
    //Serial.print(F(" Hz, Magnitude: "));
    //Serial.println(xg.magnitude, 2);
    //Serial.print(F("yg: "));
    //Serial.print(yg.frequency, 2);
    //Serial.print(F(" Hz, Magnitude: "));
    //Serial.println(yg.magnitude, 2);
    //Serial.print(F("zg: "));
    //Serial.print(zg.frequency, 2);
    //Serial.print(F(" Hz, Magnitude: "));
    //Serial.println(zg.magnitude, 2);

  }

  generalizedFreq = sqrt(xg.frequency * xg.frequency + yg.frequency * yg.frequency + zg.frequency * zg.frequency); // Use sqrt of sum of squares for generalized frequency
  generalizedMag = max(xg.magnitude, max(yg.magnitude, zg.magnitude)); // Use max magnitude for generalized magnitude
  Serial.println(F(">Analysis Done"));
  Serial.print("Frequency: ");
  Serial.println(generalizedFreq);
  Serial.print("Amplitude: ");
  Serial.println(generalizedMag);
}

#pragma endregion FFT Functions

#pragma region Real-time Testing, Debugging, and Logging

// Real-time testing and debug function
// This function is called in the main loop to check for serial commands
void realTimeTesting()
{
  if (Serial.available())
  {
    char command = Serial.read();

    if (command == 's')
    {
      startTimer3();
      Serial.println(F(">Started 3-second sampling..."));
    }
    //else if (command == 'f')
    //{
    //  analyzeAllAxes();
//
    //  Serial.println(F(">Dominant Frequencies and Magnitudes:"));
//
    //  Serial.print(F("xg: "));
    //  Serial.print(dominantFreqMag[0], 2);
    //  Serial.print(F(" Hz, Magnitude: "));
    //  Serial.println(dominantFreqMag[3], 2);
//
    //  Serial.print(F("yg: "));
    //  Serial.print(dominantFreqMag[1], 2);
    //  Serial.print(F(" Hz, Magnitude: "));
    //  Serial.println(dominantFreqMag[4], 2);
//
    //  Serial.print(F("zg: "));
    //  Serial.print(dominantFreqMag[2], 2);
    //  Serial.print(F(" Hz, Magnitude: "));
    //  Serial.println(dominantFreqMag[5], 2);
    //}
  }
}

// Logging function
// This function queues a log message into the log buffer to prevent blocking
// It checks if logging is enabled and if the buffer is not full
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

void loopPrintLogs()
{
  if (LOGGING_ENABLED)
  {
    while (logTail != logHead)
    {
      Serial.print(logBuffer[logTail]);
      logTail = (logTail + 1) % MAX_LOG_MESSAGES;
    }
  }
}

#pragma endregion Real-time Testing, Debugging, and Logging

#pragma region USER INTERFACE

// Function to indicate detected movement via LED
void indicateConditionWithLED(float frequency, float amplitude) {
  Serial.println(F(">Displaying LEDs"));

  if (amplitude < 5.0) {
    strip.clear();
    strip.show();
    return;
  }

  if (frequency <= 3.0) {
    for (int i = 0; i < NUM_PIXELS; i++) {
      strip.setPixelColor(i, strip.Color(0, 0, 255)); // Blue
    }
    strip.show();
  }

  // Tremor: 3.0 - 5.0 Hz
  else if (frequency > 3.0 && frequency <= 5.0) {
    int pixelsToLight;
    pixelsToLight = map(frequency * 100, 300, 500, 1, NUM_PIXELS);
    pixelsToLight = constrain(pixelsToLight, 1, NUM_PIXELS);

    for (int i = 0; i < NUM_PIXELS; i++) {
      if (i < pixelsToLight)
        strip.setPixelColor(i, strip.Color(0, 255, 0)); // Green
      else
        strip.setPixelColor(i, strip.Color(0, 0, 0));   // Off
    }
    strip.show();
  }

  // Dyskinesia: 5.0 - 7.0 Hz
  else if (frequency > 5.0 && frequency <= 7.0) {
    int pixelsToLight;
    pixelsToLight = map(frequency * 100, 500, 700, 1, NUM_PIXELS);
    pixelsToLight = constrain(pixelsToLight, 1, NUM_PIXELS);

    for (int i = 0; i < NUM_PIXELS; i++) {
      if (i < pixelsToLight)
        strip.setPixelColor(i, strip.Color(255, 0, 0)); // Red
      else
        strip.setPixelColor(i, strip.Color(0, 0, 0));   // Off
    }
    strip.show();
  }

  else {
    strip.clear();
    strip.show();
  }
}


#pragma endregion USER INTERFACE


#pragma endregion Functions