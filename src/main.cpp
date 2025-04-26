#include <Arduino.h>
#include <SPI.h>
#include <math.h> 

// ========== Constants and Config ==========
#define LIS3DH_REG_CTRL1    0x20
#define LIS3DH_REG_CTRL4    0x23
#define LIS3DH_REG_OUT_X_L  0x28

#define SENSITIVITY_4G (0.08f)  // g per LSB for ±4g setting

// SPI Chip Select pin (D8 -> PB4)
#define CS_LOW()  (PORTB &= ~(1 << PB4))
#define CS_HIGH() (PORTB |= (1 << PB4))

// ========== Global Variables ==========
float x_g = 0.0;
float y_g = 0.0;
float z_g = 0.0;
float rotX_deg = 0.0;
float rotY_deg = 0.0;

// ========== Functions ==========

// Write one byte to LIS3DH register
void lis3dh_write(uint8_t reg, uint8_t value) {
  CS_LOW();
  SPI.transfer(0x00 | (reg & 0x3F));  // Write command
  SPI.transfer(value);
  CS_HIGH();
}

// Read multiple bytes from LIS3DH
void lis3dh_read_multiple(uint8_t reg, uint8_t* buffer, uint8_t len) {
  CS_LOW();
  SPI.transfer(0xC0 | (reg & 0x3F));  // Read command with auto-increment
  for (uint8_t i = 0; i < len; i++) {
    buffer[i] = SPI.transfer(0x00);
  }
  CS_HIGH();
}

// Update sensor readings and calculate rotation angles
void updateSensorData() {
  uint8_t rawData[6];
  int16_t x_raw, y_raw, z_raw;

  // Read raw accelerometer data
  lis3dh_read_multiple(LIS3DH_REG_OUT_X_L, rawData, 6);

  // Process raw data
  x_raw = (int16_t)(rawData[1] << 8 | rawData[0]) >> 6;
  y_raw = (int16_t)(rawData[3] << 8 | rawData[2]) >> 6;
  z_raw = (int16_t)(rawData[5] << 8 | rawData[4]) >> 6;

  // Convert to g-units
  x_g = x_raw * SENSITIVITY_4G;
  y_g = y_raw * SENSITIVITY_4G;
  z_g = z_raw * SENSITIVITY_4G;

  // Compute rotations (CW positive)   //TODO: Angle Coupling Bug here
  float rotX_rad = atan2(y_g, z_g);   // side-to-side tilt
  float rotY_rad = atan2(-x_g, z_g);  // forward/backward tilt

  rotX_deg = rotX_rad * (180.0 / PI);
  rotY_deg = rotY_rad * (180.0 / PI);

  // Serial Plotter Output
  Serial.print('>');
  Serial.print("x_g:"); Serial.print(x_g, 3); Serial.print(',');
  Serial.print("y_g:"); Serial.print(y_g, 3); Serial.print(',');
  Serial.print("z_g:"); Serial.print(z_g, 3); Serial.print(',');
  Serial.print("rotX_deg:"); Serial.print(rotX_deg, 2); Serial.print(',');
  Serial.print("rotY_deg:"); Serial.print(rotY_deg, 2);
  Serial.print('\r');
  Serial.print('\n');
}

// ========== Setup Function ==========
void setup() {
    Serial.begin(115200);
  
    SPI.begin();
    SPI.beginTransaction(SPISettings(5000000, MSBFIRST, SPI_MODE0));
  
    DDRB |= (1 << PB4);    // Set PB4 (D8) as output
    PORTB |= (1 << PB4);   // Set PB4 high (inactive)
  
    delay(100); // wait for LIS3DH to be ready
  
    // Configure LIS3DH
    lis3dh_write(LIS3DH_REG_CTRL1, 0x37); // 25 Hz ODR, XYZ axes enabled
    lis3dh_write(LIS3DH_REG_CTRL4, 0x10); // ±4g range
  
    delay(100);

  }
  
  // ========== Main Loop ==========
  void loop() {
    updateSensorData();
    delay(40); // 25 Hz matching delay
  }
  