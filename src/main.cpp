#include <Wire.h>
#include "Arduino.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;

void init_mpu()
{
  Serial.println("Adafruit MPU6050 test!");

  // Set the I2C pins (ESP32-S2: SDA = GPIO8, SCL = GPIO9)
  Wire.begin(8, 9);
  delay(100);

  if (!mpu.begin(0x69)) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) { delay(10); }
  }

  Serial.println("MPU6050 Found!");

  // Configure the MPU6050 settings
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  Serial.println("MPU6050 Initialized!");
  delay(100);
}

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  init_mpu();
}

void resetI2C() {
  Serial.println("I2C Resetting...");
  Wire.end();
  delay(100);
  Wire.begin(8, 9); // Reinitialize I2C with custom SDA, SCL pins
}

int zero_count = 0;
float current_yaw = 0;
float a_rate;
int samp_period = 5;

void poll_mpu()
{
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  
  // Extract accelerometer values for pitch and roll calculation
  float accelX = a.acceleration.x;
  float accelY = a.acceleration.y;
  float accelZ = a.acceleration.z;

  // Calculate pitch and roll angles (in radians)
  float pitch = atan2(accelX, sqrt(accelY * accelY + accelZ * accelZ));
  float roll = atan2(accelY, sqrt(accelX * accelX + accelZ * accelZ));

  // Convert radians to degrees
  pitch = pitch * 180.0 / PI;
  roll = roll * 180.0 / PI;

  // Print the pitch and roll angles
  Serial.println(pitch);

  // a_rate = g.gyro.x * 57.2958;
  // current_yaw = current_yaw + (a_rate * ((float)samp_period/1000));
  // Serial.println(current_yaw);
}

void loop() {
  

  delay(samp_period);  // Wait for 500ms before reading again
}
