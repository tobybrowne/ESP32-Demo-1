#include "driver/i2c.h"
#include "Arduino.h"
#define DEVICE_ADDR 0x69


#define I2C_MASTER_SCL_IO    35     // SCL Pin
#define I2C_MASTER_SDA_IO    33     // SDA Pin
#define I2C_MASTER_NUM       I2C_NUM_0 // I2C port number
#define I2C_MASTER_FREQ_HZ   400000  // I2C clock speed (400kHz)
#define I2C_MASTER_TX_BUF_DISABLE 0 // Disable buffer
#define I2C_MASTER_RX_BUF_DISABLE 0 // Disable buffer

// Function to read from an I2C register
void i2c_write_byte_to_register(uint8_t device_addr, uint8_t reg_addr, uint8_t data)
{
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();

  // Step 1: Write the register address
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (device_addr << 1) | I2C_MASTER_WRITE, true);  // Send device address with write flag
  i2c_master_write_byte(cmd, reg_addr, true);  // Send the register address
  i2c_master_write_byte(cmd, data, true);
  i2c_master_stop(cmd);

  // Execute both write and read commands
  esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
  if (ret != ESP_OK)
  {
    Serial.println(esp_err_to_name(ret));
  }

  i2c_cmd_link_delete(cmd);
}

// Function to read from an I2C register
void i2c_master_read_register(uint8_t device_addr, uint8_t reg_addr, uint8_t *data, size_t size)
{
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();

  // Step 1: Write the register address
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (device_addr << 1) | I2C_MASTER_WRITE, true);  // Send device address with write flag
  i2c_master_write_byte(cmd, reg_addr, true);  // Send the register address
  i2c_master_stop(cmd);

  // Step 2: Send the read command
  i2c_cmd_handle_t cmd_read = i2c_cmd_link_create();
  i2c_master_start(cmd_read);
  i2c_master_write_byte(cmd_read, (device_addr << 1) | I2C_MASTER_READ, true);  // Send device address with read flag
  i2c_master_read(cmd_read, data, size, I2C_MASTER_ACK);  // Read the data
  i2c_master_stop(cmd_read);

  // Execute both write and read commands
  esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
  if (ret != ESP_OK)
  {
    // printf("I2C error: %s\n", esp_err_to_name(ret));
    Serial.println(esp_err_to_name(ret));
  }

  ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd_read, pdMS_TO_TICKS(1000));
  if (ret != ESP_OK)
  {
    // printf("error reading from register!");
    Serial.println(esp_err_to_name(ret));
  }

  i2c_cmd_link_delete(cmd);
  i2c_cmd_link_delete(cmd_read);
}

void mpu6050_init()
{
  uint8_t reset_cmd = 0x00;

  // Wake up the MPU-6050
  i2c_write_byte_to_register(DEVICE_ADDR, 0x6B, reset_cmd);

  // Set Gyroscope config: ±250 °/s (default)
  uint8_t gyro_config = 0x00;  
  i2c_write_byte_to_register(DEVICE_ADDR, 0x1B, gyro_config);

  // Set Accelerometer config: ±2g (default)
  uint8_t accel_config = 0x00; 
  i2c_write_byte_to_register(DEVICE_ADDR, 0x1C, accel_config);

  Serial.println("MPU-6050 initialized!");
}