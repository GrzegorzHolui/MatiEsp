#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <esp_log.h>
#include <stdio.h>

#define MMA8452Q_ADDR 0x1C
#define MMA8452Q_SDA GPIO_NUM_21
#define MMA8452Q_SCL GPIO_NUM_22
#define DELAY_MS 1000

void i2c_init() {
  i2c_config_t conf;
  conf.mode = I2C_MODE_MASTER;
  conf.sda_io_num = MMA8452Q_SDA;
  conf.scl_io_num = MMA8452Q_SCL;
  conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
  conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
  conf.master.clk_speed = 100000;
  i2c_param_config(I2C_NUM_0, &conf);
  i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
}

void MMA8452Q_init() {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();

  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (MMA8452Q_ADDR << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, 0x2A, true);
  i2c_master_write_byte(cmd, 0x00, true); // StandBy mode
  i2c_master_stop(cmd);
  i2c_master_cmd_begin(I2C_NUM_0, cmd, portMAX_DELAY);
  i2c_cmd_link_delete(cmd);

  vTaskDelay(300 / portTICK_PERIOD_MS);

  cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (MMA8452Q_ADDR << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, 0x2A, true);
  i2c_master_write_byte(cmd, 0x01, true); // Active mode
  i2c_master_stop(cmd);
  i2c_master_cmd_begin(I2C_NUM_0, cmd, portMAX_DELAY);
  i2c_cmd_link_delete(cmd);

  vTaskDelay(300 / portTICK_PERIOD_MS);

  cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (MMA8452Q_ADDR << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, 0x0E, true);
  i2c_master_write_byte(cmd, 0x00, true); // Set range to +/- 2g
  i2c_master_stop(cmd);
  i2c_master_cmd_begin(I2C_NUM_0, cmd, portMAX_DELAY);
  i2c_cmd_link_delete(cmd);
}

void readAccel(int16_t *AccelX, int16_t *AccelY, int16_t *AccelZ) {
  uint8_t data[7];
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (MMA8452Q_ADDR << 1) | I2C_MASTER_READ, true);
  i2c_master_read(cmd, data, 7, I2C_MASTER_LAST_NACK);
  i2c_master_stop(cmd);
  i2c_master_cmd_begin(I2C_NUM_0, cmd, portMAX_DELAY);
  i2c_cmd_link_delete(cmd);
  
  int16_t x, y, z;
  x = ((data[1] << 8) | data[2]) / 16;
  if (x > 2047) {
    x -= 4096;
  }

  y = ((data[3] << 8) | data[4]) / 16;
  if (y > 2047) {
    y -= 4096;
  }

  z = ((data[5] << 8) | data[6]) / 16;
  if (z > 2047) {
    z -= 4096;
  }

  *AccelX = x;
  *AccelY = y;
  *AccelZ = z;
}

void app_main() {
  i2c_init();
  MMA8452Q_init();

  while (1) {
    int16_t xAccl, yAccl, zAccl;
    readAccel(&xAccl, &yAccl, &zAccl);

    printf("--------------------\n");
    printf("Acceleration in X-Axis : %d\n", xAccl);
    printf("Acceleration in Y-Axis : %d\n", yAccl);
    printf("Acceleration in Z-Axis : %d\n", zAccl);

    vTaskDelay(DELAY_MS / portTICK_PERIOD_MS);
  }
}