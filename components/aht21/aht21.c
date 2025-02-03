#include <stdio.h>
#include "aht21.h"
#include "driver/i2c.h"
#include "esp_log.h"



static const char *TAG = "AHT21";


esp_err_t aht21_write_data(uint8_t reg_addr, uint8_t *data, size_t len);
esp_err_t aht21_read_data(uint8_t reg_addr, uint8_t *data, size_t len);
aht21_stat_t aht_calibrate(void);

void i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0));


    
}



aht21_stat_t aht21_calibrate(void)
{
    // Placeholder calibration data
    esp_err_t err = aht21_write_data(AHT21_CAL, (uint8_t *)AHT_CALIBBRATION_CMD, sizeof(AHT_CALIBBRATION_CMD));
    if (err == ESP_OK)
    {
        ESP_LOGI(TAG, "Sensor calibrated");
        return AHT21_OK;
    }
    else
    {
        ESP_LOGE(TAG, "Failed to calibrate sensor: %s", esp_err_to_name(err));
        return AHT21_CAL_ERR;
    }
}

aht21_stat_t aht_start_meas(void)
{
    esp_err_t err = aht21_write_data(AHT_START_MES, (uint8_t *)AHT_MEASUREMENT_CMD, sizeof(AHT_MEASUREMENT_CMD));
    if (err == ESP_OK)
    {
        // ESP_LOGI(TAG, "Sensor measurement started");
        return AHT21_OK;
    }
    else
    {
        ESP_LOGE(TAG, "Failed to start sensor measurement: %s", esp_err_to_name(err));
        return AHT21_READ_ERR;
    }
}

aht21_data_t aht21_get_temp_hum(void) {
    aht21_data_t result = {.status = AHT21_READ_ERR};  // Declare a result structure on the stack
    
    uint8_t data[7];
    
    // Step 1: Read data from the sensor
    if (aht21_read_data(AHT_READ, data, sizeof(data)) != ESP_OK) {
        ESP_LOGE(TAG, "Sensor not responding. Check connection.");
        return result;  // Return early on failure, no need to free memory
    }

    // Step 2: Check if the sensor is calibrated
    if (!(data[0] & (1 << 3))) {
        ESP_LOGE(TAG, "Sensor not calibrated");
        if (aht21_calibrate() != AHT21_OK) {
            ESP_LOGE(TAG, "Calibration failed");
            result.status = AHT21_CAL_ERR;  // Set the status to calibration error
            return result;  // Return early on failure
        }
    }

    // Step 3: Start measurement
    if (aht_start_meas() != AHT21_OK) {
        ESP_LOGE(TAG, "Failed to start measurement");
        result.status = AHT21_READ_ERR;  // Set the status to read error
        return result;  // Return early on failure
    }

    vTaskDelay(80 / portTICK_PERIOD_MS);

    // Step 4: Read data after measurement
    if (aht21_read_data(AHT_READ, data, sizeof(data)) != ESP_OK || (data[0] & (1 << 7))) {
        ESP_LOGE(TAG, "Sensor damaged or busy");
        return result;  // Return early on failure
    }

    // Step 5: Parse the sensor data
    result.temperature = ((((data[3] & 0x0F) << 16) | (data[4] << 8) | data[5]) * AHT21_TEMP_SCALE) / AHT21_RESOLUTION - AHT21_TEMP_OFFSET;
    result.humidity = (((uint32_t)data[1] << 12) | ((uint32_t)data[2] << 4) | (data[3] >> 4)) * AHT21_HUM_SCALE / AHT21_RESOLUTION;
    
    ESP_LOGI(TAG, "Temperature: %2lu C, Humidity: %2lu %%", result.temperature, result.humidity);
    result.status = AHT21_OK;
    
    return result;  // Return the result directly, no need to use dynamic memory
}
esp_err_t aht21_write_data(uint8_t reg_addr, uint8_t *data, size_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ATH_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true); // Write register address
    i2c_master_write(cmd, data, len, true);     // Write data buffer
    i2c_master_stop(cmd);
    esp_err_t result = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return result;
}

esp_err_t aht21_read_data(uint8_t reg_addr, uint8_t *data, size_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ATH_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ATH_I2C_ADDRESS << 1) | I2C_MASTER_READ, true);
    if (len > 1)
    {
        i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t result = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return result;

}


