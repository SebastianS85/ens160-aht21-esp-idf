/**
 * @brief Read data from the ENS160 sensor over I2C.
 *
 * This function reads data from the ENS160 sensor at the specified register address.
 * It uses the I2C master interface to communicate with the sensor.
 *
 * @param reg_addr The register address to read from.
 * @param data Pointer to a buffer to store the read data.
 * @param len The number of bytes to read.
 * @return ESP_OK on success, otherwise an error code.
 */
esp_err_t ens160_read_data(uint8_t reg_addr, uint8_t *data, size_t len);
#include <stdio.h>
#include "ens160.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "aht21.h"

#define TAG "ens160_sensor"
ens160_data_t sensor_data;
esp_err_t ens160_write_data(uint8_t reg_addr, uint8_t *data, size_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (EN160_I2C_ADDRESS  << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);  // Write register address
    i2c_master_write(cmd, data, len, true);      // Write data buffer
    i2c_master_stop(cmd);
    esp_err_t result = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return result;
}

/**
 * Read data from the ENS160 sensor over I2C.
 *
 * This function reads data from the ENS160 sensor at the specified register address.
 * It uses the I2C master interface to communicate with the sensor.
 *
 * @param reg_addr The register address to read from.
 * @param data Pointer to a buffer to store the read data.
 * @param len The number of bytes to read.
 * @return ESP_OK on success, otherwise an error code.
 */
esp_err_t ens160_read_data(uint8_t reg_addr, uint8_t *data, size_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (EN160_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (EN160_I2C_ADDRESS << 1) | I2C_MASTER_READ, true);
    if (len > 1) {
        i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t result = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return result;
}


void get_part_id(){
    uint8_t data[2];
    if (ens160_read_data(0x00, data, sizeof(data)) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read data from ENS160");
    }
   
     for(int i = 0; i < 2; i++) {
            ESP_LOGW(TAG, "Data[%d]: 0x%02X", i, data[i]);
        }
}

void get_opmode(){
    uint8_t data[1];
    if (ens160_read_data(OPMODE , data, sizeof(data)) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read data from ENS160");
    }
        ESP_LOGW(TAG, "Data[0]: 0x%02X", data[0]);
}

void set_ens160_opmode(){
    uint8_t data[1] = {OPMODE_MODE_STANDART};
    if (ens160_write_data(0x10, data, sizeof(data)) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write data to ENS160");
    }
    ESP_LOGI(TAG, "Data written to ENS160");
}

uint8_t get_ens160_aqi(){
    uint8_t data[1];
    if (ens160_read_data(DATA_AQI, data, sizeof(data)) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read data from ENS160");
    }
    
        
     switch (data[0]) {
        case 1:
            ESP_LOGI(TAG,"AQI : \033[34m[Excellent]\033[0m ");
            break;
        case 2:
            ESP_LOGI(TAG,"AQI : \033[32m[Good]\033[0m\n ");
            break;
        case 3:
            ESP_LOGI(TAG,"AQI : \033[33mModerate\033[0m");
            break;
        case 4:
            ESP_LOGI(TAG,"AQI :\033[38;5;214m Poor\033[0m");
            break;
        case 5:
            ESP_LOGI(TAG,"AQI :\033[31m[Unhealthy]\033[0m\n");
            break;
        default:
            ESP_LOGI(TAG,"VALIDITY: Unknown status\n");
            break;
        
    }
    return data[0];

}

/**
 * @brief Get the TVOC (Total Volatile Organic Compounds) value from the ENS160 sensor.
 *
 * This function reads the TVOC data from the ENS160 sensor and returns the value. It also logs the TVOC value along with an air quality indicator based on the following ranges:
 * - TVOC < 600 PPB: [Excellent]
 * - 600 PPB <= TVOC < 800 PPB: [Good]
 * - 800 PPB <= TVOC < 1000 PPB: [Moderate]
 * - 1000 PPB <= TVOC < 1500 PPB: [Poor]
 * - TVOC >= 1500 PPB: [Unhealthy]
 *
 * @return uint16_t The TVOC value in PPB (parts per billion).
 */
/**
 * @brief Get the TVOC (Total Volatile Organic Compounds) value from the ENS160 sensor.
 *
 * This function reads the TVOC data from the ENS160 sensor and returns the value. It also logs the TVOC value along with an air quality indicator based on the following ranges:
 * - TVOC < 600 PPB: [Excellent]
 * - 600 PPB <= TVOC < 800 PPB: [Good]
 * - 800 PPB <= TVOC < 1000 PPB: [Moderate]
 * - 1000 PPB <= TVOC < 1500 PPB: [Poor]
 * - TVOC >= 1500 PPB: [Unhealthy]
 *
 * @return uint16_t The TVOC value in PPB (parts per billion).
 */
/**
 * @brief Get the TVOC (Total Volatile Organic Compounds) value from the ENS160 sensor.
 *
 * This function reads the TVOC data from the ENS160 sensor and returns the value. It also logs the TVOC value along with an air quality indicator based on the following ranges:
 * - TVOC < 600 PPB: [Excellent]
 * - 600 PPB <= TVOC < 800 PPB: [Good]
 * - 800 PPB <= TVOC < 1000 PPB: [Moderate]
 * - 1000 PPB <= TVOC < 1500 PPB: [Poor]
 * - TVOC >= 1500 PPB: [Unhealthy]
 *
 * @return uint16_t The TVOC value in PPB (parts per billion).
 */
uint16_t get_ens160_tvoc(){


    uint8_t data[2];
    if (ens160_read_data(0x22, data, sizeof(data)) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read data from ENS160");
    }

   uint16_t tvoc = (data[1] << 8) | data[0];

   if(tvoc < 600){
        ESP_LOGI(TAG,"TOVC  PPB Value: %d  \033[34m[Excellent]\033[0m",tvoc);
    }
    else if(tvoc > 600 && tvoc < 800){
        ESP_LOGI(TAG,"TOVC  PPB Value: %d  \033[32m[[Good]\033[0m",tvoc);
    }
    else if(tvoc > 800 && tvoc < 1000){
        ESP_LOGI(TAG,"TOVC  PPB Value: %d \033[33m[Moderate]\033[0m",tvoc);
    }
    else if(tvoc >1000 && tvoc < 1500){
        ESP_LOGI(TAG,"TOVC  PPB Value: %d  \033[38;5;214m[Poor]\033[0m",tvoc);
    }
    else if(tvoc > 1500){
        ESP_LOGI(TAG,"TOVC  PPB Value: %d  \033[31m[Unhealthy]\033[0m\n",tvoc);
    }
   
   return tvoc;

}


uint16_t get_ens160_eco2(){
    uint8_t data[2];
    if (ens160_read_data(0x24, data, sizeof(data)) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read data from ENS160");
    }
    uint16_t eco2 = (data[1] << 8) | data[0];
    ESP_LOGE(TAG, "ECO2  PPM Value: %d",eco2);


    uint8_t data3[2];
     if (ens160_read_data(0x30, data3, sizeof(data3)) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read data from ENS160");
    }
   return eco2;
}



/**
 * @brief Get the status of the ENS160 sensor.
 *
 * This function reads the status register of the ENS160 sensor and interprets the validity field.
 * It logs the status to the ESP_LOG system and returns the validity value.
 *
 * @return uint8_t The validity value, which can be one of the following:
 *         - 0: Normal operation
 *         - 1: Warm-Up phase
 *         - 2: Initial Start-Up phase
 *         - 3: Invalid output
 *         - Any other value: Unknown status
 */
uint8_t get_ens160_status(){
    uint8_t data[1];
    if (ens160_read_data(0x20, data, sizeof(data)) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read data from ENS160");
    }
    
    uint8_t validity = (data[0]& 0x0C) >> 2;

    // Interpret the validity field
    switch (validity) {
        case 0:
            ESP_LOGI(TAG,"VALIDITY: Normal operation\n");
            break;
        case 1:
            ESP_LOGI(TAG,"VALIDITY: Warm-Up phase\n");
            break;
        case 2:
            ESP_LOGI(TAG,"VALIDITY: Initial Start-Up phase\n");
            break;
        case 3:
            ESP_LOGI(TAG,"VALIDITY: Invalid output\n");
            break;
        default:
            ESP_LOGI(TAG,"VALIDITY: Unknown status\n");
            break;
        
    }
    return validity;
}
/**
 * @brief Compensate temperature and humidity data for the ENS160 sensor.
 *
 * This function reads the temperature and humidity data from the AHT21 sensor, converts the temperature
 * to the format expected by the ENS160 sensor, and writes the data to the ENS160 sensor.
 */
void compensate_temp_hum(void){
    aht21_data_t aht21_data = aht21_get_temp_hum();
    uint16_t temp = (uint16_t)(aht21_data.temperature + 273)*64 ;
    uint32_t hum = aht21_data.humidity;

    uint8_t data[2] = { temp &0xFF,(uint8_t)(temp>> 8)};

     if (ens160_write_data(0x30, data, sizeof(data)) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write data to ENS160");
    }

}

/**
 * @brief Get sensor data from the ENS160 and AHT21 sensors.
 *
 * This function reads the temperature, humidity, air quality index (AQI), total volatile organic compounds (TVOC),
 * and equivalent carbon dioxide (ECO2) from the ENS160 sensor, as well as the temperature and humidity from the
 * AHT21 sensor. The sensor data is stored in the `sensor_data` struct and returned.
 *
 * @return ens160_data_t The sensor data.
 */
ens160_data_t get_sensor_data(void){

    aht21_data_t aht21_data = aht21_get_temp_hum();
    sensor_data.temp= aht21_data.temperature;
    sensor_data.hum = aht21_data.humidity;
    sensor_data.aqi = get_ens160_aqi();
    sensor_data.tvoc = get_ens160_tvoc();
    sensor_data.eco2 = get_ens160_eco2();
    sensor_data.status = get_ens160_status();
    return sensor_data;
}