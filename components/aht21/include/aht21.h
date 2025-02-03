#ifndef AHT21_H
#define AHT21_H
#include "driver/i2c.h"
#include "esp_log.h"

#define ATH_I2C_ADDRESS 0x38 
#define AHT21_CAL 0xE1
#define AHT_READ 0x71
#define AHT_START_MES 0xAC

#define I2C_MASTER_SCL_IO 21    // GPIO number for I2C SCL
#define I2C_MASTER_SDA_IO 20       // GPIO number for I2C SDA
#define I2C_MASTER_NUM I2C_NUM_0   // I2C port number
#define I2C_MASTER_FREQ_HZ 100000  // I2C master clock frequency
#define I2C_MASTER_TIMEOUT_MS 1000 // Timeout for I2C operations
#define AHT21_TEMP_SCALE 200
#define AHT21_HUM_SCALE 100
#define AHT21_TEMP_OFFSET 56
#define AHT21_RESOLUTION 1048576


typedef enum {
    AHT21_OK,
    AHT21_CAL_ERR,
    AHT21_READ_ERR,
}aht21_stat_t;


typedef struct {
   
    uint32_t temperature;
    uint32_t humidity;
    aht21_stat_t status;
} aht21_data_t;

static const uint8_t AHT_MEASUREMENT_CMD[] = {0x33, 0x00};
static const uint8_t AHT_CALIBBRATION_CMD[] = {0x08, 0x00};


void aht21_init(void);
aht21_data_t aht21_get_temp_hum(void);
void i2c_master_init(void);

#endif
