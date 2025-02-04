#ifndef ENS160_H
#define ENS160_H
#define EN160_I2C_ADDRESS 0x53
#define PART_ID 0x00
#define OPMODE 0x10
#define OPMODE_MODE_STANDART 0x02
#define DATA_STATUS 0x20
#define DATA_AQI 0x21
#define DATA_TVOC 0x22
#define TEMP_IN 0x13
#include "driver/i2c.h"


static const uint8_t PART_ID_CMD[] = {0x33, 0x00};
esp_err_t ens160_read_data(uint8_t reg_addr, uint8_t *data, size_t len);

typedef struct {

    uint8_t status;
    uint8_t hum;
    uint8_t temp;
    uint8_t aqi;
    uint16_t eco2;
    uint16_t tvoc;
    

}ens160_data_t;

void get_part_id(void); 
void get_opmode(void);
void set_ens160_opmode(void);
uint8_t get_ens160_aqi(void);
uint16_t get_ens160_tvoc(void);
void get_ens160_data(void);
uint16_t get_ens160_eco2(void);
void compensate_temp_hum(void);
uint8_t get_ens160_status(void);
ens160_data_t get_sensor_data(void);



#endif
