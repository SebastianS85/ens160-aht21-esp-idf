#include <stdio.h>
#include "driver/i2c.h"
#include "esp_log.h"
#include "aht21.h"
#include "ens160.h"


    

static const char *TAG = "en160_sensor";



void app_main(void) {
    i2c_master_init();
    set_ens160_opmode();
   
    while (true) {

       
       get_sensor_data();
        vTaskDelay(2000/ portTICK_PERIOD_MS);  // Wait 5 seconds between readings
    }
}
