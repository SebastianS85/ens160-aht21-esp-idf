#include <stdio.h>
#include "driver/i2c.h"
#include "esp_log.h"
#include "aht21.h"
#include "ens160.h"





void app_main(void) {
    i2c_master_init();
    set_ens160_opmode();
   
    while (true) {

       
       get_sensor_data();
        vTaskDelay(2000/ portTICK_PERIOD_MS);  
    }
}
