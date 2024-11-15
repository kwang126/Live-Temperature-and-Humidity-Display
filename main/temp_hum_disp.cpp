#include <stdio.h>
#include <iomanip>
#include <sstream>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"

#include "DFRobot_LCD.h"

// I2C Configuration
#define I2C_MASTER_SCL_IO 8        // GPIO for I2C SCL
#define I2C_MASTER_SDA_IO 10        // GPIO for I2C SDA
#define I2C_MASTER_NUM I2C_NUM_0    // I2C port number
#define I2C_MASTER_FREQ_HZ 100000   // I2C master clock frequency
#define CMD_MEASURE 0x7CA2  // Measure Command


// Sensor I2C Address
#define SENSOR_I2C_ADDRESS 0x70

extern "C" {

void i2c_master_init() {
    i2c_config_t conf;
    conf.clk_flags = 0;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}


esp_err_t i2c_master_read_sensor(uint16_t mes_cmd, uint8_t *data, size_t size) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SENSOR_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, mes_cmd >> 8, true);
    i2c_master_write_byte(cmd, mes_cmd & 0xFF, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);

    if (ret != ESP_OK) {
        return ret;
    }

    vTaskDelay(20 / portTICK_PERIOD_MS);

    i2c_cmd_link_delete(cmd);
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SENSOR_I2C_ADDRESS << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, size, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}


void app_main() {
    i2c_master_init();
    DFRobot_LCD disp = DFRobot_LCD(16, 2);
    disp.init();
    disp.setRGB(0, 255, 0); 

    while (1) {
        uint8_t sensor_data[6] = {0,};
        esp_err_t err = i2c_master_read_sensor(CMD_MEASURE, sensor_data, 6);
        
        if (err == ESP_OK) {
            uint16_t temp_raw = (sensor_data[0] << 8) | sensor_data[1];
            uint16_t hum_raw = (sensor_data[3] << 8) | sensor_data[4];

            float temperature_c = 175.0 * ((float)temp_raw / 65536.0) - 45.0;
//            float temperature_f = (temperature_c * 9.0/5.0) + 32.0;
            float humidity = 100.0 * ((float)hum_raw / 65536.0);   // 65536 is 2^16

            // Forming temperature string
            std::stringstream temp_stream;
            temp_stream << std::fixed << std::setprecision(1) << temperature_c;
            std::string temp_str = "Temp: " + temp_stream.str() + "C";
            
            // Forming humidity string
            std::stringstream hum_stream;
            hum_stream << std::fixed << std::setprecision(1) << humidity;
            std::string hum_str = "Humidity: " + hum_stream.str() + "%";
            

            disp.printstr(temp_str.c_str());
            disp.setCursor(0, 1);
            disp.printstr(hum_str.c_str());
            disp.setCursor(0, 0);
        } else {
            printf("Failed to read sensor data\n");
        }


        vTaskDelay(1000 / portTICK_PERIOD_MS); // Wait for 1 second
    }
}

}
