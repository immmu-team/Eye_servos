#include <driver/i2c.h>

#define I2C_PORT I2C_NUM_0
#define SDA_PIN 21
#define SCL_PIN 22

void setup() {
    Serial.begin(115200);
    while (!Serial);
    Serial.println("\nI2C Scanner for ESP32");
    
    i2c_config_t config = {};
    config.mode = I2C_MODE_MASTER;
    config.sda_io_num = SDA_PIN;
    config.scl_io_num = SCL_PIN;
    config.sda_pullup_en = GPIO_PULLUP_ENABLE;
    config.scl_pullup_en = GPIO_PULLUP_ENABLE;
    config.master.clk_speed = 100000;
    
    i2c_param_config(I2C_PORT, &config);
    i2c_driver_install(I2C_PORT, config.mode, 0, 0, 0);
}

void loop() {
    Serial.println("\nScanning...");
    int devicesFound = 0;
    
    for (uint8_t address = 1; address < 127; address++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        
        esp_err_t result = i2c_master_cmd_begin(I2C_PORT, cmd, 10 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);
        
        if (result == ESP_OK) {
            Serial.print("I2C device found at 0x");
            Serial.println(address, HEX);
            devicesFound++;
            delay(10);
        }
    }
    
    if (devicesFound == 0) {
        Serial.println("No I2C devices found");
    }
    
    Serial.println("Scan complete");
    delay(5000); // Wait 5 seconds before next scan
}
