#include <stdio.h>
#include <stdlib.h>
#include <espressif/esp_wifi.h>
#include "espressif/esp_common.h"  //??????
#include <espressif/esp_sta.h>
#include <esp/uart.h>
#include <esp8266.h>
#include <FreeRTOS.h>
#include <task.h>
//#include <math.h>




#include <homekit/homekit.h>
#include <homekit/characteristics.h>
#include <wifi_config.h>

#include "i2c/i2c.h"
#include <bh1750/bh1750.h>
#include "button.h"


#define SCL_PIN 5 // Wemos D1
#define SDA_PIN 4 // Wemos D2
#define I2C_BUS 0


//#define BH1750_ADDR_LO 0x23 // ADDR pin floating/low
//#define BH1750_ADDR_HI 0x5c


#define  LED_INBUILT_GPIO 2 // The GPIO pin that is oconnected to built-in LED on ESP12F or D4 on wemos D1 mini.
#define SENSOR_POLL_PERIOD 10000  // reading time

#define FIRMWARE_VERSION "1.0.0"
#define MANUFACTURER_DEFAULT "HomeKid™"
#define MODEL_DEFAULT " LightSensor"


// The GPIO pin that is oconnected to the button on the Sonoff Basic.
const int button_gpio = 0;

bool bh1750_reading = true;


void button_callback(uint8_t gpio, button_event_t event);

void reset_configuration_task() {
    //Flash the LED first before we start the reset

    for (int i = 0; i<5; i++) {
            gpio_write(LED_INBUILT_GPIO, false);
            vTaskDelay(250 / portTICK_PERIOD_MS);
            gpio_write(LED_INBUILT_GPIO, true);
            vTaskDelay(250 / portTICK_PERIOD_MS);
        }

    
    printf("Resetting Wifi Config\n");
    
    wifi_config_reset();
    
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    
    printf("Resetting HomeKit Config\n");
    
    homekit_server_reset();
    
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    
    printf("Restarting\n");
    
    sdk_system_restart();
    
    vTaskDelete(NULL);
}

void reset_configuration() {
    printf("Resetting configuration\n");
    xTaskCreate(reset_configuration_task, "Reset configuration", 256, NULL, 2, NULL); 
}

void button_callback(uint8_t gpio, button_event_t event) {
    switch (event) {
        case button_event_long_press:
            reset_configuration();
            break;
        default:
            printf("Unknown button event: %d\n", event);
    }
}


void identify_task(void *_args) {

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            gpio_write(LED_INBUILT_GPIO, false);
            vTaskDelay(100 / portTICK_PERIOD_MS);
            gpio_write(LED_INBUILT_GPIO, true);
            vTaskDelay(100 / portTICK_PERIOD_MS);
        }
        vTaskDelay(250 / portTICK_PERIOD_MS);
    }

    vTaskDelete(NULL);
}


void sensor_identify(homekit_value_t _value) {
    printf("LightSensor identify\n");
    xTaskCreate(identify_task, "Identify", 128, NULL, 2, NULL);
}


homekit_characteristic_t lux = HOMEKIT_CHARACTERISTIC_(CURRENT_AMBIENT_LIGHT_LEVEL, 0, .min_step = (float[]) {0.01}, .min_value = (float[]) {0}, .max_value = (float[]) {100000}); // 
homekit_characteristic_t fault = HOMEKIT_CHARACTERISTIC_(STATUS_FAULT, 0);
///homekit_characteristic_t motion_detected = HOMEKIT_CHARACTERISTIC_(MOTION_DETECTED, 0);

void sensor_task(void *_args) {
   
    float lux_value;

       i2c_dev_t dev = {
        .addr = BH1750_ADDR_LO,

        .bus = I2C_BUS,
    };
    bh1750_configure(&dev, BH1750_CONTINUOUS_MODE | BH1750_HIGH_RES_MODE2);  //BH1750_HIGH_RES_MODE = 1lux, BH1750_HIGH_RES_MODE2  = 0.5lux resolution

    while (1) {

        bool success = bh1750_read(&dev);
                while (1) {



        if (success) {
            bh1750_reading = true;
            if (fault.value.int_value != 0){
                fault.value = HOMEKIT_UINT8(0);
                homekit_characteristic_notify(&fault, fault.value); }
            lux_value = bh1750_read(&dev);
            printf("Lux: %.1f\n", lux_value);
            lux.value = HOMEKIT_FLOAT(lux_value);
            homekit_characteristic_notify(&lux, lux.value);


    }

        else {
            bh1750_reading = false;
            if (fault.value.int_value != 1){
                fault.value = HOMEKIT_UINT8(1);
                homekit_characteristic_notify(&fault, fault.value);
                }
            
            printf("Couldnt read data from sensor\n");
        }


 
                vTaskDelay(SENSOR_POLL_PERIOD / portTICK_PERIOD_MS);

                }
        }
}





void sensor_init() {
    xTaskCreate(sensor_task, "Sensor Task", 256, NULL, 2, NULL);
}

homekit_characteristic_t manufacturer = HOMEKIT_CHARACTERISTIC_(MANUFACTURER,  MANUFACTURER_DEFAULT);
homekit_characteristic_t model        = HOMEKIT_CHARACTERISTIC_(MODEL, MODEL_DEFAULT);
homekit_characteristic_t revision     = HOMEKIT_CHARACTERISTIC_(FIRMWARE_REVISION,  FIRMWARE_VERSION);
homekit_characteristic_t name = HOMEKIT_CHARACTERISTIC_(NAME, NULL);
homekit_characteristic_t serial = HOMEKIT_CHARACTERISTIC_(SERIAL_NUMBER, NULL);


homekit_accessory_t *accessories[] = {
    HOMEKIT_ACCESSORY(.id=1, .category=homekit_accessory_category_sensor, .services=(homekit_service_t*[]) {
        HOMEKIT_SERVICE(ACCESSORY_INFORMATION, .characteristics=(homekit_characteristic_t*[]) {
            &name,
            &manufacturer,
            &serial,
            &model,
            &revision,
            HOMEKIT_CHARACTERISTIC(IDENTIFY, sensor_identify),
            NULL
        }),
            HOMEKIT_SERVICE(LIGHT_SENSOR, .primary=true,  .characteristics=(homekit_characteristic_t*[]) {
            HOMEKIT_CHARACTERISTIC(NAME, "Ambient Light Sensor"),
            &lux,
            &fault,
            NULL
        }),

        NULL
    }),
    NULL
};


homekit_server_config_t config = {
    .accessories = accessories,
    .password = "100-00-100",
    .setupId="HK92" /// homekit_accessory_category_sensor = 10,
};

void create_accessory_name() {
    uint8_t macaddr[6];
    sdk_wifi_get_macaddr(STATION_IF, macaddr);

    int name_len = snprintf(NULL, 0, "LightSensor-%02X%02X%02X",
                            macaddr[3], macaddr[4], macaddr[5]);
    char *name_value = malloc(name_len+1);
    snprintf(name_value, name_len+1, "LightSensor-%02X%02X%02X",
             macaddr[3], macaddr[4], macaddr[5]);

    name.value = HOMEKIT_STRING(name_value);
    
    char *serial_value = malloc(13);
    snprintf(serial_value, 13, "%02X%02X%02X%02X%02X%02X", macaddr[0], macaddr[1], macaddr[2], macaddr[3], macaddr[4], macaddr[5]);
    serial.value = HOMEKIT_STRING(serial_value);


}

void on_wifi_ready() {
    create_accessory_name();
    gpio_enable(LED_INBUILT_GPIO, GPIO_OUTPUT);        // initialise the onboard led as a secondary indicator (handy for testing)
    gpio_write(LED_INBUILT_GPIO, true); 
    homekit_server_init(&config);
}


void user_init(void) {
    uart_set_baud(0, 115200);
    i2c_init(I2C_BUS, SCL_PIN, SDA_PIN, I2C_FREQ_100K);
    sensor_init();
    wifi_config_init("HomeKid", "12345678", on_wifi_ready);



            if (button_create(button_gpio, 0, 10000, button_callback)) {
        printf("Failed to initialize button\n");
    }

}

