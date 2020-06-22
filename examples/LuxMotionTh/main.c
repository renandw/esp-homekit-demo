#include <stdio.h>
#include <stdlib.h>
#include <espressif/esp_wifi.h>
#include "espressif/esp_common.h"  //??????
#include <espressif/esp_sta.h>
#include <esp/uart.h>
#include <esp8266.h>
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <string.h>
//#include <math.h>




#include <homekit/homekit.h>
#include <homekit/characteristics.h>
#include <wifi_config.h>

#include "i2c/i2c.h"
#include <bh1750/bh1750.h>
#include "button.h"
#include <dht/dht.h>


#define SCL_PIN 5 // Wemos D1
#define SDA_PIN 4 // Wemos D2
#define I2C_BUS 0
#define MOTION_SENSOR_GPIO 13
#define LED_GPIO  = LED_INBUILT_GPIO
#define SENSOR_PIN 12
#ifndef SENSOR_PIN
#error SENSOR_PIN is not specified
#endif


//#define BH1750_ADDR_LO 0x23 // ADDR pin floating/low
//#define BH1750_ADDR_HI 0x5c



#define LED_INBUILT_GPIO 2 // The GPIO pin that is oconnected to built-in LED on ESP12F or D4 on wemos D1 mini.
#define SENSOR_POLL_PERIOD 3000  // reading time



homekit_characteristic_t motion_detected  = HOMEKIT_CHARACTERISTIC_(MOTION_DETECTED, 0);
homekit_characteristic_t temperature = HOMEKIT_CHARACTERISTIC_(CURRENT_TEMPERATURE, 0);
homekit_characteristic_t humidity    = HOMEKIT_CHARACTERISTIC_(CURRENT_RELATIVE_HUMIDITY, 0);

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

void identify() {
    xTaskCreate(identify_task, "identify", 256, NULL, 2, NULL);
}

void temperature_sensor_identify(homekit_value_t _value) {
    printf("Temperature sensor identify\n");
}

void temperature_sensor_task(void *_args) {

    float humidity_value, temperature_value;
    while (1) {
        bool success = dht_read_float_data(
            DHT_TYPE_DHT22, SENSOR_PIN,
            &humidity_value, &temperature_value
        );
        if (success) {
            temperature.value.float_value = temperature_value;
            humidity.value.float_value = humidity_value;

            homekit_characteristic_notify(&temperature, HOMEKIT_FLOAT(temperature_value));
            homekit_characteristic_notify(&humidity, HOMEKIT_FLOAT(humidity_value));
        } else {
            printf("Couldnt read data from sensor\n");
        }

        vTaskDelay(3000 / portTICK_PERIOD_MS);
    }
}
void temperature_sensor_init() {
    xTaskCreate(temperature_sensor_task, "Temperature Sensor", 256, NULL, 2, NULL);
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


void motion_sensor_callback(uint8_t gpio) {
    if (gpio == MOTION_SENSOR_GPIO){
        int new = 0;
        new = gpio_read(MOTION_SENSOR_GPIO);
        if (new) {
            printf("Motion detected\n");
            identify();	
        }
        motion_detected.value = HOMEKIT_BOOL(new);
        homekit_characteristic_notify(&motion_detected, HOMEKIT_BOOL(new));
    }
    else {
        printf("Interrupt on %d", gpio);
    }
}

homekit_characteristic_t name = HOMEKIT_CHARACTERISTIC_(NAME, NULL);
homekit_characteristic_t serial = HOMEKIT_CHARACTERISTIC_(SERIAL_NUMBER, NULL);

homekit_accessory_t *accessories[] = {
    HOMEKIT_ACCESSORY(
        .id=1,
        .category=homekit_accessory_category_sensor,
        .services=(homekit_service_t*[]) {
            HOMEKIT_SERVICE(
                ACCESSORY_INFORMATION,
                .characteristics=(homekit_characteristic_t*[]) {
                    HOMEKIT_CHARACTERISTIC(NAME, "Lux Sensor"),
                    HOMEKIT_CHARACTERISTIC(MANUFACTURER, "renandw"),
                    &serial,
                    HOMEKIT_CHARACTERISTIC(MODEL, "MyButton"),
                    HOMEKIT_CHARACTERISTIC(FIRMWARE_REVISION, "0.1"),
                    HOMEKIT_CHARACTERISTIC(IDENTIFY, sensor_identify),
                    NULL
                },
            ),
            HOMEKIT_SERVICE(LIGHT_SENSOR, .primary=true,  .characteristics=(homekit_characteristic_t*[]) {
                    HOMEKIT_CHARACTERISTIC(NAME, "Ambient Light Sensor"),
                    &lux,
                    &fault,
                    NULL
                },
            ),
            NULL
        },
    ),
    HOMEKIT_ACCESSORY(.id=2, .category=homekit_accessory_category_switch, .services=(homekit_service_t*[]){
    HOMEKIT_SERVICE(ACCESSORY_INFORMATION, .characteristics=(homekit_characteristic_t*[]){
        HOMEKIT_CHARACTERISTIC(NAME, "Temperature Sensor"),
        HOMEKIT_CHARACTERISTIC(MANUFACTURER, "renandw"),
        &serial,
        HOMEKIT_CHARACTERISTIC(MODEL, "MyButton"),
        HOMEKIT_CHARACTERISTIC(FIRMWARE_REVISION, "0.1"),
        HOMEKIT_CHARACTERISTIC(IDENTIFY, identify),
        NULL
    }),
    HOMEKIT_SERVICE(MOTION_SENSOR, .primary=true, .characteristics=(homekit_characteristic_t*[]){
        HOMEKIT_CHARACTERISTIC(NAME, "Motion Sensor"),
        &motion_detected,
        NULL
    }),
    NULL
}),
    HOMEKIT_ACCESSORY(.id=3, .category=homekit_accessory_category_thermostat, .services=(homekit_service_t*[]) {
        HOMEKIT_SERVICE(ACCESSORY_INFORMATION, .characteristics=(homekit_characteristic_t*[]) {
            HOMEKIT_CHARACTERISTIC(NAME, "Temperature Sensor"),
            HOMEKIT_CHARACTERISTIC(MANUFACTURER, "renandw"),
            &serial,
            HOMEKIT_CHARACTERISTIC(MODEL, "MyButton"),
            HOMEKIT_CHARACTERISTIC(FIRMWARE_REVISION, "0.1"),
            HOMEKIT_CHARACTERISTIC(IDENTIFY, temperature_sensor_identify),
            NULL
        }),
        HOMEKIT_SERVICE(TEMPERATURE_SENSOR, .primary=true, .characteristics=(homekit_characteristic_t*[]) {
            HOMEKIT_CHARACTERISTIC(NAME, "Temperature Sensor"),
            &temperature,
            NULL
        }),
        HOMEKIT_SERVICE(HUMIDITY_SENSOR, .characteristics=(homekit_characteristic_t*[]) {
            HOMEKIT_CHARACTERISTIC(NAME, "Humidity Sensor"),
            &humidity,
            NULL
        }),
        NULL
    }),
    
    NULL
};

void gpio_init() {
    gpio_enable(MOTION_SENSOR_GPIO, GPIO_INPUT);
    gpio_enable(SENSOR_PIN, GPIO_INPUT);
    //gpio_set_pullup(SENSOR_PIN, false, false);
    //gpio_set_pullup(MOTION_SENSOR_GPIO, false, false);
    gpio_set_interrupt(MOTION_SENSOR_GPIO, GPIO_INTTYPE_EDGE_ANY, motion_sensor_callback);
}

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
    gpio_init();
    temperature_sensor_init();
    wifi_config_init("LuxMotionTH", "12345678", on_wifi_ready);



            if (button_create(button_gpio, 0, 10000, button_callback)) {
        printf("Failed to initialize button\n");
    }

}
