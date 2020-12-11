/*
 * Occupancy sensor for HomeKit based on HC-SR501.
 */

#include <stdio.h>
#include <espressif/esp_wifi.h>
#include <espressif/esp_sta.h>
#include <espressif/esp_common.h>
#include <esp/uart.h>
#include <esp8266.h>
#include <FreeRTOS.h>
#include <task.h>
#include <etstimer.h>
#include <esplibs/libmain.h>

#include <homekit/homekit.h>
#include <homekit/characteristics.h>

#include <toggle.h>
#include <wifi_config.h>

// The GPIO pin that is connected to a LED
const int led_gpio = 2;

#define SENSOR_PIN 4
#ifndef SENSOR_PIN
#error SENSOR_PIN is not specified
#endif
#define SENSOR_PIN_2 5
#ifndef SENSOR_PIN_2
#error SENSOR_PIN_2 is not specified
#endif
#define SENSOR_PIN_3 14
#ifndef SENSOR_PIN_3
#error SENSOR_PIN_3 is not specified
#endif


void led_write(bool on) {
    gpio_write(led_gpio, on ? 0 : 1);
}

void reset_configuration_task() {
    //Flash the LED first before we start the reset
    for (int i=0; i<3; i++) {
        led_write(true);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        led_write(false);
        vTaskDelay(100 / portTICK_PERIOD_MS);
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

void gpio_init() {
    gpio_enable(led_gpio, GPIO_OUTPUT);
    led_write(false);
}

void occupancy_identify(homekit_value_t _value) {
    printf("Occupancy identify\n");
    // We identify the Sensor by Flashing it's LED.
    for (int i=0; i<3; i++) {
        led_write(true);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        led_write(false);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
  led_write(false);
}



homekit_characteristic_t occupancy_detected = HOMEKIT_CHARACTERISTIC_(OCCUPANCY_DETECTED, 0);
homekit_characteristic_t occupancy_detected_2 = HOMEKIT_CHARACTERISTIC_(OCCUPANCY_DETECTED, 0);
homekit_characteristic_t occupancy_detected_3 = HOMEKIT_CHARACTERISTIC_(OCCUPANCY_DETECTED, 0);


void sensor_callback(bool high, void *context) {
    occupancy_detected.value = HOMEKIT_UINT8(high ? 1 : 0);
    homekit_characteristic_notify(&occupancy_detected, occupancy_detected.value);
    for (int i=0; i<3; i++) {
        led_write(true);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        led_write(false);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
  led_write(false);
}

void sensor_callback_2(bool high, void *context) {
    occupancy_detected_2.value = HOMEKIT_UINT8(high ? 1 : 0);
    homekit_characteristic_notify(&occupancy_detected_2, occupancy_detected_2.value);
    for (int i=0; i<3; i++) {
        led_write(true);
        vTaskDelay(200 / portTICK_PERIOD_MS);
        led_write(false);
        vTaskDelay(200 / portTICK_PERIOD_MS);
    }
  led_write(false);
}
void sensor_callback_3(bool high, void *context) {
    occupancy_detected_3.value = HOMEKIT_UINT8(high ? 1 : 0);
    homekit_characteristic_notify(&occupancy_detected_3, occupancy_detected_3.value);
    for (int i=0; i<3; i++) {
        led_write(true);
        vTaskDelay(300 / portTICK_PERIOD_MS);
        led_write(false);
        vTaskDelay(300 / portTICK_PERIOD_MS);
    }
  led_write(false);
}


homekit_accessory_t *accessories[] = {
    HOMEKIT_ACCESSORY(.id=1, .category=homekit_accessory_category_sensor, .services=(homekit_service_t*[]) {
        HOMEKIT_SERVICE(ACCESSORY_INFORMATION, .characteristics=(homekit_characteristic_t*[]) {
            HOMEKIT_CHARACTERISTIC(NAME, "Occupancy Sensor"),
            HOMEKIT_CHARACTERISTIC(MANUFACTURER, "HaPK"),
            HOMEKIT_CHARACTERISTIC(SERIAL_NUMBER, "1"),
            HOMEKIT_CHARACTERISTIC(MODEL, "Occupancy Sensor"),
            HOMEKIT_CHARACTERISTIC(FIRMWARE_REVISION, "1"),
            HOMEKIT_CHARACTERISTIC(IDENTIFY, occupancy_identify),
            NULL
        }),
        HOMEKIT_SERVICE(OCCUPANCY_SENSOR, .primary=true, .characteristics=(homekit_characteristic_t*[]) {
            HOMEKIT_CHARACTERISTIC(NAME, "Occupancy Sensor"),
            &occupancy_detected,
          }),
        NULL
        }),
    HOMEKIT_ACCESSORY(.id=2, .category=homekit_accessory_category_sensor, .services=(homekit_service_t*[]) {
        HOMEKIT_SERVICE(ACCESSORY_INFORMATION, .characteristics=(homekit_characteristic_t*[]) {
            HOMEKIT_CHARACTERISTIC(NAME, "Occupancy Sensor_2"),
            HOMEKIT_CHARACTERISTIC(MANUFACTURER, "HaPK"),
            HOMEKIT_CHARACTERISTIC(SERIAL_NUMBER, "1"),
            HOMEKIT_CHARACTERISTIC(MODEL, "Occupancy Sensor"),
            HOMEKIT_CHARACTERISTIC(FIRMWARE_REVISION, "1"),
            HOMEKIT_CHARACTERISTIC(IDENTIFY, occupancy_identify),
            NULL
            }),    
        HOMEKIT_SERVICE(OCCUPANCY_SENSOR, .primary=false, .characteristics=(homekit_characteristic_t*[]) {
            HOMEKIT_CHARACTERISTIC(NAME, "Occupancy Sensor_2"),
            &occupancy_detected_2,
          }),
        NULL
        }),
    HOMEKIT_ACCESSORY(.id=3, .category=homekit_accessory_category_sensor, .services=(homekit_service_t*[]) {
        HOMEKIT_SERVICE(ACCESSORY_INFORMATION, .characteristics=(homekit_characteristic_t*[]) {
            HOMEKIT_CHARACTERISTIC(NAME, "Occupancy Sensor_3"),
            HOMEKIT_CHARACTERISTIC(MANUFACTURER, "HaPK"),
            HOMEKIT_CHARACTERISTIC(SERIAL_NUMBER, "1"),
            HOMEKIT_CHARACTERISTIC(MODEL, "Occupancy Sensor"),
            HOMEKIT_CHARACTERISTIC(FIRMWARE_REVISION, "1"),
            HOMEKIT_CHARACTERISTIC(IDENTIFY, occupancy_identify),
            NULL
            }),      
        HOMEKIT_SERVICE(OCCUPANCY_SENSOR, .primary=false, .characteristics=(homekit_characteristic_t*[]) {
            HOMEKIT_CHARACTERISTIC(NAME, "Occupancy Sensor_3"),
            &occupancy_detected_3,
            NULL
        }),
        NULL
    }),
    NULL
};


static bool homekit_initialized = false;
static homekit_server_config_t config = {
    .accessories = accessories,
    .password = "111-11-111"
};


void on_wifi_config_event(wifi_config_event_t event) {
    if (event == WIFI_CONFIG_CONNECTED) {
        if (!homekit_initialized) {
            homekit_server_init(&config);
            homekit_initialized = true;
        }
    }
}


void user_init(void) {
    uart_set_baud(0, 115200);
    gpio_init();
    wifi_config_init2("occupancy-sensor", NULL, on_wifi_config_event);

    if (toggle_create(SENSOR_PIN, sensor_callback, NULL)) {
        printf("Failed to initialize sensor\n");
    }
    if (toggle_create(SENSOR_PIN_2, sensor_callback_2, NULL)) {
        printf("Failed to initialize sensor\n");
    }
    if (toggle_create(SENSOR_PIN_3, sensor_callback_3, NULL)) {
        printf("Failed to initialize sensor\n");
    }
}
