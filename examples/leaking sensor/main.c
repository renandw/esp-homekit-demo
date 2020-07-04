#include <stdio.h>
#include <espressif/esp_wifi.h>
#include <espressif/esp_sta.h>
#include <esp/uart.h>
#include <esp8266.h>
#include <FreeRTOS.h>
#include <task.h>

#include <homekit/homekit.h>
#include <homekit/characteristics.h>
#include <wifi_config.h>

#define SENSOR_PIN 5
#ifndef SENSOR_PIN
#error SENSOR_PIN is not specified
#endif


void skimmer_sensor_identify(homekit_value_t _value) {
    printf("Skimmer Control identify\n");
}

homekit_characteristic_t leak_event = HOMEKIT_CHARACTERISTIC_(LEAK_DETECTED, 0);


void skimmer_sensor_task() {
    while (true) {
        gpio_set_pullup(SENSOR_PIN, true, true);
        gpio_set_interrupt(SENSOR_PIN, GPIO_INTTYPE_EDGE_ANY, NULL);
       /* printf(">>>>> %i , PIN: %i ",gpio_read(SENSOR_PIN),SENSOR_PIN); */
        if (gpio_read(SENSOR_PIN)) {
         /*   printf("Libre\n"); */
            homekit_characteristic_notify(&leak_event, HOMEKIT_UINT8(0));
        } else {
            /* printf("Fuga!\n"); */
            homekit_characteristic_notify(&leak_event, HOMEKIT_UINT8(1));
        }

        vTaskDelay(6000 / portTICK_PERIOD_MS);
    }
}

void skimmer_sensor_init() {
    xTaskCreate(skimmer_sensor_task, "Leak Sensor", 256, NULL, 2, NULL);
}


homekit_accessory_t *accessories[] = {
    HOMEKIT_ACCESSORY(
        .id=1,
        .category=homekit_accessory_category_sensor,
        .services=(homekit_service_t*[]) {
            HOMEKIT_SERVICE(
                ACCESSORY_INFORMATION,
                .characteristics=(homekit_characteristic_t*[]) {
                    HOMEKIT_CHARACTERISTIC(NAME, "Controlador de Skimmer"),
                    HOMEKIT_CHARACTERISTIC(MANUFACTURER, "Coderty"),
                    HOMEKIT_CHARACTERISTIC(SERIAL_NUMBER, "201902"),
                    HOMEKIT_CHARACTERISTIC(MODEL, "CLD1"),
                    HOMEKIT_CHARACTERISTIC(FIRMWARE_REVISION, "0.3"),
                    HOMEKIT_CHARACTERISTIC(IDENTIFY, skimmer_sensor_identify),
                    NULL
                },
            ),
            HOMEKIT_SERVICE(
                LEAK_SENSOR,
                .primary=true,
                .characteristics=(homekit_characteristic_t*[]) {
                    HOMEKIT_CHARACTERISTIC(NAME, "Leak Detect"),
                    &leak_event,
                    NULL
                },
            ),
            NULL
        },
    ),
    NULL
};

homekit_server_config_t config = {
    .accessories = accessories,
    .password = "111-11-111"
};

void on_wifi_ready() {
    homekit_server_init(&config);
}

void user_init(void) {
    uart_set_baud(0, 115200);
    wifi_config_init("my-accessory", NULL, on_wifi_ready);
    skimmer_sensor_init();
}
