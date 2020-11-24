/*
 * Occupancy sensor for HomeKit based on HC-SR501.
 */

#include <stdio.h>
#include <esp/uart.h>

#include <homekit/homekit.h>
#include <homekit/characteristics.h>

#include <toggle.h>
#include <wifi_config.h>


#define SENSOR_PIN 4
#ifndef SENSOR_PIN
#error SENSOR_PIN is not specified
#endif
#define SENSOR_PIN_2 5
#ifndef SENSOR_PIN_2
#error SENSOR_PIN_2 is not specified
#endif


void door_identify(homekit_value_t _value) {
    printf("door identify\n");
}


homekit_characteristic_t door_open_detected = HOMEKIT_CHARACTERISTIC_(CONTACT_SENSOR_STATE, 0);
homekit_characteristic_t door_2_open_detected = HOMEKIT_CHARACTERISTIC_(CONTACT_SENSOR_STATE, 0);


void sensor_callback(bool high, void *context) {
    door_open_detected.value = HOMEKIT_UINT8(high ? 1 : 0);
    homekit_characteristic_notify(&door_open_detected, door_open_detected.value);
}

void sensor_2_callback(bool high, void *context) {
    door_2_open_detected.value = HOMEKIT_UINT8(high ? 1 : 0);
    homekit_characteristic_notify(&door_2_open_detected, door_2_open_detected.value);
}



homekit_accessory_t *accessories[] = {
    HOMEKIT_ACCESSORY(
        .id=1,
        .category=homekit_accessory_category_sensor,
        .services=(homekit_service_t*[]) {
            HOMEKIT_SERVICE(
                ACCESSORY_INFORMATION,
                .characteristics=(homekit_characteristic_t*[]) {
                    HOMEKIT_CHARACTERISTIC(NAME, "Contact Sensor"),
                    HOMEKIT_CHARACTERISTIC(MANUFACTURER, "ObjP"),
                    HOMEKIT_CHARACTERISTIC(SERIAL_NUMBER, "2012345"),
                    HOMEKIT_CHARACTERISTIC(MODEL, "DS1"),
                    HOMEKIT_CHARACTERISTIC(FIRMWARE_REVISION, "0.1"),
                    HOMEKIT_CHARACTERISTIC(IDENTIFY, door_identify),
                    NULL
                },
            ),
            HOMEKIT_SERVICE(
                CONTACT_SENSOR,
                .primary=true,
                .characteristics=(homekit_characteristic_t*[]) {
                    HOMEKIT_CHARACTERISTIC(NAME, "Kontakt"),
                    &door_open_detected,
                    NULL
                }),
            HOMEKIT_SERVICE(
                CONTACT_SENSOR,
                .primary=false,
                .characteristics=(homekit_characteristic_t*[]) {
                HOMEKIT_CHARACTERISTIC(NAME, "Kontakt2"),
                &door_2_open_detected,
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

    wifi_config_init2("occupancy-sensor", NULL, on_wifi_config_event);

    if (toggle_create(SENSOR_PIN, sensor_callback, NULL)) {
        printf("Failed to initialize sensor\n");
    }
    if (toggle_create(SENSOR_PIN_2, sensor_2_callback, NULL)) {
        printf("Failed to initialize sensor\n");
    }
}
